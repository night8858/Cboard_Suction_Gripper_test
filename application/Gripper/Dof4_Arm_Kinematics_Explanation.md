# 双四自由度机械臂解算代码说明

本文说明 `application/Gripper` 中双四自由度机械臂的几何解算代码，重点解释正运动学、逆运动学、笛卡尔轨迹规划、碰撞检测和一步式控制循环之间的关系。

相关文件：

- `Dof4_Arm.h / Dof4_Arm.c`：机械臂配置、正解、逆解、舵机角度转换、一步式控制循环。
- `Trajectory_Planning.h / Trajectory_Planning.c`：`x/y/z/pitch` 笛卡尔五次多项式轨迹规划。
- `Dof4_Collision.h / Dof4_Collision.c`：双臂连杆胶囊体碰撞检测。

## 1. 建模约定

机械臂参数来源于 `double_arm4_urdf.urdf`。为了降低 STM32F407 上的计算量，代码没有在每个控制周期执行完整 4x4 齐次矩阵数值 IK，而是把 URDF 提取成经典 4DOF 几何模型：

```text
J1：基座绕 Z 轴水平旋转
J2：肩关节，在 J1 决定的垂直平面内运动
J3：肘关节，在同一垂直平面内运动
J4：腕关节，用于满足末端 pitch
```

IK 输入固定为：

```c
Dof4_Pose target = {
    .x = ...,
    .y = ...,
    .z = ...,
    .pitch = ...
};
```

其中：

- `x/y/z` 是 TCP 在 `base_link` 坐标系下的位置，单位 m。
- `pitch` 是末端吸盘在 J2/J3/J4 垂直运动平面内的俯仰角，单位 rad。
- `pitch` 不是旧代码中的平面 `yaw`，也不是每周期从完整姿态矩阵求欧拉角得到的结果，而是低算力几何模型中的末端俯仰约束。

## 2. 核心数据结构

### `Dof4_ArmConfig`

`Dof4_ArmConfig` 保存一条机械臂的全部可配置参数：

- `base[3]`：URDF 中 J1 在 `base_link` 下的位置。
- `base_offset[3]`：用户后续标定用的基座附加偏移。
- `shoulder_r / shoulder_z`：J1 到 J2 的等效偏移。
- `link_len[3]`：J2-J3、J3-J4、J4-TCP 的等效长度。
- `tcp_offset[3]`：吸盘 TCP 标定偏移。
- `pitch_offset`：末端 pitch 零位偏置。
- `joint_min/max[4]`：关节限位，默认来自 URDF。
- `servo_zero / servo_offset / servo_sign / servo_reverse`：舵机标定参数。
- `ws_min/ws_max`：工作空间限制。
- `ik_branch`：逆解多解选择策略。

默认配置由：

```c
Dof4_ArmConfig Dof4_arm_default_config(Dof4_ArmId arm_id);
```

生成。后续如果实机零位、吸盘位置或舵机方向与 URDF 有差异，应优先通过配置或标定 API 修改，不要直接改算法公式。

### `Dof4_Arm`

`Dof4_Arm` 是运行时实例，保存：

- 当前真实反馈关节角 `joint_actual`，可超出 URDF 限位，用于调试和实机姿态观测
- 当前安全目标关节角 `joint_target`，限制在 URDF 限位内
- 当前末端位姿 `current_pose`
- 用户目标位姿 `target_pose`
- 当前/目标舵机步进值
- `joint_world[5][3]`，用于碰撞检测的关节/TCP 世界坐标
- 最近状态码 `last_status`

## 3. 正运动学 FK

正解函数：

```c
Dof4_Status Dof4_arm_forward_kinematics(Dof4_Arm *arm,
                                        const Dof4_JointState *joints,
                                        Dof4_Pose *pose);
```

输入为 4 个关节角，输出 TCP 的 `x/y/z/pitch`。

代码中的主要步骤如下：

1. 计算实际基座位置：

```c
base = cfg.base + cfg.base_offset
```

2. `J1` 决定水平旋转方向：

```c
c1 = cosf(q1);
s1 = sinf(q1);
```

3. `J2/J3/J4` 在同一个垂直平面内累加：

```text
theta2   = -q2
theta3   = -q3
theta4   = -q4
theta23  = theta2 + theta3
theta234 = theta2 + theta3 + theta4
```

代码中使用负号，是为了把 URDF/舵机方向折算到几何平面角定义中。这样几何公式保持直观，同时关节限位仍使用 URDF 的角度范围。

4. 依次计算 J2、J3、J4、TCP 的世界坐标：

```text
J2  = base + Rz(q1) * shoulder
J3  = J2   + Rz(q1) * link2(theta2)
J4  = J3   + Rz(q1) * link3(theta2 + theta3)
TCP = J4   + Rz(q1) * tool(theta2 + theta3 + theta4)
```

5. 末端 pitch：

```c
pose->pitch = theta234 + arm->cfg.pitch_offset;
```

6. 同时更新：

```c
arm->joint_world[0..4]
arm->current_pose
```

其中 `joint_world` 会被碰撞检测模块直接使用。

## 4. 逆运动学 IK

逆解函数：

```c
Dof4_Status Dof4_arm_inverse_kinematics(Dof4_Arm *arm,
                                        const Dof4_Pose *target,
                                        Dof4_JointState *joints);
```

输入为目标 TCP 位姿 `x/y/z/pitch`，输出 4 个关节角。

### 4.1 工作空间检查

函数首先检查目标点是否在该臂工作空间内：

```c
check_workspace(arm, target)
```

如果超出范围，直接返回：

```c
DOF4_STATUS_OUT_OF_WORKSPACE
```

这样可以避免对明显不可达目标继续做三角函数计算。

### 4.2 求 J1

目标点减去基座位置后，在 XY 平面中求方向：

```c
dx = target->x - base[0];
dy = target->y - base[1];
q1 = atan2f(dy, dx);
```

`J1` 的作用是把目标点所在方向确定下来。确定 `J1` 后，后续 `J2/J3/J4` 问题就变成二维平面内的 3R 机械臂问题。

### 4.3 转换到垂直平面

水平距离：

```c
planar_dist = sqrtf(dx * dx + dy * dy);
```

减去 J1 到 J2 的肩部偏移：

```c
r = planar_dist - shoulder_r;
z = target->z - base_z - shoulder_z;
```

此时 `(r, z)` 就是 J2 平面坐标系下 TCP 的目标位置。

### 4.4 由 pitch 回推腕点

末端工具段长度是 `link_len[2]`。由于 TCP 最终要满足目标 pitch，先从 TCP 反推 J4 腕点：

```c
phi = target->pitch - pitch_offset;
wrist_r = r - tool * cosf(phi);
wrist_z = z - tool * sinf(phi);
```

这样 J2/J3 只需要把手臂末端放到腕点 `(wrist_r, wrist_z)`。

### 4.5 余弦定理解 J2/J3

J2-J3 和 J3-J4 构成标准 2R 平面机械臂：

```c
cos_q3 = (wrist_r^2 + wrist_z^2 - L2^2 - L3^2) / (2 * L2 * L3);
```

如果 `cos_q3` 超出 `[-1, 1]`，说明目标不可达：

```c
DOF4_STATUS_IK_UNREACHABLE
```

否则根据肘型分支选择 `sin_q3` 正负：

```c
theta3 = atan2f(elbow_sign * sqrtf(1 - cos_q3^2), cos_q3);
theta2 = atan2f(wrist_z, wrist_r)
       - atan2f(L3 * sin(theta3), L2 + L3 * cos(theta3));
```

### 4.6 由 pitch 求 J4

因为：

```text
theta2 + theta3 + theta4 = pitch - pitch_offset
```

所以：

```c
theta4 = phi - theta2 - theta3;
```

最后转换回关节角：

```c
q2 = -theta2;
q3 = -theta3;
q4 = -theta4;
```

并检查关节限位。

## 5. 多解选择

多解策略在 `Dof4_ArmConfig.ik_branch` 中设置：

```c
DOF4_IK_BRANCH_DEFAULT
DOF4_IK_BRANCH_ELBOW_UP
DOF4_IK_BRANCH_ELBOW_DOWN
DOF4_IK_BRANCH_NEAREST
```

代码会分别尝试肘上和肘下候选解：

```c
solve_ik_candidate(..., +1.0f, &cand_up);
solve_ik_candidate(..., -1.0f, &cand_down);
```

如果选择 `DOF4_IK_BRANCH_NEAREST`，则比较候选解与当前关节角的距离：

```c
score += angle_distance(candidate.q[i], arm->joint_actual.q[i]);
```

分数更小的解会被选中。这样可以减少解分支突然跳变。

## 6. 舵机角度转换

关节角和舵机步进之间通过两组函数转换：

```c
Dof4_servo_to_angle(...)
Dof4_angle_to_servo(...)
```

转换时会使用：

- `servo_zero[i]`：舵机机械零位 position，默认 `2048`，不是角度。
- `servo_offset[i]`：实际/URDF 角度偏置，单位 rad；对应头文件里的 `*_ZERO_BIAS_DEG`。
- `servo_sign[i]`：URDF 正角对应舵机 position 增减方向，取 `+1` 或 `-1`。
- `servo_reverse[i]`：反向安装标志。
- `servo_min/max[i]`：舵机软限位。
- `joint_min/max[i]`：来自 URDF `<limit lower upper>` 的关节限位，单位 rad。

目标输出换算会先把实际/URDF 角度限制到 URDF 关节范围内：

```text
angle_rad = clamp(angle_rad, joint_min[i], joint_max[i])
servo_pos = servo_zero[i]
          + servo_sign[i] * (angle_rad - servo_offset[i])
          * DOF4_SERVO_POS_PER_RAD
servo_pos = clamp(round(servo_pos), servo_min[i], servo_max[i])
```

反馈换算只会先限制舵机 position，再反算真实角度，不再套用 URDF 关节限位：

```text
servo_pos = clamp(servo_pos, servo_min[i], servo_max[i])
angle_rad = (servo_pos - servo_zero[i])
          / (servo_sign[i] * DOF4_SERVO_POS_PER_RAD)
          + servo_offset[i]
```

因此 `joint_actual.q[i]` 会保留实机反馈的原始关节角；安全限位只作用在 `Dof4_angle_to_servo()` 和 `joint_target` 目标输出路径上。

右臂默认方向已按实机安装写入 `Dof4_arm_default_config(DOF4_ARM_RIGHT)`：

- R_J1：URDF `+Z`，顺时针 position 增加，`servo_sign = -1`。
- R_J2：安装对应 URDF `-Y`，逆时针 position 增加，`servo_sign = +1`。
- R_J3：安装对应 URDF `-Y`，顺时针 position 增加，`servo_sign = -1`。
- R_J4：安装对应 URDF `+Y`，逆时针 position 增加，`servo_sign = +1`。

左臂暂未实测，头文件中保留 `L_Jx_ZERO_POS / L_Jx_ZERO_BIAS_DEG / L_Jx_SERVO_SIGN` 占位和 TODO 标注。

这样后续实机标定时，只需要改配置或调用：

```c
Dof4_arm_set_servo_offset(...)
Dof4_arm_set_pitch_offset(...)
Dof4_arm_set_tcp_offset(...)
Dof4_arm_set_base_offset(...)
```

不需要改 IK 公式。

## 7. 笛卡尔五次轨迹规划

轨迹规划器在 `Trajectory_Planning.c` 中，核心类型是：

```c
Dof4_CartesianPlanner
```

它包含 4 个五次多项式通道：

```text
axis[0] -> x
axis[1] -> y
axis[2] -> z
axis[3] -> pitch
```

每个通道的公式为：

```text
p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
```

边界条件默认是：

```text
起点速度 = 0
起点加速度 = 0
终点速度 = 0
终点加速度 = 0
```

规划流程：

```c
Dof4_cartesian_planner_plan(...)
Dof4_cartesian_planner_sample(...)
```

控制循环每次被外部调用时，只采样当前时刻的 `x/y/z/pitch`，然后交给 IK 求关节目标。

## 8. 一步式控制循环

主入口：

```c
Dof4_Status Dof4_dual_arm_control_loop(Dof4_Arm *left,
                                       Dof4_Arm *right,
                                       uint32_t now_ms);
```

该函数不设置频率，不包含 `while(1)`，不调用 `osDelay` 或 `HAL_Delay`。外部 RTOS 任务按 200 Hz 或其他频率调用即可。

单次调用流程：

1. `Dof4_batch_read_all_servo()` 读取反馈。
2. 反馈步进值转换为关节角。
3. FK 更新当前 TCP 和连杆端点。
4. 左右臂分别采样笛卡尔轨迹。
5. 对采样点执行几何 IK。
6. 预测下一步左右臂姿态。
7. `Dof4_collision_check()` 做胶囊体碰撞检测。
8. 无碰撞风险时，把关节角转成舵机步进。
9. 写入舵机目标。

任一环节失败都会返回明确错误码，不继续输出危险目标。

## 9. 胶囊体碰撞检测

碰撞检测使用每条连杆的世界坐标端点：

```c
Dof4_arm_get_link_endpoints(...)
```

每条连杆近似为：

```text
线段 + 半径 = 胶囊体
```

检测时计算左右臂所有连杆线段之间的最短距离：

```c
Dof4_col_segment_distance(...)
```

再减去两条连杆半径：

```text
surface_dist = center_dist - radius_a - radius_b
```

如果 `surface_dist` 小于安全距离，返回：

```c
DOF4_STATUS_COLLISION_RISK
```

## 10. 常见状态码

| 状态码 | 含义 |
| --- | --- |
| `DOF4_STATUS_OK` | 正常 |
| `DOF4_STATUS_NULL_PARAM` | 空指针 |
| `DOF4_STATUS_BAD_CONFIG` | 配置非法 |
| `DOF4_STATUS_OUT_OF_WORKSPACE` | 目标超出工作空间 |
| `DOF4_STATUS_IK_UNREACHABLE` | IK 不可达 |
| `DOF4_STATUS_JOINT_LIMIT` | 关节角超限 |
| `DOF4_STATUS_SERVO_LIMIT` | 舵机步进超限 |
| `DOF4_STATUS_COLLISION_RISK` | 存在碰撞风险 |
| `DOF4_STATUS_COMM_FAIL` | 舵机通信失败 |
| `DOF4_STATUS_NOT_READY` | 轨迹或模块尚未准备好 |

## 11. 推荐调试顺序

1. 先只调用 `Dof4_arm_default_config()` 和 `Dof4_arm_config_init()`。
2. 用一组手动关节角调用 FK，检查 `current_pose` 是否符合预期。
3. 用 FK 输出的 pose 再调用 IK，检查 IK/FK 回代误差。
4. 设置 `servo_zero`、`servo_sign`、`servo_reverse`，让关节角和实机运动方向一致。
5. 设置 `pitch_offset`，让吸盘 pitch 与实际姿态一致。
6. 设置 `tcp_offset`，把 TCP 从末端连杆端点校准到真实吸盘中心。
7. 最后接入 `Dof4_dual_arm_control_loop()` 周期调用。

## 12. 示例调用

```c
Dof4_Arm left;
Dof4_ArmConfig cfg = Dof4_arm_default_config(DOF4_ARM_LEFT);

cfg.ik_branch = DOF4_IK_BRANCH_NEAREST;
cfg.servo_zero[0] = 2048;
cfg.servo_sign[0] = 1;

if (Dof4_arm_config_init(&left, &cfg) != DOF4_STATUS_OK) {
    /* 处理配置错误 */
}

if (Dof4_arm_set_target(&left, 0.35f, 0.22f, 0.12f, -0.60f) != DOF4_STATUS_OK) {
    /* 目标超出工作空间或参数错误 */
}
```

双臂控制循环应放在外部任务中周期调用：

```c
void arm_task_loop(void)
{
    uint32_t now = HAL_GetTick();
    (void)Dof4_dual_arm_control_loop(&g_dof4_arm_left,
                                     &g_dof4_arm_right,
                                     now);
}
```

注意：该模块内部不负责 `osDelay(5)` 或 200 Hz 调度。
