# 双四自由度机械臂详细设计

## 1. 目标和范围

本设计用于 `application/Gripper` 下的双四自由度机械臂模块，目标是在 STM32F407 上以较低计算量实现完整可用的运动学、笛卡尔轨迹规划、工作空间限制和双臂防碰撞。

本次实现采用几何解析解，不采用完整 URDF 数值迭代 IK。URDF 文件 `E:\double_arm4_urdf\urdf\double_arm4_urdf.urdf` 作为机械参数来源，用于提取左右臂基座位置、关节限位、关节轴方向、连杆等效长度和末端偏置。控制频率不在模块内部实现，外部 RTOS 任务或定时器按目标频率周期调用一步式控制循环。

## 2. URDF 关节模型提取

URDF 中左右臂均为 4 个旋转关节：

| 关节 | URDF 关节 | 等效功能 | 备注 |
| --- | --- | --- | --- |
| J1 | `L_joint1` / `R_joint1` | 基座绕 Z 轴水平旋转 | 轴为 `(0, 0, 1)` |
| J2 | `L_joint2` / `R_joint2` | 垂直工作平面内肩关节 | 零位轴换算后近似平行 Y 轴 |
| J3 | `L_joint3` / `R_joint3` | 垂直工作平面内肘关节 | 零位轴换算后近似平行 Y 轴 |
| J4 | `L_joint4` / `R_joint4` | 垂直工作平面内腕俯仰 | 零位轴换算后近似平行 Y 轴 |

URDF 关节限位作为默认关节限位：

| 机械臂 | J1 | J2 | J3 | J4 |
| --- | --- | --- | --- | --- |
| 左臂 | `[-3.49, 0.96]` | `[-2.70, 0]` | `[-4.44, 0]` | `[-1.57, 1.57]` |
| 右臂 | `[-0.96, 3.49]` | `[-2.70, 0]` | `[-4.44, 0]` | `[-1.57, 1.57]` |

为了降低计算量，固件中不在每个周期执行完整 4x4 齐次矩阵链。初始化时将 URDF 参数折算为以下等效几何量：

- `base[3]`：J1 在 `base_link` 下的位置。
- `shoulder_offset[3]`：J1 到 J2 的固定偏移。
- `link_len[3]`：J2-J3、J3-J4、J4-EE 的等效长度。
- `joint_min[4]` / `joint_max[4]`：关节限位。
- `pitch_offset`：末端安装带来的俯仰零位偏置，后续可通过 API 校准。
- `servo_offset[4]`、`servo_sign[4]`、`servo_reverse[4]`：舵机零点、方向和安装反向校准参数。

## 3. 坐标和姿态约定

外部坐标系与 URDF `base_link` 一致：

- X：机械臂前方。
- Y：左侧为正。
- Z：上方为正。

IK 输入固定为 `x, y, z, pitch`：

- `x, y, z` 是吸盘 TCP 在 `base_link` 下的位置，单位 m。
- `pitch` 是末端吸盘在 J2/J3/J4 垂直工作平面内的俯仰角，单位 rad。
- `pitch` 采用几何模型定义，不从完整欧拉角矩阵实时提取。

## 4. 正运动学

几何正解流程：

1. 从 J1 角度得到水平平面方向。
2. 将 J2/J3/J4 在垂直平面内逐段累加：
   - `p2 = base + shoulder_offset`
   - `p3 = p2 + Rz(J1) * plane_vec(L2, q2)`
   - `p4 = p3 + Rz(J1) * plane_vec(L3, q2 + q3)`
   - `pee = p4 + Rz(J1) * plane_vec(L4, q2 + q3 + q4)`
3. `current_pose.pitch = q2 + q3 + q4 + pitch_offset`。
4. 同时填充 `joint_world[]`，供胶囊体碰撞检测使用。

该方法每次 FK 只需要少量 `sinf/cosf`，适合 200 Hz 外部调用。

## 5. 逆运动学

几何 IK 流程：

1. 检查目标是否在该臂工作空间内。
2. 计算 J1：
   - 将目标点减去 `base` 和肩部固定偏移。
   - `q1 = atan2(y_local, x_local)`，再按左右臂基座和关节限位裁剪/验证。
3. 将 TCP 目标投影到 J1 定义的垂直平面，得到平面坐标 `(r, z)`。
4. 根据目标 `pitch` 回推腕点：
   - `wrist_r = r - L4 * cos(pitch - pitch_offset)`
   - `wrist_z = z - L4 * sin(pitch - pitch_offset)`
5. 用余弦定理求 J2/J3：
   - `cos_q3 = (wrist_r^2 + wrist_z^2 - L2^2 - L3^2) / (2 L2 L3)`
   - 根据初始化时配置的分支选择 `sin_q3` 正负。
   - `q2 = atan2(wrist_z, wrist_r) - atan2(L3 sin_q3, L2 + L3 cos_q3)`
6. 由 pitch 约束求 J4：
   - `q4 = pitch - pitch_offset - q2 - q3`
7. 检查所有关节限位和舵机限位。
8. 若配置为最近解，则同时计算肘上/肘下两个候选，选择与当前角度距离最小的解。

不可达目标不强行输出舵机命令，返回错误码。

## 6. 多解策略

多解策略在初始化时配置，运行中不猜测用户意图：

- `DOF4_IK_BRANCH_ELBOW_UP`：固定肘上。
- `DOF4_IK_BRANCH_ELBOW_DOWN`：固定肘下。
- `DOF4_IK_BRANCH_NEAREST`：在有效候选中选择最接近当前关节角的解。
- `DOF4_IK_BRANCH_DEFAULT`：使用模块默认分支。

API：

```c
Dof4_Status Dof4_arm_config_init(Dof4_Arm *arm,
                                 const Dof4_ArmConfig *config);

Dof4_Status Dof4_arm_set_ik_branch(Dof4_Arm *arm,
                                   Dof4_IkBranch branch);
```

## 7. 笛卡尔五次轨迹规划

轨迹规划在笛卡尔空间执行，规划变量为：

- `x`
- `y`
- `z`
- `pitch`

每个通道使用五次多项式：

```text
p(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
```

边界条件为起点和终点的位置、速度、加速度。默认起止速度和加速度为 0。每次控制循环由外部传入 `now_ms` 或 `dt_s`，模块只采样当前笛卡尔目标，不负责创建 200 Hz 调度。

轨迹执行流程：

1. 目标变化时，从当前实际 TCP 或当前轨迹采样点重规划。
2. 对 `x/y/z/pitch` 生成五次多项式。
3. 每次外部调用控制循环时采样当前笛卡尔目标。
4. 对采样点执行几何 IK。
5. 通过工作空间、关节限位和碰撞检测后输出舵机目标。

## 8. 双臂防碰撞

采用“连杆线段 + 半径”的胶囊体检测：

1. FK 填充每个臂的 J1、J2、J3、J4、EE 世界坐标。
2. 每条连杆近似为线段和半径。
3. 计算左右臂各连杆线段之间的最短距离。
4. 表面距离 `surface_dist = segment_dist - radius_a - radius_b`。
5. 若表面距离小于安全距离，控制循环拒绝输出该步目标。

默认检测 4 条主要连杆，也可包含 J4-EE 短末端段。安全距离和各连杆半径放在文件顶部常量或配置结构体中。

## 9. 控制循环

模块提供一步式控制循环：

```c
Dof4_Status Dof4_dual_arm_control_loop(Dof4_Arm *left,
                                       Dof4_Arm *right,
                                       uint32_t now_ms);
```

该函数不包含频率设定、不包含 `while(1)`，不使用 `HAL_Delay`。外部任务以 200 Hz 或其他频率调用。

单步流程：

1. 读取舵机反馈。
2. 更新当前关节角和 FK。
3. 采样左右臂各自的笛卡尔轨迹。
4. 对采样目标执行几何 IK。
5. 预测左右臂下一步关节姿态并执行胶囊体碰撞检测。
6. 通过检查后写入舵机目标。
7. 任一环节失败时返回明确错误码。

## 10. 错误处理

统一状态码：

| 错误码 | 含义 |
| --- | --- |
| `DOF4_STATUS_OK` | 成功 |
| `DOF4_STATUS_NULL_PARAM` | 空指针参数 |
| `DOF4_STATUS_BAD_CONFIG` | 配置非法 |
| `DOF4_STATUS_OUT_OF_WORKSPACE` | 目标超出工作空间 |
| `DOF4_STATUS_IK_UNREACHABLE` | 几何 IK 不可达 |
| `DOF4_STATUS_JOINT_LIMIT` | 关节角超限 |
| `DOF4_STATUS_SERVO_LIMIT` | 舵机步进值超限 |
| `DOF4_STATUS_COLLISION_RISK` | 存在碰撞风险 |
| `DOF4_STATUS_COMM_FAIL` | 舵机通信失败 |

舵机读写必须检查返回值，连续失败超过阈值后进入错误状态。ISR 内不得调用阻塞延时函数。

## 11. API 校准接口

为后续实机标定保留 API：

- 设置 TCP 偏移。
- 设置 pitch 零位偏置。
- 设置舵机零点偏置。
- 设置舵机方向和反向标志。
- 设置关节限位。
- 设置工作空间。
- 设置 IK 分支。
- 设置胶囊体半径和安全距离。

## 12. 测试和验证

主机端数学验证：

1. FK 零位输出应与 URDF 等效参数一致。
2. 对若干可达目标执行 IK，再 FK 回代，位置误差小于阈值。
3. 肘上/肘下分支产生不同但有效的关节解。
4. 关节限位外目标返回错误码。
5. 五次轨迹在起止点满足位置、速度、加速度边界条件。
6. 胶囊体距离检测能识别安全和碰撞风险场景。

嵌入式验证：

1. 编译 STM32 工程。
2. 确认所有新增函数有 Doxygen 注释。
3. 检查控制循环内部无 `HAL_Delay`。
4. 检查舵机通信调用均处理失败返回值。

