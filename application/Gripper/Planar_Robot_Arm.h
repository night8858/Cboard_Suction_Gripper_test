#ifndef PLANAR_ROBOT_ARM_H
#define PLANAR_ROBOT_ARM_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_TYPE_1 0
#define ARM_TYPE_2 1


/* ---- 轨迹规划参数 ---- */
/* 重规划阈值：末端目标变化超过该距离时触发轨迹重规划 */
#define CONTROLA_REPLAN_EPS_MM              20.0f
/* 自适应轨迹时长上下限：防止运动过快(振荡)或过慢(响应迟钝) */
#define CONTROLA_TRAJ_MIN_S                 0.5f    /* 单段轨迹最短时间 (s) */
#define CONTROLA_TRAJ_MAX_S                 2.0f    /* 单段轨迹最长时间 (s) */
/* 轨迹时长缩放基准：步进空间目标运动速度 (步/s) */
#define CONTROLA_MAX_SERVO_SPEED_STEP_PER_S 4096.0f

/* ---- 工作空间安全参数 ---- */
/* 在最大/最小可达半径处各留出的安全余量 (mm)，
 * 防止末端趋近全伸(theta2≈0°)或全折(theta2≈±180°)奇异构型，
 * 避免 cos_theta2 趋近 ±1 时 IK 数值不稳定。             */
#define ARM_WORKSPACE_MARGIN_MM             2.0f


typedef enum
{
    ARM_STATE_INITIALIZING,
    ARM_STATE_IDLE,
    ARM_STATE_MOVING,
    ARM_STATE_ERROR

} arm_state_e;

typedef enum
{
    ARM_ID_LF = 0, // 左前
    ARM_ID_RF = 1, // 右前
    ARM_ID_LB = 2, // 左后
    ARM_ID_RB = 3  // 右后

}arm_id_e;

typedef struct {
    // 机械臂参数DH参数、关节限制等
    arm_id_e arm_id; // 机械臂ID
    uint8_t arm_type; // 机械臂类型(ARM_TYPE_1/ARM_TYPE_2)
    uint8_t SERVO_ID1;
    uint8_t SERVO_ID2;

    float link1_length;
    float link2_length;
    // 其他参数

    arm_state_e state; // 机械臂当前状态
    
    float current_joint_angles[2]; // 各关节角度数组
    int16_t current_servo_positions[2]; // 各舵机位置数组

    float target_joint_angles[2]; // 目标关节角度


    int16_t target_servo_positions[2]; // 目标舵机位置数组

    uint8_t servo_move_state[2]; // 舵机运动状态数组  1-运动中 0-停止

    float servo_angle_offset[2]; // 舵机角度偏移数组

    int servo_Reverse_installation[2]; // 舵机反向安装标志数组  1-反向安装 0-正常安装

    float end_effector_x; // 末端执行器X坐标
    float end_effector_y; // 末端执行器Y坐标

    float end_aim_x; // 末端执行器目标X坐标
    float end_aim_y; // 末端执行器目标Y坐标

    float planned_target_x; // 本次规划锁定的用户目标X（快照，不被IK修改）
    float planned_target_y; // 本次规划锁定的用户目标Y（快照，不被IK修改）


    float origin_point_x_offset ; // 原点X坐标偏移
    float origin_point_y_offset ; // 原点Y坐标偏移

    /* 各关节舵机步进值软件限位（0~4095 为硬件全量程）。
     * 在 planar_robot_arm_all_init 中按各臂实际机械结构配置。
     * IK 转步进值和轨迹输出两处均引用此字段，保证计划与执行一致。 */
    int16_t servo_pos_min[2]; // 关节1/2最小步进值
    int16_t servo_pos_max[2]; // 关节1/2最大步进值

    /* 末端工作空间矩形限位（外部坐标系，单位 mm）。
     * set_target 入口处自动将目标钳位到该矩形内的最近边界点。
     * 在 planar_robot_arm_all_init 中按各臂安装位置配置。     */
    float workspace_x_min;   /* X 轴下界 */
    float workspace_x_max;   /* X 轴上界 */
    float workspace_y_min;   /* Y 轴下界 */
    float workspace_y_max;   /* Y 轴上界 */

} Planar_Robot_Arm;


// 轨迹段结构体：描述一段从起点到终点的运动
typedef struct {
    // 边界条件
    float p0, v0, a0; // 起点：位置、速度、加速度
    float pf, vf, af; // 终点：位置、速度、加速度
    float duration;   // 运动总时间 (秒)
    
    // 计算出的五次多项式系数 [a0, a1, a2, a3, a4, a5]
    float coeffs[6];
    
    bool is_valid;    // 标记该轨迹段是否有效
} TrajectorySegment;

// 轨迹规划器状态机
typedef struct {
    TrajectorySegment current_segment;
    uint32_t start_time; /* HAL_GetTick() snapshot; uint32 arithmetic is wrap-safe */
    bool is_running;  // 状态机运行标志
} TrajectoryPlanner;

//轨迹规划器（关节空间）
typedef struct {
    TrajectoryPlanner planner_j1; /* joint 1 servo-step trajectory */
    TrajectoryPlanner planner_j2; /* joint 2 servo-step trajectory */
    float last_cmd_x;
    float last_cmd_y;
    /* 上周期末端指令状态（步进空间）——重规划起点，保证指令位置连续 */
    float cmd_j1, cmd_j2;   /* 指令位置 (步) */
    float vel_j1, vel_j2;   /* 指令速度 (步/s) */
    float acc_j1, acc_j2;   /* 指令加速度 (步/s²) */
    bool initialized;
} ArmTrajectoryContext;

void planar_robot_arm_move_to_position(Planar_Robot_Arm *arm, float x, float y);
void planar_robot_arm_feedback(Planar_Robot_Arm *arm);
void get_arm_servo_pos(Planar_Robot_Arm *arm);

// 根据机械臂ID获取实例指针，供外部模块做状态查询。
Planar_Robot_Arm *planar_robot_arm_get_by_id(arm_id_e arm_id);

// 外部控制接口：按机械臂ID下发目标末端位置，肘型使用该机械臂当前配置。
bool planar_robot_arm_set_target(arm_id_e arm_id, float target_x, float target_y);

// 外部控制接口（带肘型）：按机械臂ID下发目标末端位置，并同时更新肘型配置。
bool planar_robot_arm_set_target_with_elbow(arm_id_e arm_id,
                                            float target_x,
                                            float target_y,
                                            bool elbow_up);

void planar_arm_forward_kinematics(Planar_Robot_Arm *arm);
bool planar_arm_inverse_kinematics(Planar_Robot_Arm *arm, bool elbow_up);
bool controlA_loop(void);
bool planar_arm_control_loop(void);
bool planar_robot_arm_all_init(void);

void planar_robot_arm_config_init(int arm_type , Planar_Robot_Arm *arm ,
                                  uint8_t SERVO_ID1 , uint8_t SERVO_ID2,
                                  arm_id_e ARM_ID,
                                  float origin_point_x_offset , 
                                  float origin_point_y_offset , 
                                  float servo_angle_offset1   ,
                                  float servo_angle_offset2);

/* 将四臂末端目标设为预定义归位点；控制任务下一周期起自动规划并执行归位轨迹。
 * 归位位置已验证可达且各臂互不干涉，可在任意时刻调用以触发归位动作。        */
void planar_robot_arm_go_home(void);

/* ════════════════════════════════════════════════════════════════
 *  物块交接动作控制接口
 *
 *  使用方式：
 *    1. 在 RTOS 任务中周期调用 ACTION_loop()（建议 10~20ms 周期）
 *    2. 调用 associate_trigger(pair_idx) 启动指定交接对的流程
 *    3. pair_idx: 0=前侧(LF↔RF), 1=后侧(LB↔RB)
 * ════════════════════════════════════════════════════════════════ */

/** @brief 触发指定交接对开始物块移交流程（仅 IDLE 状态有效） */
bool associate_trigger(uint8_t pair_idx);

/** @brief 强制中止指定交接对的当前流程，立即回到 IDLE */
void associate_abort(uint8_t pair_idx);

/** @brief 查询指定交接对的当前状态（返回枚举值的 uint8_t 表示） */
uint8_t associate_get_state(uint8_t pair_idx);

/** @brief 专用动作循环，周期处理所有交接对状态机并同步全局状态变量 */
void ACTION_loop(void);

#endif // PLANAR_ROBOT_ARM_H
