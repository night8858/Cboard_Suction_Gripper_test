#ifndef PLANAR_ROBOT_ARM_H
#define PLANAR_ROBOT_ARM_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_TYPE_1 0
#define ARM_TYPE_2 1


// 重规划阈值的定义：当末端位置误差小于该值时，认为当前轨迹仍然有效，无需重新规划。
#define CONTROLA_TRAJ_DURATION_S 2.0f
#define CONTROLA_REPLAN_EPS_MM   3.0f



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



    float origin_point_x_offset ; // 原点X坐标偏移
    float origin_point_y_offset ; // 原点Y坐标偏移


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
    float start_time; // 当前段开始的时间戳 (ms)
    bool is_running;  // 状态机运行标志
} TrajectoryPlanner;

//轨迹规划器
typedef struct {
    TrajectoryPlanner planner_x;
    TrajectoryPlanner planner_y;
    float last_cmd_x;
    float last_cmd_y;
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
bool planar_arm_inverse_kinematics(Planar_Robot_Arm *arm,
                                   bool elbow_up);
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
#endif // PLANAR_ROBOT_ARM_H
