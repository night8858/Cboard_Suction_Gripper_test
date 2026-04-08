#ifndef PLANAR_ROBOT_ARM_H
#define PLANAR_ROBOT_ARM_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_TYPE_1 0
#define ARM_TYPE_2 1

typedef enum
{
    ARM_STATE_INITIALIZING,
    ARM_STATE_IDLE,
    ARM_STATE_MOVING,
    ARM_STATE_ERROR

} arm_state_e;

enum
{
    ARM_ID_LF = 0, // 左前
    ARM_ID_RF = 1, // 右前
    ARM_ID_LB = 2, // 左后
    ARM_ID_RB = 3  // 右后

}arm_id;

typedef struct {
    // 机械臂参数DH参数、关节限制等
    uint8_t arm_id; // 机械臂ID
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

void planar_robot_arm_move_to_position(Planar_Robot_Arm *arm, float x, float y);
void planar_robot_arm_feedback(Planar_Robot_Arm *arm);
void get_arm_servo_pos(Planar_Robot_Arm *arm);

void planar_arm_forward_kinematics(Planar_Robot_Arm *arm);
bool planar_arm_inverse_kinematics(Planar_Robot_Arm *arm,
                                   bool elbow_up);
bool planar_arm_control_loop(void);
bool planar_robot_arm_all_init(void);

void planar_robot_arm_config_init(int arm_type , Planar_Robot_Arm *arm ,
                                  uint8_t SERVO_ID1 , uint8_t SERVO_ID2,
                                  uint8_t ARM_ID,
                                  float origin_point_x_offset , 
                                  float origin_point_y_offset , 
                                  float servo_angle_offset1   ,
                                  float servo_angle_offset2);
#endif // PLANAR_ROBOT_ARM_H
