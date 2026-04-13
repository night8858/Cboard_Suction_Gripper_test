#ifndef GIMBAL_H
#define GIMBAL_H    

#include <stdbool.h>
#include <stdint.h>

typedef enum 
{
    GIMBAL_STATE_IDLE,
    GIMBAL_STATE_MOVING,
    GIMBAL_STATE_ERROR
} GimbalState;


typedef struct {

    uint8_t arm_type; // 机械臂类型(ARM_TYPE_1/ARM_TYPE_2)
    uint8_t SERVO_ID1;
    uint8_t SERVO_ID2;

    GimbalState state; // 机械臂当前状态
    
    float current_joint_angles[2]; // 各关节角度数组
    int16_t current_servo_positions[2]; // 各舵机位置数组

    float target_joint_angles[2]; // 目标关节角度
    int16_t target_servo_positions[2]; // 目标舵机位置数组

    float current_YAW;    // 水平旋转角度，单位：度
    float current_PITCH;  // 垂直旋转角度，单位：度

    float target_YAW;     // 目标水平旋转角度，单位：度
    float target_PITCH;   // 目标垂直旋转角度，单位：度

} Gimbal_s;

void gimbal_set_target_position(Gimbal_s *Gimbal, float yaw, float pitch);
void gimbal_init(void);



#endif // GIMBAL_Hq
