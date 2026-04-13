#include "gimbal.h"
#include <stdio.h>

extern Gimbal_s Gimbal; // 定义全局云台结构体实例



static void get_gimbal_servo_pos(Gimbal_s *Gimbal) 
{
    if (Gimbal == NULL) {
        return;
    }
    // 根据云台状态获取当前舵机位置反馈
    int16_t pos1 = 0;
    int16_t pos2 = 0;

    pos1 = ReadPos(Gimbal->SERVO_ID1);
    pos2 = ReadPos(Gimbal->SERVO_ID2);
    if (pos1 < 0 || pos2 < 0) {
        Gimbal->state = GIMBAL_STATE_ERROR;
        return;
    }
    // 将舵机位置转换为云台当前角度，更新Gimbal结构体中的当前位置信息
    Gimbal->current_servo_positions[0] = pos1;
    Gimbal->current_servo_positions[1] = pos2;

    //此处加负号是因为舵机顺时针为累加，正常是逆时针
    Gimbal->current_YAW   = -(Gimbal->current_servo_positions[0] / 4095.0f * 360.0f) - 180.0f; // 水平旋转角度，单位：度
    Gimbal->current_PITCH = -(Gimbal->current_servo_positions[1] / 4095.0f * 360.0f) - 180.0f; // 垂直旋转角度，单位：度
}

static void gimbal_servo_move_to_position(Gimbal_s *Gimbal) 
{
    if (Gimbal == NULL) {
        return;
    }
    // 将目标位置转换为舵机控制指令，并发送给云台电机
    uint8_t ID[2] = {Gimbal->SERVO_ID1, Gimbal->SERVO_ID2};
    int16_t Position[2] = {
        (int16_t)(-((Gimbal->target_YAW + 180.0f)   / 360.0f * 4095.0f)),   // 水平旋转角度转换为舵机位置
        (int16_t)(-((Gimbal->target_PITCH + 180.0f) / 360.0f * 4095.0f))  // 垂直旋转角度转换为舵机位置
    };
    uint16_t Speed[2] = {0, 0}; // 速度数组
    uint8_t ACC[2] = {0, 0}; // 加速度数组

    SyncWritePosEx(ID, 2, Position, Speed, ACC);
}


void gimbal_init(void) 
{
    // 初始化云台相关的硬件和参数
    // 例如：设置云台的初始位置、配置PID控制器等

    Gimbal.SERVO_ID1 = 9; // 设置云台舵机ID
    Gimbal.SERVO_ID2 = 10; // 设置云台舵机ID
    Gimbal.state = GIMBAL_STATE_IDLE; // 设置云台初始状态
    Gimbal.current_YAW = 0.0f; // 设置云台初始水平旋转角度
    Gimbal.current_PITCH = 0.0f; // 设置云台初始垂直旋转角度
    gimbal_servo_move_to_position(&Gimbal); // 将云台移动到初始位置
}


void gimbal_control_loop(void) 
{
    // 云台控制循环
    // 例如：根据目标位置计算云台的控制指令，并发送给云台电机
    gimbal_set_target_position(&Gimbal, Gimbal.target_YAW, Gimbal.target_PITCH);
    get_gimbal_servo_pos(&Gimbal);
    gimbal_servo_move_to_position(&Gimbal);
}


void gimbal_set_target_position(Gimbal_s *Gimbal, float yaw, float pitch) 
{
    if (Gimbal == NULL) {
        return;
    }

    Gimbal->target_YAW = yaw;
    Gimbal->target_PITCH = pitch;
    gimbal_servo_move_to_position(Gimbal);
    // 设置云台的目标位置
    // 例如：将目标位置转换为云台的控制指令格式，并存储在全局变量中供控制循环使用

}

