#include "gimbal.h"
#include "SCSCL.h"
#include <stdio.h>

Gimbal_s Gimbal; // 定义全局云台结构体实例

/**
 * @brief 禁用舵机扭矩 (兼容包装，SCSCL 无此函数)
 */
static inline void DisableTorque(uint8_t ID)
{
    (void)EnableTorque(ID, 0);
}

/**
 * @brief 同步写位置扩展 (适配 SCSCL SyncWritePos)
 *
 * SCSCL 的 SyncWritePos 签名为:
 *   SyncWritePos(ID[], IDN, Position[], Time[], Speed[])
 *
 * 本包装将 ACC 参数忽略（SCSCL 不支持），Speed 原样传递，
 * Time 使用固定值 0 表示最大速度运动。
 */
static void SyncWritePosEx(uint8_t ID[], uint8_t IDN,
                           int16_t Position[],
                           uint16_t Speed[],
                           uint8_t ACC[])
{
    (void)ACC;
    uint16_t time[3] = {0, 0, 0};
    uint16_t pos_u16[3];
    for (uint8_t i = 0; i < IDN && i < 3; ++i) {
        pos_u16[i] = (uint16_t)Position[i];
    }
    SyncWritePos(ID, IDN, pos_u16, time, Speed);
}



static void get_gimbal_servo_pos(Gimbal_s *Gimbal) 
{
    if (Gimbal == NULL) {
        return;
    }
    // 根据云台状态获取当前舵机位置反馈
    int16_t pos1 = 0;
    int16_t pos2 = 0;
    int16_t pos3 = 0;

    pos1 = ReadPos(Gimbal->SERVO_ID1);  //读取舵机1的位置
    pos2 = ReadPos(Gimbal->SERVO_ID2);
    pos3 = ReadPos(Gimbal->SERVO_ID3);
    if (pos1 < 0 || pos2 < 0 || pos3  < 0) {
        Gimbal->state = GIMBAL_STATE_ERROR;
        return;
    }
    // 将舵机位置转换为云台当前角度，更新Gimbal结构体中的当前位置信息
    Gimbal->current_servo_positions[0] = pos1;
    Gimbal->current_servo_positions[1] = pos2;
    Gimbal->current_servo_positions[2] = pos3;

    //此处加负号是因为舵机顺时针为累加，正常是逆时针
    Gimbal-> current_J1    = -(Gimbal->current_servo_positions[0] / 4095.0f * 360.0f) - 180.0f; // 关节1角度，  单位：度   用于控制高度
    Gimbal-> current_PITCH  = -(Gimbal->current_servo_positions[1] / 4095.0f * 360.0f) - 180.0f; // 水平旋转角度，单位：度
    Gimbal-> current_YAW = -(Gimbal->current_servo_positions[2] / 4095.0f * 360.0f) - 180.0f; // 垂直旋转角度，单位：度
}

//
static void gimbal_servo_move_to_position(Gimbal_s *Gimbal) 
{
    if (Gimbal == NULL) {
        return;
    }
    // 将目标位置转换为舵机控制指令，并发送给云台电机
    uint8_t ID[3] = {Gimbal->SERVO_ID1, Gimbal->SERVO_ID2, Gimbal->SERVO_ID3};
    int16_t Position[3] = {
        (int16_t)(-((Gimbal->target_J1 + 180.0f) / 360.0f * 4095.0f)),      // 关节1角度转换为舵机位置
        (int16_t)(-((Gimbal->target_PITCH + 180.0f) / 360.0f * 4095.0f)),   // 水平旋转角度转换为舵机位置
        (int16_t)(-((Gimbal->target_YAW + 180.0f) / 360.0f * 4095.0f)),     // 垂直旋转角度转换为舵机位置
    };
    uint16_t Speed[3] = {1000, 1000, 1000}; // 速度数组
    uint8_t ACC[3] = {0, 0, 0}; // 加速度数组

    SyncWritePosEx(ID, 3, Position, Speed, ACC);
}

//失能
static void gimbal_servo_Disable(Gimbal_s *Gimbal) 
{
    if (Gimbal == NULL) {
        return;
    }
    // 禁用云台舵机
    uint8_t ID[3] = {Gimbal->SERVO_ID1, Gimbal->SERVO_ID2, Gimbal->SERVO_ID3};
    for (int i = 0; i < 3; i++) {
        DisableTorque(ID[i]);
    }
}

//使能
static void gimbal_servo_Enable(Gimbal_s *Gimbal) 
{
    if (Gimbal == NULL) {
        return;
    }
    // 启用云台舵机
    uint8_t ID[3] = {Gimbal->SERVO_ID1, Gimbal->SERVO_ID2, Gimbal->SERVO_ID3};
    for (int i = 0; i < 3; i++) {
        EnableTorque(ID[i], 1);
    }
}

//限制
static float clamp_float(float value, float min, float max) 
{
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

////////////////////////////////////////////////////////////
//对外接口


void gimbal_init(void) 
{
    // 初始化云台相关的硬件和参数
    // 例如：设置云台的初始位置、配置PID控制器等

    Gimbal.SERVO_ID1 = 0x11; // 设置云台J1舵机ID
    Gimbal.SERVO_ID2 = 0x12; // 设置云台PITCH舵机ID
    Gimbal.SERVO_ID3 = 0x13; // 设置云台YAW舵机ID
    Gimbal.state = GIMBAL_STATE_IDLE; // 设置云台初始状态

    Gimbal.current_YAW = 0.0f; // 设置云台初始水平旋转角度
    Gimbal.current_PITCH = 0.0f; // 设置云台初始垂直旋转角度
    Gimbal.current_J1 = 0.0f; // 设置云台初始关节角度
    gimbal_servo_Disable(&Gimbal); // 禁用云台舵机
    
}

void gimbal_start(void)
{
    gimbal_servo_Enable(&Gimbal); // 启用云台舵机
    Gimbal.state = GIMBAL_STATE_IDLE; // 设置云台状态为空闲
    gimbal_servo_move_to_position(&Gimbal); // 将云台移动到初始位置
}

void gimbal_control_loop(void) 
{
    // 云台控制循环：读取当前舵机位置反馈，并持续下发目标位置给舵机
    get_gimbal_servo_pos(&Gimbal);
    gimbal_servo_move_to_position(&Gimbal);
}


void gimbal_set_target_position(Gimbal_s *Gimbal, float J1, float pitch, float yaw) 
{
    if (Gimbal == NULL) {
        return;
    }

    Gimbal->target_J1 = J1;
    Gimbal->target_YAW = yaw;
    Gimbal->target_PITCH = pitch;
    gimbal_servo_move_to_position(Gimbal);
    // 设置云台的目标位置
    // 例如：将目标位置转换为云台的控制指令格式，并存储在全局变量中供控制循环使用

}



// //云台运动到指定角度位置--识别物块模式
// void gimbal_to_see_block(void)
// {


// }


// //云台运动到指定角度位置--识别数学模式
// void gimbal_to_see_math(void)
// {


// }

// //云台运动到指定角度位置--回到初始位置
// void gimbal_goback(void)
// {

// }