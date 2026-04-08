#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "can.h"
#include "PID.h"

#define DM4310_P_MIN -12.5f // 位置最小值
#define DM4310_P_MAX 12.5f  // 位置最大值
#define DM4310_V_MIN -10.0f   // 速度最小值
#define DM4310_V_MAX 10.0f    // 速度最大值
#define DM4310_KP_MIN 0.0f    // Kp最小值
#define DM4310_KP_MAX 500.0f  // Kp最大值
#define DM4310_KD_MIN 0.0f    // Kd最小值
#define DM4310_KD_MAX 5.0f    // Kd最大值
#define DM4310_T_MIN -9.0f   // 转矩最大值
#define DM4310_T_MAX 9.0f    // 转矩最小值


#define DM4340_P_MIN -12.5f // 位置最小值
#define DM4340_P_MAX 12.5f  // 位置最大值
#define DM4340_V_MIN -10.0f   // 速度最小值
#define DM4340_V_MAX 10.0f    // 速度最大值
#define DM4340_KP_MIN 0.0f    // Kp最小值
#define DM4340_KP_MAX 500.0f  // Kp最大值
#define DM4340_KD_MIN 0.0f    // Kd最小值
#define DM4340_KD_MAX 5.0f    // Kd最大值
#define DM4340_T_MIN -27.0f   // 转矩最大值
#define DM4340_T_MAX 27.0f    // 转矩最小值

#define DM8006_P_MIN -12.5f // 位置最小值
#define DM8006_P_MAX 12.5f  // 位置最大值
#define DM8006_V_MIN -45.0f   // 速度最小值
#define DM8006_V_MAX 45.0f    // 速度最大值
#define DM8006_KP_MIN 0.0f    // Kp最小值
#define DM8006_KP_MAX 500.0f  // Kp最大值
#define DM8006_KD_MIN 0.0f    // Kd最小值
#define DM8006_KD_MAX 5.0f    // Kd最大值
#define DM8006_T_MIN -18.0f   // 转矩最大值
#define DM8006_T_MAX 18.0f    // 转矩最小值

typedef enum 
{
    DM4310 = 0,
    DM4340 = 1,

}DM_motor_type;

typedef enum 
{
    PID_control = 0,
    MIT_control = 1,

}DM_motor_contorl_type;

typedef enum 
{
    MIT = 0,
    POS_SPEED = 1,
    SPEED = 2,

}DM_motor_control_mode;

enum PID_Type
{
    PID_POS = 0,
    PID_SPD = 1,
};

enum DM_motor_IDregistered
{
    DM_4340_1 = 0x01,
    DM_4340_2 = 0x02,
    DM_4340_3 = 0x03,
    DM_4310_1 = 0x04,
    DM_4310_2 = 0x05,
    DM_4310_3 = 0x06,
};

typedef struct 
{
    int state;              // 状态
    CAN_HandleTypeDef hcan; // CAN句柄
    int CAN_id;             // 从机ID
    int master_id;          // 主机ID

    DM_motor_control_mode control_mode;       // 控制模式 0MIT模式 1位置速度模式 2速度模式
    DM_motor_type motor_type;    // 电机类型
    DM_motor_contorl_type control_type; // 控制类型

    s_pid_absolute_t position_pid; // 位置环PID
    s_pid_absolute_t speed_pid;    // 速度环PID

    float f_kp;           // 位置环增益
    float f_p;            // 位置环偏差
    float f_kd;           // 速度环增益
    float f_v;            // 速度环偏差
    float f_t;            // 转矩

    int p_int;            
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float esc_back_position; // 返回的位置
    float esc_back_speed;    // 反馈速度
    float esc_back_current;  // 反馈电流/扭矩
    float esc_back_angle;    // 反馈电流/扭矩

    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;

    /*处理连续码盘值*/
    float esc_back_position_last; // 上一次返回的位置
    int64_t circle_num;           // 旋转圈数
    float serial_position;        // 总码盘值
    float serial_angle;           // 总角度
    float serial_angle_last;      // 上一次的总角度
    float real_angle;         // 真实角度

    /*目标值*/
    float target_speed; // 目标速度
    float set_speed;    // 设置速度
    double target_position; // 目标位置
    float target_angle;     // 目标角度
    float target_out_current;     // 目标电流

    int target_state;

    /*电机输出电流*/
    float out_current; // 输出电流
} s_DMmotor_data_t;

void DM_motor_Enable(s_DMmotor_data_t *motor_data);
void DM_motor_Disable(s_DMmotor_data_t *motor_data);
void DM_motor_setZero(s_DMmotor_data_t *motor_data);



int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
// static float uint_to_float(int x_int, float x_min, float x_max, int bits);
void DM_motor_config(s_DMmotor_data_t *motor_data, int id, 
                     CAN_HandleTypeDef *hcan , int motor_type , int control_type);
void DM_motor_PID_init(s_DMmotor_data_t *motor_data, float kp, float ki,
                       float kd, float errlim, float max_output, int PID_Type);
void DM_MITvlue_set(s_DMmotor_data_t *traget_motor, float p, float v, float kp, float kd, float t);

void MD4310_motor_Control(s_DMmotor_data_t *traget_motor);
void MD4340_motor_Control(s_DMmotor_data_t *traget_motor);
void DM_motor_control(s_DMmotor_data_t *traget_motor);

void DMmotor_data_recive(s_DMmotor_data_t *motor_data, uint8_t *RxDate);


#endif
