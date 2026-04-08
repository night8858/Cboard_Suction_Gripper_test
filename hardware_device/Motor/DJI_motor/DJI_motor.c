#include "DJI_motor.h"
#include "math.h"
#include "main.h"
#include "can.h"

#include "PID.h"

static CAN_TxHeaderTypeDef RM6020_tx_message; // can_6020发送邮箱
static CAN_TxHeaderTypeDef RM3508_tx_message; // can_3508发送邮箱RM
static CAN_TxHeaderTypeDef RM3508_tx_message2; // can_3508发送邮箱RM


static uint8_t can_3508_send_data[8];

void s_Dji_motor_config(s_Dji_motor_data_t *motor_data, int id, CAN_HandleTypeDef *hcan)
{
    motor_data->master_id = id;
    motor_data->state = 0;
    motor_data->hcan = *hcan;
    if(id >= 4)
    {
    motor_data->slave_id = id + 0x1FF;
    }
    else motor_data->slave_id = id + 0x200;

}

void DJI_motor_SPEED_PID_init(s_Dji_motor_data_t *motor_data, float kp, float ki,
                       float kd, float errlim, float max_output)
{
        pid_abs_param_init(&motor_data->speed_pid, kp, ki, kd, errlim,
                           max_output);
}

void DJI_motor_POS_PID_init(s_Dji_motor_data_t *motor_data, float kp, float ki,
                       float kd, float errlim, float max_output)
{
        pid_abs_param_init(&motor_data->pos_pid, kp, ki, kd, errlim,
                           max_output);
}

void M3508_PID_SPEED_CONTROL(s_Dji_motor_data_t *motor_data, float speed_set)
{
    int16_t speed_output = motor_single_loop_PID(&motor_data->speed_pid, speed_set, motor_data->back_motor_speed);
    motor_data->out_current = (int16_t)speed_output;
}


// void DJI_motor_3508_control(int out_current1, int out_current2, int out_current3, int out_current4)
// {
//     uint32_t send_mail_box01;

//     RM3508_tx_message.StdId = 0x200;
//     RM3508_tx_message.IDE = CAN_ID_STD;
//     RM3508_tx_message.RTR = CAN_RTR_DATA;
//     RM3508_tx_message.DLC = 0x08;

//     can_3508_send_data[0] = (motor_data1->out_current >> 8);
//     can_3508_send_data[1] = motor_data1->out_current;
//     can_3508_send_data[2] = (motor_data2->out_current >> 8);
//     can_3508_send_data[3] = motor_data2->out_current;
//     can_3508_send_data[4] = (motor_data3->out_current >> 8);
//     can_3508_send_data[5] = motor_data3->out_current;
//     can_3508_send_data[6] = (motor_data4->out_current >> 8);
//     can_3508_send_data[7] = motor_data4->out_current;
//     HAL_CAN_AddTxMessage(&hcan1, &RM3508_tx_message, can_3508_send_data, &send_mail_box01);
// }

void DJI_motor_3508_ID_1to4_control(int16_t out_current1, int16_t out_current2, int16_t out_current3, int16_t out_current4)
{
    uint32_t send_mail_box01;

    RM3508_tx_message.StdId = 0x200;
    RM3508_tx_message.IDE = CAN_ID_STD;
    RM3508_tx_message.RTR = CAN_RTR_DATA;
    RM3508_tx_message.DLC = 0x08;

    can_3508_send_data[0] = (out_current1 >> 8);
    can_3508_send_data[1] = out_current1;
    can_3508_send_data[2] = (out_current2 >> 8);
    can_3508_send_data[3] = out_current2;
    can_3508_send_data[4] = (out_current3 >> 8);
    can_3508_send_data[5] = out_current3;
    can_3508_send_data[6] = (out_current4 >> 8);
    can_3508_send_data[7] = out_current4;
    HAL_CAN_AddTxMessage(&hcan1, &RM3508_tx_message, can_3508_send_data, &send_mail_box01);
}

void DJI_motor_3508_recevie(s_Dji_motor_data_t *motor_data, uint8_t *RxDate)
{
    // 处理接收到的数据
        	motor_data->back_position    =		RxDate[0]<<8 | RxDate[1];
        	motor_data->back_motor_speed =		RxDate[2]<<8 | RxDate[3];
        	motor_data->back_current     = 	    RxDate[4]<<8 | RxDate[5];
            continue_motor_pos(motor_data);
}

// dji电机连续编码器数据处理
void continue_motor_pos(s_Dji_motor_data_t *s_motor)
{
    if (s_motor->is_pos_ready == 1) // 如果电机第一次上电后记录了那时的电机编码器值并将预备标志位置一了的话，进入此判断
    {
        // 如果（当前电机返回值-上一次电机返回值）值大于4096，因为电机不可能在几毫秒内转过半圈
        if (s_motor->back_position - s_motor->back_pos_last > 4096)
        {
            s_motor->circle_num--; // 圈数--
        }
        else if (s_motor->back_position - s_motor->back_pos_last < -4096) // 同上，只不过方向是反的
        {
            s_motor->circle_num++; // 圈数++
        }
    }
    else
    {
        s_motor->target_pos = s_motor->back_position; // 如果电机预备标志位不为1，也就是电机第一次上电
        s_motor->is_pos_ready = 1;                    // 电机预备标志位赋值为一，也就是说电机已经准备好
    }
    s_motor->back_pos_last = s_motor->back_position;                                // 将上一次进入该函数的电机返回值赋值，方便计算连续值
    s_motor->serial_position = s_motor->back_position + s_motor->circle_num * 8191; // 返回的电机连续编码器值
    s_motor->back_motor_ang = s_motor->back_position / 8191.0f * 360.0f;            // 返回的电机绝对角度
    s_motor->serial_motor_ang = s_motor->serial_position / 8191.0f * 360.0f;        // 返回的电机连续角度
}
