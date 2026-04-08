#define __BM_MOTOR_H__
#ifndef __BM_MOTOR_H__

//本末科技电机驱动模块

#include "can.h"

typedef struct {
  /**电机基础信息**/
  int master_id;          // 电机所连电调CAN通信ID
  int slave_id;           // 电机所连电调CAN通信从ID
  int state;              // 状态
  CAN_HandleTypeDef hcan; // CAN句柄

  s_pid_absolute_t speed_pid; // 速度环PID
  s_pid_absolute_t pos_pid;   // 速度环PID

  int16_t temperature; // 电机温度
  /**连续圈数变量**/
  int64_t circle_num; // 电机连续编码圈数
  uint8_t
      is_pos_ready; // 电机上电时目标位置为返回的绝对编码值，只用在了连续编码函数里
  /**刻度转角度**/
  float back_motor_ang;    // 电机当前编码器转换成角度（刻度转角度）
  float serial_motor_ang;  // 电机连续编码转换成角度（刻度转角度）
  double target_motor_ang; // 电机目标角度（用在PID位置环）（刻度转角度）
  /**刻度**/
  int64_t serial_position; // 电机连续编码值（刻度）
  int16_t back_position;   // 电机返回的编码器值（刻度）
  int16_t
      back_pos_last; // 电机连续编码上一次值，只用在电机连续编码函数里了（刻度）
  int16_t back_motor_speed; // 电机当前速度（刻度）
  double target_pos;        // 电机目标编码器值（用在PID位置环）（刻度）
  float target_motor_speed; // 目标电机速度（用在PID单环速度环）（刻度）
  /**motor+IMU**/
  float back_ang_imu;            // 返回的imu角度（motor+IMU）
  float back_ang_last_imu;       // 返回的上一次imu角度（motor+IMU）
  float back_ang_speed_imu;      // 返回的imu角速度（motor+IMU）
  float back_ang_speed_last_imu; // 返回的上一次imu角速度（motor+IMU）
  float target_ang_imu;          // 目标imu角度（motor+IMU）
  float target_ang_speed_imu;    // 目标imu角速度（motor+IMU）
  /**电流**/
  int16_t out_current;  // 输出电流值
  int16_t back_current; // 返回电流值

} s_BM_motor_data_t; // 电机信息结构体类型


#endif
