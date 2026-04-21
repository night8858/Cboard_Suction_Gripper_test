#ifndef VARIABLES_H
#define VARIABLES_H

#include <stdint.h>

#include "DM_motor.h"
#include "DJI_motor.h"
#include "arm_control.h"
#include "Planar_Robot_Arm.h"
#include "user_lib.h"
#include "pneumatic_control.h"
#include "gimbal.h"

#define APP_SERVO_COUNT 8
#define APP_LED_STRIP_PIXELS 30

typedef enum {
	HB_TASK_ARM = 0,
	HB_TASK_PUMP,
	HB_TASK_MOTOR_CAN,
	HB_TASK_SERVO,
	HB_TASK_SOLENOID,
	HB_TASK_LED_STRIP,
	HB_TASK_LED,
	HB_TASK_DEBUG,
	HB_TASK_MONITOR,
	HB_TASK_COUNT
} heartbeat_task_id_t;



extern s_DMmotor_data_t DMmotor_4340[3];
extern s_DMmotor_data_t DMmotor_4310[3];
extern s_Dji_motor_data_t DJI_motor_3508;

extern Planar_Robot_Arm Arm_LF;       //左前
extern Planar_Robot_Arm Arm_RF;       //右前
extern Planar_Robot_Arm Arm_LB;       //左后
extern Planar_Robot_Arm Arm_RB;       //右后

extern Gimbal_s Gimbal;


extern Solenoid_state_s solenoid_state;   //电磁阀状态结构体


extern volatile uint32_t g_task_heartbeat_ms[HB_TASK_COUNT];
extern volatile uint8_t g_system_alarm_active;

extern float g_motor_target_speed;

void heartbeat_kick(heartbeat_task_id_t task_id, uint32_t tick_ms);
uint32_t heartbeat_get_age_ms(heartbeat_task_id_t task_id, uint32_t now_ms);

#endif
