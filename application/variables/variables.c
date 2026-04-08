#include "variables.h"

/* 全局变量定义区 */

s_DMmotor_data_t DMmotor_4340[3];
s_DMmotor_data_t DMmotor_4310[3];

s_Dji_motor_data_t DJI_motor_3508;

Planar_Robot_Arm Arm_LF;       //左前
Planar_Robot_Arm Arm_RF;       //右前
Planar_Robot_Arm Arm_LB;       //左后
Planar_Robot_Arm Arm_RB;       //右后

volatile uint32_t g_task_heartbeat_ms[HB_TASK_COUNT];
volatile uint8_t g_system_alarm_active = 0;

servo_bus_state_t g_servo_state = {
		.id = {1, 2, 3, 4, 5, 6, 7, 8},
		.target_position = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048},
		.target_speed = {300, 300, 300, 300, 300, 300, 300, 300},
		.target_acc = {40, 40, 40, 40, 40, 40, 40, 40},
};

solenoid_state_t g_solenoid_state;

led_strip_state_t g_led_strip_state = {
		.brightness = 24,
		.effect_mode = 0,
		.frame_period_ms = 30,
};

float g_motor_target_speed = 1000.0f;

/* 全局变量定义区 */

void heartbeat_kick(heartbeat_task_id_t task_id, uint32_t tick_ms) {
	if ((uint32_t)task_id < HB_TASK_COUNT) {
		g_task_heartbeat_ms[(uint32_t)task_id] = tick_ms;
	}
}

uint32_t heartbeat_get_age_ms(heartbeat_task_id_t task_id, uint32_t now_ms) {
	uint32_t last_tick = 0;

	if ((uint32_t)task_id >= HB_TASK_COUNT) {
		return 0;
	}

	last_tick = g_task_heartbeat_ms[(uint32_t)task_id];
	return now_ms - last_tick;
}

