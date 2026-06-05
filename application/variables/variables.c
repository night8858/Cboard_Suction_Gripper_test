#include "variables.h"
#include "gimbal.h"
#include "DT7.h"

/* 全局变量定义区 */

// 电机数据结构体定义
s_DMmotor_data_t DMmotor_4340[3];
s_DMmotor_data_t DMmotor_4310[3];
s_Dji_motor_data_t DJI_motor_3508;

// 机械臂实例定义
Planar_Robot_Arm Arm_LF;       //左前
Planar_Robot_Arm Arm_RF;       //右前
Planar_Robot_Arm Arm_LB;       //左后
Planar_Robot_Arm Arm_RB;       //右后

Dof4_Arm g_dof4_arm_left;
Dof4_Arm g_dof4_arm_right;


//云台实例定义
Gimbal_s Gimbal;

//斜波启动函数。
ramp_function_source_t pump_ramp;

// 全局命令结构体实例定义
all_pc_command all_pc_command_t;

volatile uint32_t g_task_heartbeat_ms[HB_TASK_COUNT];
volatile uint8_t g_system_alarm_active = 0;

Solenoid_state_s solenoid_state;

SwitchInput g_switch_input;
    //收起位置


action_state_4dof_e LEFT_state;
action_state_4dof_e RIGHT_state;

//float g_motor_target_speed = 1000.0f;

/* 全局变量定义区 */

// void heartbeat_kick(heartbeat_task_id_t task_id, uint32_t tick_ms) {
// 	if ((uint32_t)task_id < HB_TASK_COUNT) {
// 		g_task_heartbeat_ms[(uint32_t)task_id] = tick_ms;
// 	}
// }

// uint32_t heartbeat_get_age_ms(heartbeat_task_id_t task_id, uint32_t now_ms) {
// 	uint32_t last_tick = 0;

// 	if ((uint32_t)task_id >= HB_TASK_COUNT) {
// 		return 0;
// 	}

// 	last_tick = g_task_heartbeat_ms[(uint32_t)task_id];
// 	return now_ms - last_tick;
// }

