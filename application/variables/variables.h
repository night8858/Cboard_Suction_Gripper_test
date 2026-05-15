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
#include "DT7.h"
#include "block_inspect.h"
#include "action_scheduler.h"

#define APP_SERVO_COUNT 8
#define APP_LED_STRIP_PIXELS 30

#define USE_DT7_DEBUG

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


typedef struct 
{
	float LF_target_x;   // 左前机械臂目标末端X坐标，单位mm
	float LF_target_y;   // 左前机械臂目标末端Y坐标，单位mm
	float RF_target_x;   // 右前机械臂目标末端X坐标，单位mm
	float RF_target_y;   // 右前机械臂目标末端Y坐标，单位mm
	float LB_target_x;   // 左后机械臂目标末端X坐标，单位mm
	float LB_target_y;   // 左后机械臂目标末端Y坐标，单位mm
	float RB_target_x;   // 右后机械臂目标末端X坐标，单位mm
	float RB_target_y;   // 右后机械臂目标末端Y坐标，单位mm

	float gimbal_target_pitch;  // 云台目标俯仰角，单位度
	float gimbal_target_yaw;    // 云台目标偏航角，单位度

	float pump_target_speed;     // 泵电机目标速度，单位RPM

	uint8_t LF_solenoid_target_state; // 左前电磁阀目标状态，0=关闭，1=打开
	uint8_t RF_solenoid_target_state; // 右前电磁阀目标状态，0=关闭，1=打开
	uint8_t LB_solenoid_target_state; // 左后电磁阀目标状态，0=关闭，1=打开
	uint8_t RB_solenoid_target_state; // 右后电磁阀目标状态，0=关闭，1=打开

	uint8_t LF_Electromagnet_target_state; // 左前电磁铁目标状态，0=关闭，1=打开
	uint8_t RF_Electromagnet_target_state; // 右前电磁铁目标状态，0=关闭，1=打开
	uint8_t LB_Electromagnet_target_state; // 左后电磁铁目标状态，0=关闭，1=打开
	uint8_t RB_Electromagnet_target_state; // 右后电磁铁目标状态，0=关闭，1=打开

	/* 新增：动作控制指令（由 AA 08 帧写入） */
	uint8_t action_trigger;		/* 0=无动作，1=触发交接 */
	uint8_t action_pair_idx;	/* 交接对索引（ASSOC_PAIR_FRONT/REAR） */
	uint8_t action_dir_id;		/* 交接方向（LEFT_TO_RIGHT/RIGHT_TO_LEFT） */

}all_pc_command;


extern s_DMmotor_data_t DMmotor_4340[3];
extern s_DMmotor_data_t DMmotor_4310[3];
extern s_Dji_motor_data_t DJI_motor_3508;

extern Planar_Robot_Arm Arm_LF;       //左前
extern Planar_Robot_Arm Arm_RF;       //右前
extern Planar_Robot_Arm Arm_LB;       //左后
extern Planar_Robot_Arm Arm_RB;       //右后

extern Gimbal_s Gimbal;

extern Solenoid_state_s solenoid_state;   //电磁阀状态结构体

extern RC_ctrl_t rc_ctrl;   //遥控器数据结构体

extern all_pc_command all_pc_command_t;  //全局命令结构体实例定义

extern SwitchInput g_switch_input;        //物块检测微动开关状态结构体实例定义

/* 交接对状态全局变量 (兼容旧 LEFT/RIGHT 名称)
 * 类型定义: action_scheduler.h -> block_associate_state_e
 * 实际值由 ACTION_loop() 每周期刷新                          */
extern block_associate_state_e LEFT;     //前侧交接对(LF↔RF)状态
extern block_associate_state_e RIGHT;    //后侧交接对(LB↔RB)状态

extern volatile uint32_t g_task_heartbeat_ms[HB_TASK_COUNT];
extern volatile uint8_t g_system_alarm_active;

extern float g_motor_target_speed;

void heartbeat_kick(heartbeat_task_id_t task_id, uint32_t tick_ms);
uint32_t heartbeat_get_age_ms(heartbeat_task_id_t task_id, uint32_t now_ms);

#endif
