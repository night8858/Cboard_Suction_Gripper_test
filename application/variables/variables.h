#ifndef VARIABLES_H
#define VARIABLES_H

#include <stdint.h>

#include "DM_motor.h"
#include "DJI_motor.h"
#include "user_lib.h"
#include "pneumatic_control.h"
#include "block_inspect.h"
#include "Dof4_Arm.h"
#include "Dof4_Collision.h"
#include "action_scheduler_4dof.h"

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

extern Solenoid_state_s solenoid_state;
extern SwitchInput g_switch_input;
extern action_state_4dof_e LEFT_state;
extern action_state_4dof_e RIGHT_state;

extern volatile uint32_t g_task_heartbeat_ms[HB_TASK_COUNT];
extern volatile uint8_t g_system_alarm_active;

extern float g_motor_target_speed;

void heartbeat_kick(heartbeat_task_id_t task_id, uint32_t tick_ms);
uint32_t heartbeat_get_age_ms(heartbeat_task_id_t task_id, uint32_t now_ms);

#endif
