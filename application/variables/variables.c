#include "variables.h"

s_DMmotor_data_t DMmotor_4340[3];
s_DMmotor_data_t DMmotor_4310[3];
s_Dji_motor_data_t DJI_motor_3508;

Dof4_Arm g_dof4_arm_left;
Dof4_Arm g_dof4_arm_right;

ramp_function_source_t pump_ramp;

volatile uint32_t g_task_heartbeat_ms[HB_TASK_COUNT];
volatile uint8_t g_system_alarm_active = 0;

Solenoid_state_s solenoid_state;
SwitchInput g_switch_input;

action_state_4dof_e LEFT_state;
action_state_4dof_e RIGHT_state;
