#include "main.h"

#include "DJI_motor.h"
#include "variables.h"
#include "user_lib.h"

#include "pneumatic_control.h"

extern s_Dji_motor_data_t DJI_motor_3508;
extern ramp_function_source_t pump_ramp;

// static GPIO_PinState s_solenoid_state_to_pin(uint8_t state)
// {
//     return (state != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
// }

// static GPIO_TypeDef *const s_solenoid_ports[4] = {
//     RELAY1_GPIO_Port,
//     RELAY2_GPIO_Port,
//     RELAY3_GPIO_Port,
//     RELAY4_GPIO_Port,
// };

// static const uint16_t s_solenoid_pins[4] = {
//     RELAY1_Pin,
//     RELAY2_Pin,
//     RELAY3_Pin,
//     RELAY4_Pin,
// };

void pump_M3508_init(void)
{
    s_Dji_motor_config(&DJI_motor_3508, 1, &hcan1);
    DJI_motor_SPEED_PID_init(&DJI_motor_3508, 13.0f, 1.2f, 0.0f, 1000, 10000);
}

void pump_speed_set(float target_speed)
{

    M3508_PID_SPEED_CONTROL(&DJI_motor_3508, target_speed);
    DJI_motor_3508_ID_1to4_control(DJI_motor_3508.out_current, 0, 0, 0);
}



// void Solenoid_Valve_init(void)
// {
//     /* 默认全部打开，确保上电满足抓取系统初始状态要求 */
//     for (uint8_t i = 0; i < 4; i++) {
//         g_solenoid_state.command[i] = 1u;
//         g_solenoid_state.state[i] = 1u;
//         //HAL_GPIO_WritePin(s_solenoid_ports[i], s_solenoid_pins[i], GPIO_PIN_RESET);
//     }

// }


// void Solenoid_Valve_control(uint8_t id, uint8_t state)
// {
//     if (id >= APP_SOLENOID_COUNT) {
//         return;
//     }

//     state = (state != 0u) ? 1u : 0u;
//     //HAL_GPIO_WritePin(s_solenoid_ports[id], s_solenoid_pins[id], s_solenoid_state_to_pin(state));
//     g_solenoid_state.state[id] = state;
// }

// void solenoid_control(Solenoid_ID solenoid_id, uint8_t state)
// {
//     Solenoid_Valve_control((uint8_t)solenoid_id, state);
// }

//气动控制循环函数
// void pneumatic_control_loop(void)
// {
//     for (uint8_t i = 0; i < APP_SOLENOID_COUNT; i++) {
//         uint8_t command = (g_solenoid_state.command[i] != 0u) ? 1u : 0u;
//         if (g_solenoid_state.state[i] != command) {
//             Solenoid_Valve_control(i, command);
//         }
//     }

// }
