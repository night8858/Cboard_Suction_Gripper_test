#include "main.h"

#include "DJI_motor.h"
#include "variables.h"
#include "user_lib.h"

#include "pneumatic_control.h"

#define USE_RELAY_MODULE


extern s_Dji_motor_data_t DJI_motor_3508;
extern ramp_function_source_t pump_ramp;

// static GPIO_PinState s_solenoid_state_to_pin(uint8_t state)
// {
//     return (state != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
// }


void pump_M3508_init(void)
{
    s_Dji_motor_config(&DJI_motor_3508, 1, &hcan1);
    DJI_motor_SPEED_PID_init(&DJI_motor_3508, 17.2f, 1.2f, 0.0f, 1000, 20000);
}

void pump_speed_set(float target_speed)
{

    M3508_PID_SPEED_CONTROL(&DJI_motor_3508, target_speed);
    DJI_motor_3508_ID_1to4_control(DJI_motor_3508.out_current, 0, 0, 0);

}

#ifdef USE_RELAY_MODULE
// 继电器模块控制函数
// 继电器ID与GPIO引脚的映射关系（根据实际连接修改了）
static const uint16_t relay_pins[4] = {RELAY1_Pin_Pin, RELAY2_Pin_Pin, RELAY4_Pin_Pin,RELAY3_Pin_Pin};

void relay_init(void)
{
    /* 初始化时默认全部打开，确保安全 */
    for (uint8_t i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(GPIOB, relay_pins[i], GPIO_PIN_SET);
    }
}

void relay_control(uint8_t relay_id, uint8_t state)
{
    if (relay_id >= 4) {
        return;
    }

    GPIO_PinState pin_state = (state != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOB, relay_pins[relay_id], pin_state);
}


#endif /* USE_RELAY_MODULE */
