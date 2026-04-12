#include "main.h"

#include "DJI_motor.h"
#include "variables.h"
#include <stdbool.h>
#include "user_lib.h"

extern s_Dji_motor_data_t DJI_motor_3508;
extern ramp_function_source_t pump_ramp;

void pump_M3508_init(void)
{
    s_Dji_motor_config(&DJI_motor_3508, 1, &hcan1);
    DJI_motor_SPEED_PID_init(&DJI_motor_3508, 13.0f, 1.2, 0.0, 1000, 14000);
}

void pump_speed_set(s_Dji_motor_data_t *motor_data,float target_speed)
{
    M3508_PID_SPEED_CONTROL(&DJI_motor_3508, target_speed);
    DJI_motor_3508_ID_1to4_control( DJI_motor_3508.out_current ,0,0,0);
}



void Solenoid_Valve_init(void)
{
    // 初始化电磁阀控制引脚
    // 例如：HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // 关闭电磁阀

}

void Solenoid_Valve_control(uint8_t ID , bool state)
{
    // 控制电磁阀的开关
    // 例如：HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (state) {
        // 打开电磁阀
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    } else {
        // 关闭电磁阀
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    
    }
}
