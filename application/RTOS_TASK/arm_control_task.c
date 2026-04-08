#include "main.h"

#include "freertos.h"
#include "cmsis_os.h"

#include "arm_control.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "variables.h"
#include <stdint.h>

#include "planar_robot_arm.h"

void arm_control_task(void *argument)
{
    // arm_config_init();
    // arm_enable();
    /* USER CODE BEGIN arm_control_task */
    /* Infinite loop */ 
    //uint32_t start_time = HAL_GetTick();
    // angle_set1();
    planar_robot_arm_all_init();

    for (;;)
    {
        // if (HAL_GetTick() - start_time >= 3000)
        // {
        //     angle_set2();
        //     start_time = HAL_GetTick();
        // }
        planar_arm_control_loop();
        // gripper_loop();
        // heartbeat_kick(HB_TASK_ARM, HAL_GetTick());
        // // Add arm control logic here
        osDelay(2);
    }
    /* USER CODE END arm_control_task */
}
