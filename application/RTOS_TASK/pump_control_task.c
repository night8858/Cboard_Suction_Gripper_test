#include "main.h"

#include "freertos.h"
#include "cmsis_os.h"

#include "DJI_motor.h"
#include "variables.h"
#include "pump_control.h"

void pump_control_task(void const * argument)
{
    /* USER CODE BEGIN pump_control_task */
    pump_M3508_init();     //初始化3508电机和pid参数
    /* Infinite loop */
    for(;;)
    {
        pump_speed_set(3000.0f);  ///不建议太大
        //heartbeat_kick(HB_TASK_PUMP, HAL_GetTick());
        osDelay(2);
    }
    /* USER CODE END pump_control_task */

}
