#include "main.h"

#include "cmsis_os.h"

#include "DJI_motor.h"
#include "pneumatic_control.h"
#include "variables.h"

extern PumpCtrl g_pump;  /* pneumatic_control.c 定义 */


/**
 * @brief 气泵控制任务
 *
 * 初始化 M3508 电机和继电器, 随后每 4ms 周期调用
 * pump_control_loop() 驱动气泵启停与转速控制.
 */
void pump_control_task(void const * argument)
{
    /* USER CODE BEGIN pump_control_task */
    pump_M3508_init();     /* 初始化 M3508 电机及 PID 参数 */
    relay_init();          /* 初始化继电器, 默认全部打开(安全) */
    osDelay(500);          /* 等待上电稳定 */

    for (;;)
    {

        pump_control_loop(&g_pump);
        // heartbeat_kick(HB_TASK_PUMP, HAL_GetTick());
        osDelay(2); /* 控制周期 2ms, 500Hz */
    }
    /* USER CODE END pump_control_task */
}
