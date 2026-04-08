#include "main.h"

#include "freertos.h"
#include "cmsis_os.h"

#include "bsp_led.h"
#include "variables.h"

void led_indicate_task(void const * argument)
{   osDelay(1000);
    /* USER CODE BEGIN led_indicate_task */
    /* Infinite loop */
    for (;;)
    {
        // led_heartbeat();
        // heartbeat_kick(HB_TASK_LED, HAL_GetTick());
        osDelay(500);
    }
    /* USER CODE END led_indicate_task */
}