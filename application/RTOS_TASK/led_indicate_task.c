#include "main.h"

#include "cmsis_os.h"

#include "bsp_led.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "variables.h"

void led_indicate_task(void const * argument)
{   osDelay(1000);
    /* USER CODE BEGIN led_indicate_task */
    /* Infinite loop */
    for (;;)
    {
        // led_heartbeat();
        // heartbeat_kick(HB_TASK_LED, HAL_GetTick());
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
        osDelay(500);
    }
    /* USER CODE END led_indicate_task */
}
