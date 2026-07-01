#include "main.h"

#include "cmsis_os.h"

#include "bsp_led.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "variables.h"

#define LED_HEARTBEAT_PERIOD_MS        500U
#define LED_YELLOW_BLINK_ON_MS         150U
#define LED_YELLOW_BLINK_OFF_MS        150U
#define LED_YELLOW_BLINKS_PER_ERROR      3U
#define LED_TASK_PERIOD_MS              10U

void led_indicate_task(void const * argument)
{
    (void)argument;
    osDelay(1000);
    /* USER CODE BEGIN led_indicate_task */
    uint32_t last_heartbeat_ms = HAL_GetTick();
    uint32_t last_error_event_count = 0U;
    uint32_t pending_yellow_blinks = 0U;
    uint32_t yellow_phase_start_ms = HAL_GetTick();
    uint8_t yellow_on = 0U;

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);

    /* Infinite loop */
    for (;;)
    {
        const uint32_t now_ms = HAL_GetTick();

        if ((uint32_t)(now_ms - last_heartbeat_ms) >= LED_HEARTBEAT_PERIOD_MS) {
            HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
            last_heartbeat_ms = now_ms;
        }

        const uint32_t current_error_count = g_pc_action_error_event_count;
        const uint32_t new_error_events = current_error_count - last_error_event_count;
        if (new_error_events > 0U) {
            const uint32_t queued_blinks =
                (new_error_events > (UINT32_MAX / LED_YELLOW_BLINKS_PER_ERROR))
                    ? UINT32_MAX
                    : (new_error_events * LED_YELLOW_BLINKS_PER_ERROR);
            if (pending_yellow_blinks <= (UINT32_MAX - queued_blinks)) {
                pending_yellow_blinks += queued_blinks;
            } else {
                pending_yellow_blinks = UINT32_MAX;
            }
            last_error_event_count = current_error_count;
        }

        if (yellow_on != 0U) {
            if ((uint32_t)(now_ms - yellow_phase_start_ms) >= LED_YELLOW_BLINK_ON_MS) {
                HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
                yellow_on = 0U;
                yellow_phase_start_ms = now_ms;
                if (pending_yellow_blinks > 0U) {
                    pending_yellow_blinks--;
                }
            }
        } else if (pending_yellow_blinks > 0U) {
            if ((uint32_t)(now_ms - yellow_phase_start_ms) >= LED_YELLOW_BLINK_OFF_MS) {
                HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
                yellow_on = 1U;
                yellow_phase_start_ms = now_ms;
            }
        } else {
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
        }

        osDelay(LED_TASK_PERIOD_MS);
    }
    /* USER CODE END led_indicate_task */
}
