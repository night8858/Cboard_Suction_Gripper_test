#include "main.h"

#include "cmsis_os.h"

#include "bsp_usart.h"
#include "usart.h"
#include "variables.h"
#include "command_decode.h"


void update_task(void const *argument) {
  osDelay(1500);

  for (;;) {
    // now = HAL_GetTick();
    // g_system_alarm_active = 0;

    // if (heartbeat_get_age_ms(HB_TASK_ARM, now) > HB_TIMEOUT_CTRL_MS ||
    //     heartbeat_get_age_ms(HB_TASK_PUMP, now) > HB_TIMEOUT_CTRL_MS ||
    //     heartbeat_get_age_ms(HB_TASK_MOTOR_CAN, now) > HB_TIMEOUT_CTRL_MS ||
    //     heartbeat_get_age_ms(HB_TASK_SERVO, now) > HB_TIMEOUT_CTRL_MS ||
    //     heartbeat_get_age_ms(HB_TASK_SOLENOID, now) > HB_TIMEOUT_CTRL_MS ||
    //     heartbeat_get_age_ms(HB_TASK_LED_STRIP, now) > HB_TIMEOUT_SLOW_MS) {
    //   g_system_alarm_active = 1;
    //   uart_dma_printf(&huart6,
    //                   "HB WARN A:%lu P:%lu M:%lu S:%lu V:%lu L:%lu\r\n",
    //                   heartbeat_get_age_ms(HB_TASK_ARM, now),
    //                   heartbeat_get_age_ms(HB_TASK_PUMP, now),
    //                   heartbeat_get_age_ms(HB_TASK_MOTOR_CAN, now),
    //                   heartbeat_get_age_ms(HB_TASK_SERVO, now),
    //                   heartbeat_get_age_ms(HB_TASK_SOLENOID, now),
    //                   heartbeat_get_age_ms(HB_TASK_LED_STRIP, now));
    // }
      cmd_send_feedback();
      osDelay(20);

  }
}
