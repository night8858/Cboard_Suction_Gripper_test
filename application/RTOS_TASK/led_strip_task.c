#include "main.h"

#include "cmsis_os.h"

#include "bsp_led_strip.h"
#include "variables.h"

void led_strip_task(void const *argument) {
  uint8_t mode = 0;

  (void)argument;
  bsp_led_strip_init();

  for (;;) {
    // mode = g_system_alarm_active ? 1U : g_led_strip_state.effect_mode;
    // bsp_led_strip_service(mode, g_led_strip_state.brightness);
    // heartbeat_kick(HB_TASK_LED_STRIP, HAL_GetTick());
    
    osDelay(g_led_strip_state.frame_period_ms);
  }
}
