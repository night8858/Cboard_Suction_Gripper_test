#include "main.h"

#include "cmsis_os.h"

#include "bsp_solenoid.h"
#include "variables.h"

void solenoid_control_task(void const *argument) {
  uint8_t i = 0;

  (void)argument;
  //bsp_solenoid_init();

  for (;;) {
    // for (i = 0; i < APP_SOLENOID_COUNT; i++) {
    //   bsp_solenoid_set(i, g_solenoid_state.command[i]);
    //   g_solenoid_state.state[i] = g_solenoid_state.command[i];
    // }

    // heartbeat_kick(HB_TASK_SOLENOID, HAL_GetTick());
    osDelay(2);
  }
}
