#include "main.h"

#include "cmsis_os.h"

#include "variables.h"
#include "command_decode_4dof.h"
#include "block_inspect.h"

void update_task(void const *argument)
{
  for (;;) {
      cmd4_rx_process();
      //block_inspect_process();
      cmd4_send_action_done();
      cmd4_send_diagnostic();
      cmd4_send_feedback();
      osDelay(20);
  }
}
