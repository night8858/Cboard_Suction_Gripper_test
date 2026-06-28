#include "main.h"

#include "Dof4_Arm.h"
#include "bsp_usart.h"
#include "cmsis_os.h"

extern Dof4_Arm g_dof4_arm_left;
extern Dof4_Arm g_dof4_arm_right;

void usartr_debug_task(void const *argument)
{
  (void)argument;
  osDelay(1600);

  for (;;) {
    /*
     * Optional 4DOF debug stream. Keep this task free of the legacy planar
     * four-arm objects; active PC control is handled by command_decode_4dof.
     */
    // uart_dma_printf(
    //     &huart6,
    //     "%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\n",
    //     g_dof4_arm_left.current_pose.x,
    //     g_dof4_arm_left.current_pose.y,
    //     g_dof4_arm_left.current_pose.z,
    //     g_dof4_arm_left.current_pose.pitch,
    //     g_dof4_arm_right.current_pose.x,
    //     g_dof4_arm_right.current_pose.y,
    //     g_dof4_arm_right.current_pose.z,
    //     g_dof4_arm_right.current_pose.pitch);

    osDelay(4);
  }
}
