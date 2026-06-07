#include "main.h"

#include "cmsis_os.h"

#include "bsp_usart.h"
#include "usart.h"
#include "variables.h"
#include "command_decode_4dof.h"
#include "block_inspect.h"
#include "DT7.h"


// 机械臂状态反馈任务，周期约50ms
// 通过串口发送当前机械臂末端位置、目标位置、当前舵机位置等信息，供上位机调试使用
// 同时也通过这个任务周期性地处理接收到的命令并发送反馈，确保命令的及时响应和系统状态的实时监控

void update_task(void const *argument) 
{
  remote_control_init();  // 初始化遥控器接收
  osDelay(1500);

  for (;;) {

      cmd4_rx_process();
      block_inspect_process();
      cmd4_send_feedback();
      osDelay(20);

  }
}
