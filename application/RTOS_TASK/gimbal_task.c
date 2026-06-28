#include "main.h"

#include "cmsis_os.h"

#include "variables.h"
#include "gimbal.h"

/**
 * @brief 云台控制任务 —— 覆盖 freertos.c 中的 __weak StartTask07
 *
 * 启动后延时等待系统就绪，然后进入周期控制循环：
 *   - 读取舵机位置反馈，更新 current_J1/YAW/PITCH
 *   - 下发目标位置到舵机（持续跟踪最近一次设定的目标）
 *
 * 目标位置由 CC 协议命令（CC 99 启动 / CC 01 运动）通过
 * gimbal_set_target_position() 异步更新。
 */
void gimbal_control_task(void const *argument)
{

  osDelay(1500);

  for (;;) {
    //尚未装配，暂时不启用云台控制
    //gimbal_control_loop();
    osDelay(10);
  }
}