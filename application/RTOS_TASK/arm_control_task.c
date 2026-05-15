#include "main.h"

#include "freertos.h"
#include "cmsis_os.h"

#include "arm_control.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "variables.h"
#include <stdint.h>

#include "planar_robot_arm.h"
#include "action_scheduler.h"
#include "input_arbiter.h"



/// 机械臂控制任务
/// 负责机械臂的整体控制流程，包括任务规划、运动学计算、轨迹规划、关节控制和位置反馈等。
/// 目前是控制四个机械臂的代码
///
/// 【控制循环顺序】（优先级由高到低）：
///   1. input_arbiter_resolve() — 按优先级从 PC/RC 输入源仲裁目标位置
///   2. ACTION_loop()           — 若动作激活，覆盖 target 数组为动作路径点
///   3. planar_arm_control_loop() — 机械臂 IK + 轨迹规划 + 舵机输出
void arm_control_task(void *argument)
{
    // arm_config_init();
    // arm_enable();
    /* USER CODE BEGIN arm_control_task */
    /* Infinite loop */ 
    //uint32_t start_time = HAL_GetTick();
    // angle_set1();
    planar_robot_arm_all_init();
    input_arbiter_init();  /* 初始化输入仲裁器 */
    osDelay(500);  // 等待机械臂上电完成自检，确保通信链路稳定后再进入控制循环
    for (;;)
    {
        /*
         * 执行顺序说明：
         * 1. 将最新遥控器数据喂入 RC 缓冲区（由 DMA 后台持续更新）
         * 2. 输入仲裁器：将 PC/RC 数据按优先级写入 target_x_test/y_test
         * 3. 动作调度器：若 g_action_state != IDLE，覆盖 target 数组
         * 4. 机械臂控制：消费 target_x_test/y_test，执行 IK → 轨迹 → 舵机
         *
         * 这样设计确保：
         *   - 无动作时，PC/RC 可直接控制各臂
         *   - 动作执行时，路径点优先于手动控制
         *   - 各模块职责单一，互不耦合
         */
        input_arbiter_update_rc(&rc_ctrl);
        input_arbiter_resolve(action_get_global_state() != ACTION_STATE_IDLE);
        ACTION_loop();  // 物块交接动作控制循环，需周期调用以驱动状态机推进
        planar_arm_control_loop();
        // gripper_loop();
        // heartbeat_kick(HB_TASK_ARM, HAL_GetTick());8
        // // Add arm control logic here
        osDelay(5); // 控制周期 5ms，200Hz
    }
    /* USER CODE END arm_control_task */
}
