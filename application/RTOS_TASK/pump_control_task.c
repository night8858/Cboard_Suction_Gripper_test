#include "DT7.h"
#include "main.h"

#include "freertos.h"
#include "cmsis_os.h"

#include "DJI_motor.h"
#include "pneumatic_control.h"
#include "variables.h"

extern all_pc_command all_pc_command_t;


void pump_control_task(void const * argument)
{
    /* USER CODE BEGIN pump_control_task */
    pump_M3508_init();     //初始化3508电机和pid参数
    relay_init();         //初始化继电器模块，确保电磁阀和电磁铁初始状态安全
    osDelay(500);        //等待系统稳定
    
        /* Infinite loop */
    /* Infinite loop */
    float target_speed = 2200.0f;
    uint8_t prev_s0 = 0xFF, prev_s1 = 0xFF;  /* 记录上次拨杆状态 */
    for(;;)
    {   
        #ifdef USE_DT7_DEBUG
        // uint8_t cur_s0 = get_remote_control_point()->rc.s[0];
        // uint8_t cur_s1 = get_remote_control_point()->rc.s[1];

        // /* 仅在拨杆状态变化时设定一次速度（边沿触发） */
        // if (cur_s1 != prev_s1 || cur_s0 != prev_s0) {
        //     if (cur_s1 == 1) {
        //         target_speed = 3000.0f;  // 右拨杆向后，测试用
        //     } else if (cur_s0 == 1) {
        //         target_speed = 0.0f;    // 双拨杆相同停止气泵
        //     }
        //     prev_s0 = cur_s0;
        //     prev_s1 = cur_s1;
        // }

        pump_speed_set(target_speed);  ///不建议太大

        #endif
        // if (get_remote_control_point()->rc.ch[3] > 400)
        // {z
        //     //relay_control(0, 1);  // 控制继电器0为开启状态 
        //     relay_control(3, 1);  // 控制继电器2为开启状态 
        // }else {
        //     relay_control(3, 0);  // 其他情况，关闭继电器2  
        // }
        //pump_speed_set(all_pc_command_t.pump_target_speed);
         //pump_speed_set(target_speed);  ///不建议太大
        //heartbeat_kick(HB_TASK_PUMP, HAL_GetTick());
        osDelay(4);
    }
    /* USER CODE END pump_control_task */

}
