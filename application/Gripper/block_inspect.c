#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "block_inspect.h"

#include "cmsis_os.h"

#define USE_CBOARD_SUCTION_GRIPPER_TEST

//////////////////////////////////////////////////////////////
//此处是负责检测和判断物块是否吸附的程序，利用光电或者微动开关实现// 
///物块检测任务，负责处理物块吸附状态的更新//

#ifdef USE_CBOARD_SUCTION_GRIPPER_TEST

static const uint16_t block_detect_pins[4] = {BLOCK_NSPECT_LF_Pin, BLOCK_NSPECT_RF_Pin, BLOCK_NSPECT_LB_Pin, BLOCK_NSPECT_RB_Pin}; 

static volatile uint8_t sw_debounce_pending[4] = {0}; // 每个开关的去抖状态标志
static volatile uint32_t sw_last_irq_tick[4]   = {0};   // 每个开关的上次中断时间戳
static const uint32_t sw_debounce_ms = 15U;

void switch_state_update(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) 
    {
        case BLOCK_NSPECT_LF_Pin:
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_RESET) {
                // 物块已吸附
                g_switch_input.state[0] = 1;
            }
            else {
                // 物块未吸附
                g_switch_input.state[0] = 0;
            }
            break;
        case BLOCK_NSPECT_RF_Pin:
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_RESET) {
                // 物块已吸附
                g_switch_input.state[1] = 1;
            }
            else {
                // 物块未吸附
                g_switch_input.state[1] = 0;
            }
            break;
        case BLOCK_NSPECT_LB_Pin:
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_RESET) {
                // 物块已吸附
                g_switch_input.state[2] = 1;
            }
            else {
                // 物块未吸附
                g_switch_input.state[2] = 0;
            }
            break;
        case BLOCK_NSPECT_RB_Pin:
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_RESET) {
                // 物块已吸附
                g_switch_input.state[3] = 1;
            }
            else {
                // 物块未吸附
                g_switch_input.state[3] = 0;
            }
            break;
        default:
            break;
    }
}

// 物块检测任务函数
void block_inspect_process(void)
{
    for (int i = 0; i < 4; i++) {
        if (sw_debounce_pending[i] != 0U) {
            uint32_t now = HAL_GetTick();
            if ((now - sw_last_irq_tick[i]) >= sw_debounce_ms) {
                switch_state_update(block_detect_pins[i]);
                sw_debounce_pending[i] = 0U;
            }
        }
    }
}


//中断回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) 
    {
        // 中断中只记录事件，把去抖和采样放到任务上下文
        case BLOCK_NSPECT_LF_Pin:
            
            sw_last_irq_tick[0] = HAL_GetTick();
            sw_debounce_pending[0] = 1U;
            break;
        case BLOCK_NSPECT_RF_Pin:
            sw_last_irq_tick[1] = HAL_GetTick();
            sw_debounce_pending[1] = 1U;
            break;
        case BLOCK_NSPECT_LB_Pin:
            sw_last_irq_tick[2] = HAL_GetTick();
            sw_debounce_pending[2] = 1U;
            break;  
        case BLOCK_NSPECT_RB_Pin:
            sw_last_irq_tick[3] = HAL_GetTick();
            sw_debounce_pending[3] = 1U;
            break;
        default:
            break;
    }

}

#endif
