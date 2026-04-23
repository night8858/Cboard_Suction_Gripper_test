#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "block_inspect.h"

#include "cmsis_os.h"

//////////////////////////////////////////////////////////////
//此处是负责检测和判断物块是否吸附的程序，利用光电或者微动开关实现// 

//static const uint16_t block_detect_pins[4] = {SW0_Pin, SW1_Pin, SW2_Pin, SW3_Pin}; 

//外部引脚要浮空输入
// void switch_input_gpio_init(uint16_t* Pins)
// {

// }


///物块检测任务，负责处理物块吸附状态的更新//




SwitchInput g_switch_input;

static volatile uint8_t sw0_debounce_pending = 0;
static volatile uint32_t sw0_last_irq_tick = 0;
static const uint32_t sw0_debounce_ms = 15U;

void switch_state_update(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_12) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_RESET) {
            // 物块已吸附
            g_switch_input.state[0] = 1;
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
        }
        else {
            // 物块未吸附
            g_switch_input.state[0] = 0;
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
        }
    }
}


void block_inspect_process(void)
{
    if (sw0_debounce_pending != 0U) {
        uint32_t now = HAL_GetTick();
        if ((now - sw0_last_irq_tick) >= sw0_debounce_ms) {
            switch_state_update(GPIO_PIN_12);
            sw0_debounce_pending = 0U;
        }
    }
}


//中断回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) 
    {
        case GPIO_PIN_12:
            // 中断中只记录事件，把去抖和采样放到任务上下文
            sw0_last_irq_tick = HAL_GetTick();
            sw0_debounce_pending = 1U;
            break;
        // case SW1_Pin:
        //     // 处理 SW1 的中断事件
        //     break;
        // case SW2_Pin:
        //     // 处理 SW2 的中断事件
        //     break;
        // case SW3_Pin:
        //     // 处理 SW3 的中断事件
        //     break;
        // default:
        //     break;
    }

}

