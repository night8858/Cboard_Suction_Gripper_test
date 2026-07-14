#include "main.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include "led_indicate_state.h"
#include "led_indicate_task.h"
#include "variables.h"

#define LED_TASK_PERIOD_MS 1U
#define LED_RGB_ALL_PINS   (RGB_R_Pin | RGB_G_Pin | RGB_B_Pin)

typedef struct {
    uint8_t answer;
    uint32_t generation;
} LedAnswerRequest;

typedef struct {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} LedPdmAccumulator;

static volatile LedAnswerRequest s_answer_request;

void led_indicate_answer_notify(uint8_t answer)
{
    if (answer >= 4U) {
        return;
    }

    taskENTER_CRITICAL();
    s_answer_request.answer = answer;
    s_answer_request.generation++;
    taskEXIT_CRITICAL();
}

static bool led_pdm_channel_step(uint16_t *accumulator, uint8_t level)
{
    if (level == 0U) {
        *accumulator = 0U;
        return false;
    }
    if (level == UINT8_MAX) {
        *accumulator = 0U;
        return true;
    }

    *accumulator += level;
    if (*accumulator >= 256U) {
        *accumulator -= 256U;
        return true;
    }
    return false;
}

static void led_rgb_write(const LedIndicateOutput *output,
                          LedPdmAccumulator *accumulator)
{
    uint32_t set_pins = 0U;

    if (led_pdm_channel_step(&accumulator->red, output->red)) {
        set_pins |= RGB_R_Pin;
    }
    if (led_pdm_channel_step(&accumulator->green, output->green)) {
        set_pins |= RGB_G_Pin;
    }
    if (led_pdm_channel_step(&accumulator->blue, output->blue)) {
        set_pins |= RGB_B_Pin;
    }

    const uint32_t reset_pins = LED_RGB_ALL_PINS & ~set_pins;
    /* 三通道位于同一 GPIOH，单次 BSRR 写入可避免切色瞬间串色。 */
    RGB_R_GPIO_Port->BSRR = set_pins | (reset_pins << 16U);
}

void led_indicate_task(void const *argument)
{
    (void)argument;

    LedIndicateState state;
    LedPdmAccumulator pdm_accumulator = {0U, 0U, 0U};
    uint32_t handled_answer_generation = 0U;

    led_indicate_state_init(&state, HAL_GetTick());
    HAL_GPIO_WritePin(RGB_R_GPIO_Port, LED_RGB_ALL_PINS, GPIO_PIN_RESET);

    for (;;) {
        LedAnswerRequest request;
        bool arm_error_active;
        uint32_t error_event_count;

        taskENTER_CRITICAL();
        request.answer = s_answer_request.answer;
        request.generation = s_answer_request.generation;
        arm_error_active =
            (g_dof4_arm_left.state == DOF4_ARM_STATE_ERROR) ||
            (g_dof4_arm_right.state == DOF4_ARM_STATE_ERROR);
        error_event_count = g_pc_action_error_event_count;
        taskEXIT_CRITICAL();

        const uint32_t now_ms = HAL_GetTick();
        if (request.generation != handled_answer_generation) {
            handled_answer_generation = request.generation;
            led_indicate_state_start_answer(&state, request.answer, now_ms);
        }

        const LedIndicateOutput output =
            led_indicate_state_step(&state,
                                    now_ms,
                                    arm_error_active,
                                    error_event_count);
        led_rgb_write(&output, &pdm_accumulator);

        osDelay(LED_TASK_PERIOD_MS);
    }
}
