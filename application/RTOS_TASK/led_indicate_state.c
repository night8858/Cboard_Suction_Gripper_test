#include "led_indicate_state.h"

#include <limits.h>
#include <stddef.h>

#define LED_NORMAL_PHASE_MS                500U
#define LED_NORMAL_PERIOD_MS              (LED_NORMAL_PHASE_MS * 2U)
#define LED_ERROR_PHASE_MS                 200U
#define LED_ERROR_BLINKS_PER_EVENT           3U
#define LED_ANSWER_PHASE_MS                500U
#define LED_ANSWER_PERIOD_MS              (LED_ANSWER_PHASE_MS * 2U)
#define LED_ANSWER_DURATION_MS            8000U
#define LED_LEVEL_FULL                     255U
#define LED_LEVEL_GRAY                      64U

static LedIndicateOutput led_indicate_output(LedIndicateMode mode,
                                              uint8_t red,
                                              uint8_t green,
                                              uint8_t blue)
{
    const LedIndicateOutput output = {
        .mode = mode,
        .red = red,
        .green = green,
        .blue = blue,
    };
    return output;
}

static void led_indicate_queue_error_blinks(LedIndicateState *state,
                                             uint32_t event_count)
{
    uint32_t blink_count;

    if (event_count > (UINT32_MAX / LED_ERROR_BLINKS_PER_EVENT)) {
        blink_count = UINT32_MAX;
    } else {
        blink_count = event_count * LED_ERROR_BLINKS_PER_EVENT;
    }

    if (state->pending_error_blinks > (UINT32_MAX - blink_count)) {
        state->pending_error_blinks = UINT32_MAX;
    } else {
        state->pending_error_blinks += blink_count;
    }
}

static LedIndicateOutput led_indicate_normal_output(const LedIndicateState *state,
                                                     uint32_t now_ms)
{
    const uint32_t phase_ms =
        (uint32_t)(now_ms - state->normal_epoch_ms) % LED_NORMAL_PERIOD_MS;
    const uint8_t level =
        (phase_ms < LED_NORMAL_PHASE_MS) ? LED_LEVEL_FULL : 0U;

    /* 正常状态黄色慢闪；错误状态使用更快的 200ms 相位闪烁。 */
    return led_indicate_output(LED_INDICATE_MODE_NORMAL, level, level, 0U);
}

static LedIndicateOutput led_indicate_answer_output(uint8_t answer,
                                                     uint32_t elapsed_ms)
{
    if ((elapsed_ms % LED_ANSWER_PERIOD_MS) >= LED_ANSWER_PHASE_MS) {
        return led_indicate_output(LED_INDICATE_MODE_ANSWER, 0U, 0U, 0U);
    }

    switch (answer) {
        case 0U:
            return led_indicate_output(LED_INDICATE_MODE_ANSWER,
                                       0U,
                                       LED_LEVEL_FULL,
                                       0U);
        case 1U:
            return led_indicate_output(LED_INDICATE_MODE_ANSWER,
                                       LED_LEVEL_GRAY,
                                       LED_LEVEL_GRAY,
                                       LED_LEVEL_GRAY);
        case 2U:
            return led_indicate_output(LED_INDICATE_MODE_ANSWER,
                                       0U,
                                       0U,
                                       LED_LEVEL_FULL);
        case 3U:
            return led_indicate_output(LED_INDICATE_MODE_ANSWER,
                                       LED_LEVEL_FULL,
                                       0U,
                                       0U);
        default:
            return led_indicate_output(LED_INDICATE_MODE_ANSWER, 0U, 0U, 0U);
    }
}

void led_indicate_state_init(LedIndicateState *state, uint32_t now_ms)
{
    if (state == NULL) {
        return;
    }

    *state = (LedIndicateState){0};
    state->normal_epoch_ms = now_ms;
    state->error_phase_start_ms = now_ms;
}

void led_indicate_state_start_answer(LedIndicateState *state,
                                     uint8_t answer,
                                     uint32_t now_ms)
{
    if (state == NULL || answer >= 4U) {
        return;
    }

    state->answer = answer;
    state->answer_start_ms = now_ms;
    state->answer_active = true;
    state->error_display_active = false;
}

LedIndicateOutput led_indicate_state_step(LedIndicateState *state,
                                          uint32_t now_ms,
                                          bool arm_error_active,
                                          uint32_t error_event_count)
{
    if (state == NULL) {
        return led_indicate_output(LED_INDICATE_MODE_NORMAL, 0U, 0U, 0U);
    }

    const uint32_t new_error_events =
        error_event_count - state->last_error_event_count;
    if (new_error_events > 0U) {
        led_indicate_queue_error_blinks(state, new_error_events);
        state->last_error_event_count = error_event_count;
    }

    /* 若机械臂错误完全发生在答案窗口内，答案结束后仍补报 3 次快闪。 */
    if (arm_error_active && !state->arm_error_previous && state->answer_active) {
        led_indicate_queue_error_blinks(state, 1U);
    }
    state->arm_error_previous = arm_error_active;

    if (state->answer_active) {
        const uint32_t elapsed_ms = now_ms - state->answer_start_ms;
        if (elapsed_ms < LED_ANSWER_DURATION_MS) {
            return led_indicate_answer_output(state->answer, elapsed_ms);
        }

        state->answer_active = false;
        state->error_display_active = false;
    }

    if (arm_error_active) {
        /* 持续错误已经覆盖所有瞬时错误，无需在恢复后重复补闪。 */
        state->pending_error_blinks = 0U;
    }

    const bool error_required =
        arm_error_active || (state->pending_error_blinks > 0U);
    if (!error_required) {
        state->error_display_active = false;
        return led_indicate_normal_output(state, now_ms);
    }

    if (!state->error_display_active) {
        state->error_display_active = true;
        state->error_phase_on = true;
        state->error_phase_start_ms = now_ms;
    }

    while (state->error_display_active &&
           (uint32_t)(now_ms - state->error_phase_start_ms) >= LED_ERROR_PHASE_MS) {
        state->error_phase_start_ms += LED_ERROR_PHASE_MS;

        if (state->error_phase_on) {
            state->error_phase_on = false;
            continue;
        }

        if (!arm_error_active && state->pending_error_blinks > 0U) {
            state->pending_error_blinks--;
        }
        if (!arm_error_active && state->pending_error_blinks == 0U) {
            state->error_display_active = false;
            break;
        }
        state->error_phase_on = true;
    }

    if (!state->error_display_active) {
        return led_indicate_normal_output(state, now_ms);
    }

    const uint8_t level = state->error_phase_on ? LED_LEVEL_FULL : 0U;
    return led_indicate_output(LED_INDICATE_MODE_ERROR, level, level, 0U);
}
