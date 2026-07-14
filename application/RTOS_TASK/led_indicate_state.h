#ifndef LED_INDICATE_STATE_H
#define LED_INDICATE_STATE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_INDICATE_MODE_NORMAL = 0,
    LED_INDICATE_MODE_ERROR,
    LED_INDICATE_MODE_ANSWER
} LedIndicateMode;

typedef struct {
    LedIndicateMode mode;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} LedIndicateOutput;

typedef struct {
    uint32_t normal_epoch_ms;
    uint32_t answer_start_ms;
    uint32_t error_phase_start_ms;
    uint32_t last_error_event_count;
    uint32_t pending_error_blinks;
    uint8_t answer;
    bool answer_active;
    bool error_display_active;
    bool error_phase_on;
    bool arm_error_previous;
} LedIndicateState;

/** @brief 初始化与硬件无关的 RGB 灯效状态机。 */
void led_indicate_state_init(LedIndicateState *state, uint32_t now_ms);

/** @brief 启动或覆盖一次答案灯效；无效答案会被忽略。 */
void led_indicate_state_start_answer(LedIndicateState *state,
                                     uint8_t answer,
                                     uint32_t now_ms);

/**
 * @brief 按当前时间和错误输入推进灯效状态机。
 * @param arm_error_active 任一机械臂处于 ERROR 状态时为 true。
 * @param error_event_count 动作错误累计计数器，允许自然回绕。
 */
LedIndicateOutput led_indicate_state_step(LedIndicateState *state,
                                          uint32_t now_ms,
                                          bool arm_error_active,
                                          uint32_t error_event_count);

#ifdef __cplusplus
}
#endif

#endif /* LED_INDICATE_STATE_H */
