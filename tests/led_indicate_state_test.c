#include "led_indicate_state.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static void require_true(bool value, const char *message)
{
    if (!value) {
        printf("FAIL: %s\n", message);
        exit(1);
    }
}

static void require_rgb(LedIndicateOutput output,
                        LedIndicateMode mode,
                        uint8_t red,
                        uint8_t green,
                        uint8_t blue,
                        const char *message)
{
    if (output.mode != mode || output.red != red ||
        output.green != green || output.blue != blue) {
        printf("FAIL: %s mode=%d rgb=(%u,%u,%u)\n",
               message,
               (int)output.mode,
               (unsigned)output.red,
               (unsigned)output.green,
               (unsigned)output.blue);
        exit(1);
    }
}

static void test_normal_yellow_blink(void)
{
    LedIndicateState state;
    led_indicate_state_init(&state, 0U);

    require_rgb(led_indicate_state_step(&state, 0U, false, 0U),
                LED_INDICATE_MODE_NORMAL, 255U, 255U, 0U,
                "normal blink starts yellow");
    require_rgb(led_indicate_state_step(&state, 499U, false, 0U),
                LED_INDICATE_MODE_NORMAL, 255U, 255U, 0U,
                "normal blink remains on before 500ms");
    require_rgb(led_indicate_state_step(&state, 500U, false, 0U),
                LED_INDICATE_MODE_NORMAL, 0U, 0U, 0U,
                "normal blink turns off at 500ms");
    require_rgb(led_indicate_state_step(&state, 1000U, false, 0U),
                LED_INDICATE_MODE_NORMAL, 255U, 255U, 0U,
                "normal blink repeats every second");
}

static void test_answer_colors_and_timing(void)
{
    static const uint8_t expected[4][3] = {
        {0U, 255U, 0U},
        {64U, 64U, 64U},
        {0U, 0U, 255U},
        {255U, 0U, 0U},
    };

    for (uint8_t answer = 0U; answer < 4U; ++answer) {
        LedIndicateState state;
        led_indicate_state_init(&state, 100U);
        led_indicate_state_start_answer(&state, answer, 100U);

        require_rgb(led_indicate_state_step(&state, 100U, false, 0U),
                    LED_INDICATE_MODE_ANSWER,
                    expected[answer][0], expected[answer][1], expected[answer][2],
                    "answer starts on with mapped color");
        require_rgb(led_indicate_state_step(&state, 600U, false, 0U),
                    LED_INDICATE_MODE_ANSWER, 0U, 0U, 0U,
                    "answer turns off after 500ms");
        require_rgb(led_indicate_state_step(&state, 1100U, false, 0U),
                    LED_INDICATE_MODE_ANSWER,
                    expected[answer][0], expected[answer][1], expected[answer][2],
                    "answer turns on after one second");
        require_true(led_indicate_state_step(&state, 8099U, false, 0U).mode ==
                         LED_INDICATE_MODE_ANSWER,
                     "answer remains active before eight seconds");
        require_true(led_indicate_state_step(&state, 8100U, false, 0U).mode ==
                         LED_INDICATE_MODE_NORMAL,
                     "answer ends at eight seconds");
    }
}

static void test_new_answer_restarts_window(void)
{
    LedIndicateState state;
    led_indicate_state_init(&state, 0U);
    led_indicate_state_start_answer(&state, 0U, 0U);
    led_indicate_state_start_answer(&state, 3U, 7000U);

    require_rgb(led_indicate_state_step(&state, 7000U, false, 0U),
                LED_INDICATE_MODE_ANSWER, 255U, 0U, 0U,
                "new answer replaces color immediately");
    require_true(led_indicate_state_step(&state, 14999U, false, 0U).mode ==
                     LED_INDICATE_MODE_ANSWER,
                 "new answer restarts duration");
    require_true(led_indicate_state_step(&state, 15000U, false, 0U).mode ==
                     LED_INDICATE_MODE_NORMAL,
                 "restarted answer ends after eight seconds");
}

static void test_error_and_answer_priority(void)
{
    LedIndicateState state;
    led_indicate_state_init(&state, 0U);

    require_rgb(led_indicate_state_step(&state, 10U, true, 0U),
                LED_INDICATE_MODE_ERROR, 255U, 255U, 0U,
                "persistent error starts yellow on");
    require_rgb(led_indicate_state_step(&state, 210U, true, 0U),
                LED_INDICATE_MODE_ERROR, 0U, 0U, 0U,
                "persistent error turns off after 200ms");

    led_indicate_state_start_answer(&state, 2U, 300U);
    require_rgb(led_indicate_state_step(&state, 300U, true, 0U),
                LED_INDICATE_MODE_ANSWER, 0U, 0U, 255U,
                "answer overrides persistent error");
    require_rgb(led_indicate_state_step(&state, 8300U, true, 0U),
                LED_INDICATE_MODE_ERROR, 255U, 255U, 0U,
                "persistent error resumes after answer");
    require_true(led_indicate_state_step(&state, 8500U, false, 0U).mode ==
                     LED_INDICATE_MODE_NORMAL,
                 "cleared persistent error returns to normal");
}

static void test_transient_error_queues_three_blinks(void)
{
    LedIndicateState state;
    led_indicate_state_init(&state, 0U);
    led_indicate_state_start_answer(&state, 0U, 0U);

    require_true(led_indicate_state_step(&state, 100U, false, 1U).mode ==
                     LED_INDICATE_MODE_ANSWER,
                 "answer hides queued error");
    require_rgb(led_indicate_state_step(&state, 8000U, false, 1U),
                LED_INDICATE_MODE_ERROR, 255U, 255U, 0U,
                "queued error starts after answer");
    require_rgb(led_indicate_state_step(&state, 8200U, false, 1U),
                LED_INDICATE_MODE_ERROR, 0U, 0U, 0U,
                "first queued blink off phase");
    require_true(led_indicate_state_step(&state, 8400U, false, 1U).mode ==
                     LED_INDICATE_MODE_ERROR,
                 "second queued blink starts");
    require_true(led_indicate_state_step(&state, 8800U, false, 1U).mode ==
                     LED_INDICATE_MODE_ERROR,
                 "third queued blink starts");
    require_true(led_indicate_state_step(&state, 9200U, false, 1U).mode ==
                     LED_INDICATE_MODE_NORMAL,
                 "three queued blinks complete");
}

static void test_arm_error_inside_answer_is_reported_later(void)
{
    LedIndicateState state;
    led_indicate_state_init(&state, 0U);
    led_indicate_state_start_answer(&state, 3U, 0U);

    require_true(led_indicate_state_step(&state, 100U, true, 0U).mode ==
                     LED_INDICATE_MODE_ANSWER,
                 "answer hides new arm error");
    require_true(led_indicate_state_step(&state, 300U, false, 0U).mode ==
                     LED_INDICATE_MODE_ANSWER,
                 "answer remains after arm error recovers");
    require_rgb(led_indicate_state_step(&state, 8000U, false, 0U),
                LED_INDICATE_MODE_ERROR, 255U, 255U, 0U,
                "arm error inside answer is reported afterward");
}

int main(void)
{
    test_normal_yellow_blink();
    test_answer_colors_and_timing();
    test_new_answer_restarts_window();
    test_error_and_answer_priority();
    test_transient_error_queues_three_blinks();
    test_arm_error_inside_answer_is_reported_later();
    printf("led indicate state tests passed\n");
    return 0;
}
