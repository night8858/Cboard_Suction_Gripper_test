#ifndef LED_INDICATE_TASK_H
#define LED_INDICATE_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 通知 RGB 灯光线程显示答案颜色。
 * @param answer 答案编号，范围 0..3；越界值会被忽略。
 *
 * 本接口只更新一份受临界区保护的请求，不等待灯效完成，供 RTOS 任务上下文调用。
 * 新答案会覆盖当前答案并重新开始 8 秒显示窗口。
 */
void led_indicate_answer_notify(uint8_t answer);

#ifdef __cplusplus
}
#endif

#endif /* LED_INDICATE_TASK_H */
