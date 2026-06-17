#ifndef USART_INTERFACE_H
#define USART_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32f4xx_hal.h"

/**
 * @brief 通过指定串口发送小 R 语音模块的“答案为 N”播报命令。
 *
 * @param huart 需要使用的 HAL 串口句柄，例如 &huart6、&huart1、&huart3。
 *              由调用者传入句柄，可以在同一个函数里灵活决定走哪一路 USART。
 * @param answer 答案编号，范围 0..3。
 * @retval HAL_OK 5 字节命令发送成功。
 * @retval HAL_ERROR huart 为空、answer 越界，或 HAL_UART_Transmit 返回失败。
 *
 * @note 命令帧固定为 5 字节，无额外 CRC/换行：
 *       AA 55 01 cmd FB，其中 cmd = 0x10 + answer。
 */
HAL_StatusTypeDef xiao_R_usart_send_answer(UART_HandleTypeDef *huart, uint8_t answer);

#ifdef __cplusplus
}
#endif

#endif /* USART_INTERFACE_H */
