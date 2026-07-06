#include "usart_interface.h"

/**
 * @brief “答案为零/一/二/三”的完整串口命令表。
 *
 * 字节含义：
 * - [0] 0xAA：帧头 1
 * - [1] 0x55：帧头 2
 * - [2] 0x01：语音播报指令类型
 * - [3] 0x10..0x13：答案播报 ID，0x10 + answer
 * - [4] 0xFB：帧尾
 *
 * 使用静态表而不是运行时拼帧，是为了让 4 条命令和 voise_command.c
 * 中的协议表一一对应，后续对照调试更直观。
 */
static const uint8_t k_answer_cmd_table[4][5] = {
    {0xAA, 0x55, 0x01, 0x10, 0xFB},
    {0xAA, 0x55, 0x01, 0x11, 0xFB},
    {0xAA, 0x55, 0x01, 0x12, 0xFB},
    {0xAA, 0x55, 0x01, 0x13, 0xFB},
};

/*
 * @brief 通过小 R 语音模块串口发送“答案为零/一/二/三”命令。
 *
 * @param huart 串口句柄，必须已初始化并打开
 * @param answer 答案编号，0~3 对应“零/一/二/三”
 * @return HAL_StatusTypeDef HAL_OK 表示发送成功，其他值表示失败
 */
HAL_StatusTypeDef xiao_R_usart_send_answer(UART_HandleTypeDef *huart, uint8_t answer)
{
    if (huart == NULL || answer >= 4U)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit_IT(huart,
                                (uint8_t *)k_answer_cmd_table[answer],
                                (uint16_t)sizeof(k_answer_cmd_table[answer]));
}
