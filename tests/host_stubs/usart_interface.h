#ifndef HOST_STUB_USART_INTERFACE_H
#define HOST_STUB_USART_INTERFACE_H

#include <stdint.h>
#include "usart.h"

int xiao_R_usart_send_answer(UART_HandleTypeDef *huart, uint8_t answer);

#endif
