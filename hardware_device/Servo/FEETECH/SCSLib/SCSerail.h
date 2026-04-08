#ifndef _SCS_SERIAL_H
#define _SCS_SERIAL_H

#include <stdint.h>

#include "usart.h"

void SCS_SetUART(UART_HandleTypeDef *huart);
UART_HandleTypeDef *SCS_GetUART(void);

void ftUart_Send(uint8_t *nDat, int nLen);
int ftUart_Read(uint8_t *nDat, int nLen);
void ftBus_Delay(void);

#endif