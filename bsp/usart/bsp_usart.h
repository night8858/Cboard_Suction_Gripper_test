#ifndef BSP_USART_H
#define BSP_USART_H
#include "usart.h"

void usart_tx_dma_init(UART_HandleTypeDef *huart);
extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
void usart_printf(const char *fmt,...);

void uart_dma_printf(UART_HandleTypeDef *huart, char *fmt, ...);
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

void usart_cmd_rx_init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef usart_cmd_rx_start(void);
void usart_cmd_rx_on_event(UART_HandleTypeDef *huart, uint16_t size);

#endif
