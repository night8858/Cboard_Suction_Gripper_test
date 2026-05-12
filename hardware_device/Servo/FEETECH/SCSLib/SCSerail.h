#ifndef _SCS_SERIAL_H
#define _SCS_SERIAL_H

#include <stdint.h>

#include "usart.h"

/* 绑定 / 获取 UART 句柄 */
void SCS_SetUART(UART_HandleTypeDef *huart);
UART_HandleTypeDef *SCS_GetUART(void);

/*
 * SCS_SetHalfDuplex：配置总线模式。
 *   enable=1：半双工单线总线（TX/RX 物理共线，发送后有回声），wFlushSCS 会丢弃回声。
 *   enable=0：全双工（TX/RX 独立，无回声），wFlushSCS 不丢弃数据（默认）。
 * 须在 SCS_SetUART() 之后、第一次读写舵机之前调用。
 */
void SCS_SetHalfDuplex(uint8_t enable);

/* 发送/接收底层接口 */
void ftUart_Send(uint8_t *nDat, int nLen);
int  ftUart_Read(uint8_t *nDat, int nLen);
void ftBus_Delay(void);

/*
 * rFlushSCS：清空 UART 接收环形缓冲区中的残留字节。
 * 在发送新读取请求前调用，防止上帧残留或总线回声污染本帧应答解析。
 * 也供 my_feetech.c 等共用 USART1 总线的模块调用。
 */
void rFlushSCS(void);

/*
 * SCS_UART_RxIRQ_Enable：开启 UART RXNE 中断，在 SCS_SetUART 后自动调用，
 *   也可在外部 MX_USART1_UART_Init 完成后主动调用一次。
 *
 * SCS_UART_IRQHandler：UART 字节中断处理函数，须在 stm32f4xx_it.c 中的
 *   USART1_IRQHandler 内，于 HAL_UART_IRQHandler 之前调用：
 *
 *   void USART1_IRQHandler(void)
 *   {
 *       SCS_UART_IRQHandler();      // 先消费 RXNE，推入环形缓冲区
 *       HAL_UART_IRQHandler(&huart1); // HAL 处理其余标志（错误/DMA等）
 *   }
 */
void SCS_UART_RxIRQ_Enable(void);
void SCS_UART_IRQHandler(void);

#endif
