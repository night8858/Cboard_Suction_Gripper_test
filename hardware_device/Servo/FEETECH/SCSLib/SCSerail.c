/*
 * SCServo.c
 * ���ض��Ӳ���ӿڲ����
 * ����: 2024.12.2
 * ����: txl
 */
#include <stdint.h>

#include "SCSerail.h"

uint8_t wBuf[128];
uint8_t wLen = 0;

static UART_HandleTypeDef *s_scs_uart = &huart1;

void SCS_SetUART(UART_HandleTypeDef *huart)
{
	if (huart != NULL) {
		s_scs_uart = huart;
	}
}

UART_HandleTypeDef *SCS_GetUART(void)
{
	return s_scs_uart;
}

void ftUart_Send(uint8_t *nDat, int nLen)
{
	if ((s_scs_uart == NULL) || (nDat == NULL) || (nLen <= 0)) {
		return;
	}

	(void)HAL_UART_Transmit(s_scs_uart, nDat, (uint16_t)nLen, 100);
}

int ftUart_Read(uint8_t *nDat, int nLen)
{
	if ((s_scs_uart == NULL) || (nDat == NULL) || (nLen <= 0)) {
		return 0;
	}

	if (s_scs_uart->RxState != HAL_UART_STATE_READY) {
		(void)HAL_UART_AbortReceive(s_scs_uart);
	}

	__HAL_UART_CLEAR_OREFLAG(s_scs_uart);

	if (HAL_UART_Receive(s_scs_uart, nDat, (uint16_t)nLen, 100) == HAL_OK) {
		return nLen;
	}

	return 0;
}

void ftBus_Delay(void)
{
	HAL_Delay(1);
}

//UART �������ݽӿ�
int readSCS(unsigned char *nDat, int nLen)
{
	return ftUart_Read(nDat, nLen);
}

//UART �������ݽӿ�
int writeSCS(unsigned char *nDat, int nLen)
{
	while(nLen--){
		if(wLen<sizeof(wBuf)){
			wBuf[wLen] = *nDat;
			wLen++;
			nDat++;
		}
	}
	return wLen;
}

int writeByteSCS(unsigned char bDat)
{
	if(wLen<sizeof(wBuf)){
		wBuf[wLen] = bDat;
		wLen++;
	}
	return wLen;
}

//���ջ�����ˢ��
void rFlushSCS()
{
	ftBus_Delay();
}

//���ͻ�����ˢ��
void wFlushSCS()
{
	if(wLen){
		ftUart_Send(wBuf, wLen);
		wLen = 0;
	}
}
