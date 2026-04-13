#ifndef VIRTUAL_SERIAL_PORT_H
#define VIRTUAL_SERIAL_PORT_H

#include <stdint.h>

/**
 * @brief 初始化虚拟串口模块（环形缓冲区清零）
 */
void vcp_init(void);

/**
 * @brief 通过 USB VCP 发送数据（内部封装 CDC_Transmit_FS，USB 忙时最多重试3次）
 * @param buf  待发送数据缓冲区指针
 * @param len  发送字节数
 * @return 0=发送成功，1=发送失败（USB 繁忙或断开）
 */
uint8_t vcp_transmit(const uint8_t *buf, uint16_t len);

/**
 * @brief USB CDC 接收回调 —— 由 CDC_Receive_FS 调用，将数据写入环形缓冲区
 * @param buf  接收到的数据指针
 * @param len  接收字节数
 */
void vcp_on_receive(const uint8_t *buf, uint32_t len);

/**
 * @brief 从环形缓冲区读取一个字节
 * @param byte  输出字节的指针
 * @return 1=成功读到数据，0=缓冲区为空
 */
uint8_t vcp_rx_read_byte(uint8_t *byte);

#endif /* VIRTUAL_SERIAL_PORT_H */
