#include "virtual_serial_port.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"

/* ────────────────────────────────────────────────────────────────
 * 环形缓冲区（中断/回调写入，任务轮询读取）
 * 容量 256 字节，索引用 uint8_t 自然溢出实现循环
 * ──────────────────────────────────────────────────────────────── */
#define VCP_RX_BUF_SIZE  256u

static uint8_t  s_rx_buf[VCP_RX_BUF_SIZE]; /* 环形缓冲区存储 */
static uint8_t  s_rx_head = 0;              /* 写指针（由回调写入） */
static uint8_t  s_rx_tail = 0;              /* 读指针（由任务读取） */

/* USB 发送重试次数上限 */
#define VCP_TX_RETRY_MAX  3u

/* ──────────────────────────────────────────────────────────────── */

/**
 * @brief 初始化虚拟串口模块（环形缓冲区清零）
 */
void vcp_init(void)
{
    s_rx_head = 0;
    s_rx_tail = 0;
}

/**
 * @brief 通过 USB VCP 发送数据
 *        若 USB 端点忙，间隔 1ms 最多重试 VCP_TX_RETRY_MAX 次
 * @param buf  待发送数据缓冲区指针
 * @param len  发送字节数
 * @return 0=发送成功，1=发送失败
 */
uint8_t vcp_transmit(const uint8_t *buf, uint16_t len)
{
    uint8_t result;
    uint8_t retry = 0;

    if (buf == NULL || len == 0) {
        return 1;
    }

    do {
        result = CDC_Transmit_FS((uint8_t *)buf, len);
        if (result == 0 /* USBD_OK */) {
            return 0;
        }
        /* USB 忙（USBD_BUSY），等待 1ms 后重试 */
        osDelay(1);
        retry++;
    } while (retry < VCP_TX_RETRY_MAX);

    return 1; /* 超出重试次数，发送失败 */
}

/**
 * @brief USB CDC 接收回调 —— 由 CDC_Receive_FS 调用
 *        将接收到的数据逐字节写入环形缓冲区，缓冲区满则丢弃超出部分
 * @param buf  接收数据指针
 * @param len  接收字节数
 */
void vcp_on_receive(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL) {
        return;
    }

    for (uint32_t i = 0; i < len; i++) {
        uint8_t next_head = (uint8_t)(s_rx_head + 1u);
        if (next_head == s_rx_tail) {
            /* 缓冲区已满，丢弃后续数据 */
            break;
        }
        s_rx_buf[s_rx_head] = buf[i];
        s_rx_head = next_head;
    }
}

/**
 * @brief 从环形缓冲区读取一个字节（由任务轮询调用）
 * @param byte  输出字节的指针
 * @return 1=成功读到数据，0=缓冲区为空
 */
uint8_t vcp_rx_read_byte(uint8_t *byte)
{
    if (byte == NULL || s_rx_tail == s_rx_head) {
        return 0; /* 缓冲区为空 */
    }

    *byte    = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint8_t)(s_rx_tail + 1u);
    return 1;
}
