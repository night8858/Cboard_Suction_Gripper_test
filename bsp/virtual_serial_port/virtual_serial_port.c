#include "virtual_serial_port.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_def.h"
#include "cmsis_os.h"

/* ────────────────────────────────────────────────────────────────
 * 环形缓冲区（中断/回调写入，任务轮询读取）
 * 容量 256 字节，索引用 uint8_t 自然溢出实现循环
 * ──────────────────────────────────────────────────────────────── */
#define VCP_RX_BUF_SIZE  256u

static uint8_t  s_rx_buf[VCP_RX_BUF_SIZE]; /* 环形缓冲区存储 */
static uint8_t  s_rx_head = 0;              /* 写指针（由回调写入） */
static uint8_t  s_rx_tail = 0;              /* 读指针（由任务读取） */

/* 连通状态缓存：1=上一次检测为已连通，0=未连通 */
static uint8_t  s_last_connected = 0u;

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USB 发送重试次数上限 */
#define VCP_TX_RETRY_MAX  3u

/* ──────────────────────────────────────────────────────────────── */

/**
 * @brief 判断 USB CDC 当前是否处于可通信状态
 *
 * 条件说明：
 * 1) USB 设备已经进入 CONFIGURED 状态（枚举完成）；
 * 2) CDC 类数据结构已就绪（pClassData 非空）。
 *
 * @return 1=可通信，0=不可通信
 */
static uint8_t vcp_is_usb_ready(void)
{
    if ((hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) && (hUsbDeviceFS.pClassData != NULL)) {
        return 1u;
    }
    return 0u;
}

/**
 * @brief 刷新连通状态并在断连沿触发时清空接收缓冲
 *
 * 断连后清空缓冲可避免：
 * - 上位机重新连接时误处理旧数据；
 * - 业务层继续消费“历史残留字节”。
 *
 * @return 1=当前已连通，0=当前未连通
 */
 
static uint8_t vcp_refresh_connection(void)
{
    uint8_t connected = vcp_is_usb_ready();

    if ((s_last_connected == 1u) && (connected == 0u)) {
        /* 检测到由“连通→断开”的边沿，立即丢弃历史接收数据 */
        s_rx_head = 0u;
        s_rx_tail = 0u;
    }

    s_last_connected = connected;
    return connected;
}

/**
 * @brief 查询虚拟串口是否已与上位机连通
 * @return 1=已连通，0=未连通
 */
uint8_t vcp_is_connected(void)
{
    return vcp_refresh_connection();
}

/**
 * @brief 初始化虚拟串口模块（环形缓冲区清零）
 */
void vcp_init(void)
{
    s_rx_head = 0u;
    s_rx_tail = 0u;

    /* 记录初始化时刻的连通状态，避免首次调用出现错误边沿判断 */
    s_last_connected = vcp_is_usb_ready();
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

    /* 未连通时直接返回失败，不进入 CDC 发送流程 */
    if (vcp_refresh_connection() == 0u) {
        return 1;
    }

    do {
        /* 重试过程中若检测到断连，立即退出 */
        if (vcp_refresh_connection() == 0u) {
            return 1;
        }

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
    /* 未连通时禁止读取，且在 vcp_refresh_connection 内部会处理断连清缓冲 */
    if (vcp_refresh_connection() == 0u) {
        return 0u;
    }

    if (byte == NULL || s_rx_tail == s_rx_head) {
        return 0; /* 缓冲区为空 */
    }

    *byte    = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint8_t)(s_rx_tail + 1u);
    return 1;
}


