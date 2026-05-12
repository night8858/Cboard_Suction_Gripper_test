/*
 * SCSerail.c
 * 串行舵机底层硬件接口
 * 重构说明（2026-04-23）：
 *   原实现使用 HAL_UART_Receive 阻塞读（100ms 超时）+ HAL_Delay(1ms) 清空延迟，
 *   导致每次 ReadPos 占用 ≥4ms，8路读取合计 ≥32ms，远超 2ms 控制周期，
 *   是运动指令离散跳变的根本原因。
 *
 *   新实现：
 *   1. 使用 UART 中断（RXNE）将接收字节推入环形缓冲区，完全非阻塞。
 *   2. rFlushSCS() 仅清空环形缓冲区，不再调用 HAL_Delay。
 *   3. wFlushSCS() 发送帧后，若为半双工总线（TX_EN 高期间回声），
 *      等待回声字节数等于发送字节数后丢弃，超时 1ms 就退出，不阻塞任务。
 *   4. ftUart_Read() 从环形缓冲区读取，带 1ms 超时（HAL_GetTick 轮询），
 *      既不阻塞 RTOS 调度，也不占用 100ms 超时槽。
 *
 *   调用方无需修改：readSCS/writeSCS/rFlushSCS/wFlushSCS 接口不变。
 */

#include <stdint.h>
#include <string.h>
#include "SCSerail.h"
#include "stm32f4xx_hal.h"

/* =====================================================================
 * 环形接收缓冲区
 * 大小必须是 2 的幂，方便掩码操作避免除法。
 * ===================================================================== */
#define SCS_RX_BUF_SIZE  256U       /* 必须是 2 的幂 */
#define SCS_RX_BUF_MASK  (SCS_RX_BUF_SIZE - 1U)

static uint8_t  s_rx_buf[SCS_RX_BUF_SIZE]; /* 环形缓冲区数据区 */
static volatile uint16_t s_rx_head = 0;    /* 读指针（任务侧修改） */
static volatile uint16_t s_rx_tail = 0;    /* 写指针（中断侧修改） */

/* 发送缓冲区（与原实现保持一致） */
uint8_t  wBuf[128];
uint8_t  wLen = 0;

/* 当前绑定的 UART 句柄 */
static UART_HandleTypeDef *s_scs_uart = &huart1;

/* =====================================================================
 * 公共接口：绑定 / 获取 UART 句柄
 * ===================================================================== */
void SCS_SetUART(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return;
    s_scs_uart = huart;
    /* 绑定新句柄后立刻开启 RXNE 中断，确保接收链路就绪 */
    SCS_UART_RxIRQ_Enable();
}

UART_HandleTypeDef *SCS_GetUART(void)
{
    return s_scs_uart;
}

/* =====================================================================
 * 中断使能 / 初始化：在 planar_robot_arm_all_init() 调用 SCS_SetUART 后
 * 自动触发，外部也可显式调用。
 * ===================================================================== */
void SCS_UART_RxIRQ_Enable(void)
{
    if (s_scs_uart == NULL) return;

    /* 清除溢出错误标志，防止首次 RXNE 被锁死 */
    __HAL_UART_CLEAR_OREFLAG(s_scs_uart);

    /* 使能 RXNE 中断（逐字节接收，推入环形缓冲区） */
    __HAL_UART_ENABLE_IT(s_scs_uart, UART_IT_RXNE);
}

/* =====================================================================
 * UART 接收中断服务（在 stm32f4xx_it.c 中 USART1_IRQHandler 已调用
 * HAL_UART_IRQHandler，HAL 会最终回调此函数）。
 *
 * 注意：本项目 USART1 已配置中断向量，HAL_UART_IRQHandler 内部
 * 处理 RXNE 时若未注册 HAL_UART_Receive_IT 则不会自动消费字节，
 * 因此这里直接操作寄存器读取 DR，绕过 HAL 的状态机。
 * ===================================================================== */
void SCS_UART_IRQHandler(void)
{
    if (s_scs_uart == NULL) return;

    USART_TypeDef *const inst = s_scs_uart->Instance;

    /* 只处理 RXNE（Receive Not Empty）标志 */
    if ((inst->SR & USART_SR_RXNE) != 0U) {
        uint8_t data = (uint8_t)(inst->DR & 0xFFU); /* 读 DR 自动清 RXNE */
        uint16_t next_tail = (s_rx_tail + 1U) & SCS_RX_BUF_MASK;
        if (next_tail != s_rx_head) {
            /* 缓冲区未满，写入数据 */
            s_rx_buf[s_rx_tail] = data;
            s_rx_tail = next_tail;
        }
        /* 缓冲区满则丢弃（舵机协议有校验，上层会重试） */
    }

    /* 处理溢出错误（ORE），防止 UART 锁死 */
    if ((inst->SR & USART_SR_ORE) != 0U) {
        (void)inst->DR; /* 读 DR 清除 ORE */
    }
}

/* =====================================================================
 * 内部辅助：从环形缓冲区读取最多 nLen 字节，带超时（毫秒）。
 * 返回实际读到的字节数。
 * ===================================================================== */
static int scs_ringbuf_read(uint8_t *nDat, int nLen, uint32_t timeout_ms)
{
    int received = 0;
    uint32_t t_start = HAL_GetTick();

    while (received < nLen) {
        if (s_rx_head != s_rx_tail) {
            /* 缓冲区有数据 */
            nDat[received] = s_rx_buf[s_rx_head];
            s_rx_head = (s_rx_head + 1U) & SCS_RX_BUF_MASK;
            received++;
        } else {
            /* 缓冲区暂时为空，检查超时 */
            if ((HAL_GetTick() - t_start) >= timeout_ms) {
                break;
            }
            /* 不调用 osDelay / HAL_Delay，保持任务持续调度 */
        }
    }
    return received;
}

/* =====================================================================
 * 硬件发送接口：使用阻塞 HAL_UART_Transmit（TX 方向通常很快，
 * 1M bps 下 10 字节约 100μs，不影响控制周期）。
 * ===================================================================== */
void ftUart_Send(uint8_t *nDat, int nLen)
{
    if ((s_scs_uart == NULL) || (nDat == NULL) || (nLen <= 0)) {
        return;
    }
    (void)HAL_UART_Transmit(s_scs_uart, nDat, (uint16_t)nLen, 10);
}

/* =====================================================================
 * ftUart_Read：从环形缓冲区读，超时 2ms（原为 100ms 阻塞）。
 * 1M bps 下一个回复帧（7~10字节）约 80~100μs 可完整到达，
 * 2ms 超时足够且不影响 2ms 控制任务节拍。
 * ===================================================================== */
int ftUart_Read(uint8_t *nDat, int nLen)
{
    if ((s_scs_uart == NULL) || (nDat == NULL) || (nLen <= 0)) {
        return 0;
    }
    return scs_ringbuf_read(nDat, nLen, 2U);
}

/* =====================================================================
 * ftBus_Delay：原为 HAL_Delay(1)，现为空操作。
 * rFlushSCS 在数据帧发送前调用，目的是确保上一帧回声已消费；
 * 新实现中 rFlushSCS 直接清空缓冲区，此延迟不再需要。
 * ===================================================================== */
void ftBus_Delay(void)
{
    /* 不再阻塞，改为清空接收缓冲区由 rFlushSCS 完成 */
}

/* =====================================================================
 * UART 读/写底层接口（SCS 协议层调用）
 * ===================================================================== */

/* readSCS：从环形缓冲区取数据 */
int readSCS(unsigned char *nDat, int nLen)
{
    return ftUart_Read(nDat, nLen);
}

/* writeSCS：将数据追加到发送缓冲区（延迟到 wFlushSCS 批量发出） */
int writeSCS(unsigned char *nDat, int nLen)
{
    while (nLen--) {
        if (wLen < sizeof(wBuf)) {
            wBuf[wLen] = *nDat;
            wLen++;
            nDat++;
        }
    }
    return wLen;
}

int writeByteSCS(unsigned char bDat)
{
    if (wLen < sizeof(wBuf)) {
        wBuf[wLen] = bDat;
        wLen++;
    }
    return wLen;
}

/* =====================================================================
 * rFlushSCS：接收刷新——清空环形缓冲区中的残留字节。
 * 在每次新指令发送前调用，防止上帧残留数据污染本帧解析。
 * 不再调用 HAL_Delay，消除 1ms 固定延迟。
 * ===================================================================== */
void rFlushSCS(void)
{
    /* 原子地将读指针追上写指针，丢弃所有缓冲数据 */
    s_rx_head = s_rx_tail;
}

/* =====================================================================
 * wFlushSCS：发送刷新——将发送缓冲区内容通过 UART 发出。
 * 半双工模式下 TX 和 RX 共线，STM32 发出的字节会被自己接收（回声）。
 * 发送完成后等待回声字节到达环形缓冲区并丢弃，避免污染下一帧应答。
 * 等待超时设为 2ms，防止回声丢失时死锁。
 * ===================================================================== */
/* s_half_duplex：置 1 时 wFlushSCS() 在发送后丢弃等量回声字节（半双工单线总线）；
 * 置 0 时不做回声丢弃（全双工独立 TX/RX，无物理回声）。
 * 根据实际硬件接线在 SCS_SetUART() 后调用 SCS_SetHalfDuplex() 配置。
 * 默认 0（全双工），避免误把舵机应答当回声丢弃。 */
static uint8_t s_half_duplex = 0U;

void SCS_SetHalfDuplex(uint8_t enable)
{
    s_half_duplex = enable ? 1U : 0U;
}

void wFlushSCS(void)
{
    if (wLen == 0) return;

    uint8_t sent_len = wLen;
    ftUart_Send(wBuf, wLen);
    wLen = 0;

    /* 仅在半双工（TX/RX 物理共线）模式下丢弃回声字节。
     * HAL_UART_Transmit() 等待 TC（传输完成）后才返回，此时所有回声字节
     * 已进入环形缓冲区，立即读取即可清除，无需等待。
     * 全双工模式下无回声，跳过此步骤，避免把舵机应答误认为回声丢弃。 */
    if (s_half_duplex) {
        uint8_t discard_buf[128];
        (void)scs_ringbuf_read(discard_buf, sent_len, 1U);
    }
}

