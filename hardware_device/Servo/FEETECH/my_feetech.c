#include "my_feetech.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "SCSerail.h"
#include "stm32f4xx_hal.h"
#include "bsp_usart.h"

/*
 * =========================================================================
 * my_feetech.c  -  飞特 FT/SMS/STS 系列舵机应用层驱动
 * =========================================================================
 * 重构说明（2026-04-26）：
 *   原实现通过 DMA + 流式中间缓冲区接收数据。
 *   实际硬件串口由 SCS_SetUART() 绑定，并由 SCSerail.c 统一管理
 *   （RXNE 中断 + 256 字节环形缓冲区，在绑定 UART 的 IRQ 中调用
 *    SCS_UART_IRQHandler 完成逐字节推入）。
 *   本次重构删除所有自建 DMA/中断基础设施，直接复用 SCSerail 传输层：
 *
 *     发送  : ftUart_Send(buf, len)
 *                — 阻塞式 HAL_UART_Transmit，1Mbps 下 10 字节约 100us
 *     接收  : ftUart_Read(buf, len)
 *                — 从环形缓冲区取数，内置 2ms HAL_GetTick 轮询，无 osDelay
 *     清空  : rFlushSCS()
 *                — 将读指针追上写指针，原子丢弃所有残留字节
 *
 *   接收延迟从原来的 >=20ms（osDelay(1)*N 次循环）降低到 <5ms。
 *
 * 协议帧格式（飞特舵机标准协议）：
 *   指令帧：FF FF  ID  Length  Instruction  Param0..ParamN  CheckSum
 *   应答帧：FF FF  ID  Length  ERROR        Param0..ParamN  CheckSum
 *   Length   = Instruction/ERROR(1) + Params(n) + CheckSum(1) 之和
 *   CheckSum = ~(ID + Length + Instr/ERROR + Param0 + ... + ParamN) & 0xFF
 *
 * !! 共用总线注意事项 !!
 *   my_feetech.c 和 SCSLib（ReadPos/WritePosEx 等）共用同一条已绑定舵机总线
 *   及 SCSerail 环形缓冲区。调用方必须串行操作，严禁多 RTOS 任务并发调用，
 *   否则发送的读帧和收到的应答帧会交叉污染。
 * =========================================================================
 */

/* =========================================================================
 * 协议常量
 * ========================================================================= */

/** READ DATA 指令码（0x02）：请求舵机返回指定寄存器的数据 */
#define FTSTS_READ_INST         0x02U

/** WRITE DATA 指令码（0x03）：向舵机指定寄存器写入数据 */
#define FTSTS_WRITE_INST        0x03U

/**
 * FTSTS_REPLY_TIMEOUT_MS：等待舵机应答帧的超时窗口（单位：ms）。
 *
 * 计算依据（1Mbps 波特率）：
 *   单帧 8 字节传输时间 = 8 * (1/1Mbps * 10bit) = 80us
 *   总线延迟 + 舵机处理延迟通常 < 500us
 *   5ms 超时提供约 60x 余量，足够可靠。
 *
 * 若波特率降为 115200bps，帧传输约 700us，建议将此值改为 15ms。
 */
#define FTSTS_REPLY_TIMEOUT_MS  5U

/**
 * FTSTS_PARSE_BUF_SIZE：帧解析本地缓冲区大小（字节）。
 *
 * 最大单帧长度估算：
 *   帧头(2) + ID(1) + Length(1) + ERROR(1) + 最多16参数 + CS(1) = 22 字节
 * 64 字节留有充足余量，也可吸收噪声字节而不溢出。
 */
#define FTSTS_PARSE_BUF_SIZE    64U

/* =========================================================================
 * 私有状态
 * ========================================================================= */

/**
 * s_last_frame[]：按舵机 ID（0x00~0xFF）索引的应答帧缓存。
 *
 * 每次成功解析到一帧应答时更新对应槽位，timestamp_ms 同步置为
 * HAL_GetTick()。timestamp_ms == 0 表示该 ID 从未收到有效应答。
 * 外部可通过 FTSTS_get_last_frame() 查询任意 ID 的最新状态。
 */
static ftsts_rx_frame_t s_last_frame[256];

/* =========================================================================
 * 内部辅助函数
 * ========================================================================= */

/**
 * @brief 计算飞特舵机协议校验码。
 *
 * 算法：将 data[0..len-1] 累加（超过 255 取低字节），然后按位取反。
 * 校验范围从帧的 ID 字节开始，覆盖到最后一个参数字节（不含校验码本身）。
 *
 * @param data  指向帧的 ID 字节（即 0xFF 0xFF 之后的第一个字节）
 * @param len   参与校验的字节数 = ID(1) + Length(1) + Instr/ERROR(1) + Params(n)
 * @return      校验码字节
 */
static uint8_t ftsts_checksum(const uint8_t *data, uint8_t len)
{
    uint16_t sum = 0U;
    uint8_t  i;

    for (i = 0U; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(~sum);
}

/**
 * @brief 将已校验的完整应答帧存入帧缓存，并更新时间戳。
 *
 * 只有通过校验和验证的帧才应调用此函数。
 *
 * @param id    舵机 ID（取自帧的第 3 字节）
 * @param frame 指向帧起始处（0xFF 0xFF），调用方保证帧长度合法
 */
static void ftsts_cache_frame(uint8_t id, const uint8_t *frame)
{
    ftsts_rx_frame_t *dst = &s_last_frame[id];

    /* frame[3] = Length = ERROR(1) + Params(n) + CS(1)
     * 所以参数字节数 = Length - 2                        */
    uint8_t length        = frame[3];
    uint8_t parameter_len = (length >= 2U) ? (uint8_t)(length - 2U) : 0U;

    /* 防止参数超出固定大小缓冲区 */
    if (parameter_len > FTSTS_MAX_PARAM_LEN) {
        parameter_len = FTSTS_MAX_PARAM_LEN;
    }

    dst->id            = id;
    dst->length        = length;
    dst->error         = frame[4];          /* ERROR/状态字节 */
    dst->parameter_len = parameter_len;
    if (parameter_len > 0U) {
        memcpy(dst->parameter, &frame[5], parameter_len);
    }
    dst->checksum     = frame[(uint16_t)length + 3U]; /* 帧末尾校验码 */
    dst->timestamp_ms = HAL_GetTick();
}

/**
 * @brief 向总线发送一帧，并丢弃半双工回声字节。
 *
 * 飞特舵机使用单线半双工 TTL 总线：STM32 通过同一引脚收发，
 * 发出的每个字节都会立刻被自己的 RX 侧接收（回声）。
 * 若不丢弃回声，下次调用 ftUart_Read 会先读到自己发出的数据，
 * 导致帧解析错误或 ID 误匹配。
 *
 * 此函数在 ftUart_Send 完成后立即调用 ftUart_Read(discard, len)，
 * 利用 SCSerail 内部的 2ms 超时等待回声字节到达并丢弃。
 *
 * @param frame  发送帧首地址（const，内部强转为 uint8_t* 以适配 HAL 接口）
 * @param len    发送字节数
 */
static void ftsts_send_discard_echo(const uint8_t *frame, uint8_t len)
{
    uint8_t discard[32];

    /* 发送：强转 const 以匹配 HAL Transmit 接口，实际不修改数据 */
    ftUart_Send((uint8_t *)(uintptr_t)frame, (int)len);

    /* 丢弃回声：限制 discard 缓冲区上限防止栈溢出 */
    uint8_t to_read = (len <= (uint8_t)sizeof(discard)) ? len : (uint8_t)sizeof(discard);
    (void)ftUart_Read(discard, (int)to_read);
}

/**
 * @brief 在字节缓冲区中搜索并解析一个飞特应答帧。
 *
 * 搜索同步头 0xFF 0xFF，验证帧长度和校验和。
 * 成功则调用 ftsts_cache_frame() 更新缓存，并（可选地）填充 out。
 *
 * @param buf      原始字节数组（从环形缓冲区增量读取后存放的本地缓冲）
 * @param buf_len  buf 中有效字节数
 * @param out      解析成功时填充应答帧内容；传 NULL 仅更新缓存
 * @return         true = 成功解析到一个有效帧；false = 无有效帧
 */
static bool ftsts_try_parse_frame(const uint8_t *buf, uint16_t buf_len,
                                   ftsts_rx_frame_t *out)
{
    uint16_t i;

    /* 最短有效应答帧：FF FF ID Length(=2) ERROR CS = 6 字节 */
    if (buf == NULL || buf_len < 6U) {
        return false;
    }

    /* 搜索同步头 0xFF 0xFF，允许前面有噪声字节 */
    for (i = 0U; i + 1U < buf_len; i++) {
        if (buf[i] == 0xFFU && buf[i + 1U] == 0xFFU) {
            break;
        }
    }

    /* 同步头找不到，或找到后剩余数据不足 6 字节 */
    if (i + 1U >= buf_len || (buf_len - i) < 6U) {
        return false;
    }

    const uint8_t *f      = &buf[i];  /* f 指向 0xFF 0xFF */
    uint8_t        id     = f[2];
    uint8_t        length = f[3];

    /* Length 至少为 2（ERROR + CS），过小则帧格式错误 */
    if (length < 2U) {
        return false;
    }

    /* 完整帧字节数：帧头(2) + ID(1) + Length(1) + Length 字节 */
    uint16_t frame_len = (uint16_t)length + 4U;
    if ((buf_len - i) < frame_len) {
        return false;   /* 帧尚未完整接收，等待更多数据 */
    }

    /* 校验：覆盖 ID + Length + ERROR + Params 共 (length+1) 字节
     * 注意：length 本身包含 CS，所以 length+1 = ID 之后到 Params 末尾的长度 */
    uint8_t expected_cs = ftsts_checksum(&f[2], (uint8_t)(length + 1U));
    if (expected_cs != f[frame_len - 1U]) {
        return false;   /* 校验失败，帧损坏或噪声误同步 */
    }

    /* 帧合法，写入缓存 */
    ftsts_cache_frame(id, f);

    if (out != NULL) {
        *out = s_last_frame[id];
    }
    return true;
}

/* =========================================================================
 * 公共接口实现
 * ========================================================================= */

/**
 * @brief 初始化帧缓存，清零所有历史记录。
 *
 * 传输层（UART + RXNE 中断）由上层通过 SCS_SetUART() 初始化，
 * 此处不重复绑定。
 * 建议在系统上电初始化阶段调用一次，确保缓存状态干净。
 */
void FTSTS_init(void)
{
    memset(s_last_frame, 0, sizeof(s_last_frame));
}

/**
 * @brief 向指定舵机发送 READ DATA 指令（仅发送，不阻塞等待应答）。
 *
 * 指令帧（8字节）：
 *   FF FF  ID  0x04  0x02  addr  data_len  CheckSum
 *   其中 Length = 0x04 = Instruction(1) + addr(1) + data_len(1) + CS(1)
 *
 * 发送完成后自动丢弃半双工回声。
 * 调用方若需读取应答，可随后调用 servo_get_position()（已封装完整流程），
 * 或自行调用 ftUart_Read() + ftsts_try_parse_frame()。
 *
 * @param id        目标舵机 ID（0x00~0xFE；0xFE 为广播地址）
 * @param addr      寄存器起始地址
 * @param data_len  请求读取的字节数
 */
void FTSTS_read_data(uint8_t id, uint8_t addr, uint8_t data_len)
{
    uint8_t frame[8];

    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = id;
    frame[3] = 0x04U;
    frame[4] = FTSTS_READ_INST;
    frame[5] = addr;
    frame[6] = data_len;
    frame[7] = ftsts_checksum(&frame[2], 5U);

    ftsts_send_discard_echo(frame, (uint8_t)sizeof(frame));
}

/**
 * @brief 查询帧缓存，获取指定舵机最近一次有效应答帧的副本。
 *
 * 此函数不发送任何总线指令，仅读取内存缓存。
 * 可在需要最新状态但不想再发一次读请求时使用。
 *
 * @param id        舵机 ID
 * @param out_frame 输出缓冲区，不可为 NULL
 * @return          true  = 找到有效缓存（timestamp_ms != 0）
 *                  false = 该 ID 从未收到有效应答，或 out_frame 为 NULL
 */
bool FTSTS_get_last_frame(uint8_t id, ftsts_rx_frame_t *out_frame)
{
    if (out_frame == NULL || s_last_frame[id].timestamp_ms == 0U) {
        return false;
    }
    *out_frame = s_last_frame[id];
    return true;
}

/**
 * @brief 查询指定舵机当前位置（步进值 0~4095）。
 *
 * 完整收发流程封装：
 *   1. rFlushSCS()            —— 清空环形缓冲区残留，防止脏数据污染应答
 *   2. FTSTS_read_data()      —— 发送读 current_position（0x38）指令并丢回声
 *   3. ftUart_Read() 循环     —— 增量读取环形缓冲区数据到本地缓冲
 *   4. ftsts_try_parse_frame() —— 搜索同步头、验证校验和、解析参数
 *   5. 返回位置值或错误码
 *
 * 全程不调用 osDelay，不主动让出 RTOS 调度权，超时后直接返回 -1。
 * 1Mbps 波特率下通常 < 1ms 完成；ftUart_Read 内部最多等待 2ms。
 *
 * @param id   舵机 ID（使用 int 与 SCSLib ReadPos 接口保持一致）
 * @return     >= 0  ：当前位置步进值（0~4095）
 *             -1    ：超时，舵机未响应或总线故障
 *             < -1  ：舵机报错，返回值为 -(error_code)
 */
int servo_get_position(int id)
{
    uint8_t  rx_buf[FTSTS_PARSE_BUF_SIZE];
    uint16_t rx_len   = 0U;
    uint32_t start_ms = HAL_GetTick();
    int      n;

    /* 步骤 1：清空缓冲区，丢弃上次操作残留或总线干扰字节 */
    rFlushSCS();

    /* 步骤 2：发送读取 current_position 寄存器（0x38）指令，读 2 字节
     *         半双工回声已在 FTSTS_read_data -> ftsts_send_discard_echo 内丢弃 */
    FTSTS_read_data((uint8_t)id, (uint8_t)current_position, 2U);

    /* 步骤 3+4：在超时窗口内循环读取并尝试解析
     * 应答帧格式（8字节）：FF FF  ID  04  ERROR  PosL  PosH  CS
     * ftUart_Read 每次最多等待 2ms（内部超时），总超时由 FTSTS_REPLY_TIMEOUT_MS 控制。
     * 循环内不调用 osDelay，保持任务持续运行。                                      */
    while ((HAL_GetTick() - start_ms) < FTSTS_REPLY_TIMEOUT_MS) {

        /* 增量追加：从缓冲区末尾读取新到字节，避免重复处理已读数据 */
        n = ftUart_Read(&rx_buf[rx_len], (int)(sizeof(rx_buf) - rx_len));
        if (n > 0) {
            rx_len = (uint16_t)(rx_len + (uint16_t)n);
        }

        /* 尝试在已收数据中解析一帧 */
        ftsts_rx_frame_t frame;
        if (ftsts_try_parse_frame(rx_buf, rx_len, &frame)) {
            /* 验证 ID 匹配，防止接到其他舵机（共用总线时的残留帧）的应答 */
            if (frame.id == (uint8_t)id) {
                if (frame.error != 0U) {
                    /* 舵机返回错误状态码，取反作为负数返回 */
                    return -(int)(uint8_t)frame.error;
                }
                if (frame.parameter_len >= 2U) {
                    /* 位置值小端存储：低字节在前 */
                    return (int)((uint16_t)frame.parameter[0] |
                                 ((uint16_t)frame.parameter[1] << 8));
                }
            }
        }

        /* 本地缓冲区满且尚未解析成功（极端情况：总线持续有噪声）：
         * 丢弃全部内容，重新开始收集，不让旧数据持续阻塞解析。 */
        if (rx_len >= sizeof(rx_buf)) {
            rx_len = 0U;
        }
    }

    return -1;  /* 超时：舵机无响应或总线故障 */
}

/**
 * @brief 清除舵机 EPROM 写入锁（寄存器 0x30 写 0x00）。
 *
 * 飞特舵机上电后 EPROM 处于锁定状态，所有写操作被拒绝。
 * 修改 ID、运动模式等存储参数之前，必须先调用此函数解锁。
 * 解锁后建议执行写操作，完成后重新上电或写 0x01 加锁。
 *
 * @param id  目标舵机 ID
 */
void FTSTS_clear_writelock(int id)
{
    uint8_t frame[8];

    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = (uint8_t)id;
    frame[3] = 0x04U;
    frame[4] = FTSTS_WRITE_INST;
    frame[5] = 0x30U;   /* EPROM_LOCK 寄存器地址 */
    frame[6] = 0x00U;   /* 0x00 = 解锁；0x01 = 加锁 */
    frame[7] = ftsts_checksum(&frame[2], 5U);

    /* 写命令通常触发应答帧，提前 flush 以免被下次读操作误消费 */
    rFlushSCS();
    ftsts_send_discard_echo(frame, (uint8_t)sizeof(frame));
}

/**
 * @brief 设置总线上唯一舵机的 ID（广播写入，寄存器 0x05）。
 *
 * 使用广播地址 0xFE 发送，总线上所有舵机均会执行。
 * 因此调用前须确保总线上只连接了一个需要更改 ID 的舵机。
 * 写入 EPROM 前须先调用 FTSTS_clear_writelock() 解锁。
 *
 * @param id  要写入的新 ID 值（0x00~0xFE）
 */
void FTSTS_setID(int id)
{
    uint8_t frame[8];

    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = 0xFEU;           /* 广播地址：所有舵机均响应 */
    frame[3] = 0x04U;
    frame[4] = FTSTS_WRITE_INST;
    frame[5] = 0x05U;           /* ID 寄存器地址 */
    frame[6] = (uint8_t)id;
    frame[7] = ftsts_checksum(&frame[2], 5U);

    rFlushSCS();
    ftsts_send_discard_echo(frame, (uint8_t)sizeof(frame));
}

/**
 * @brief 设置指定舵机最大力矩限制（固定为 700，寄存器 0x10，2字节小端）。
 *
 * 700 对应约 68% 额定力矩，用于防止过载保护。
 * 若需动态调整，将 max_f 改为参数传入。
 *
 * @param id  目标舵机 ID
 */
void FTSTS_setMAX_F(int id)
{
    const uint16_t max_f = 700U;
    uint8_t frame[9];

    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = (uint8_t)id;
    frame[3] = 0x05U;           /* Length = WRITE(1) + addr(1) + 2字节数据 + CS(1) */
    frame[4] = FTSTS_WRITE_INST;
    frame[5] = 0x10U;                            /* Max Torque 寄存器起始地址 */
    frame[6] = (uint8_t)(max_f & 0xFFU);         /* Low  byte */
    frame[7] = (uint8_t)((max_f >> 8U) & 0xFFU); /* High byte */
    frame[8] = ftsts_checksum(&frame[2], 6U);

    rFlushSCS();
    ftsts_send_discard_echo(frame, (uint8_t)sizeof(frame));
}

/**
 * @brief 驱动指定舵机以给定速度运动到目标角度。
 *
 * 指令帧（13字节）格式：
 *   FF FF  ID  09  03  2A  PosL PosH  00 00  SpdL SpdH  CS
 *   寄存器 0x2A = Goal Position（起始），写入 7 字节：
 *     Pos(2) + Time(2，固定0) + Speed(2) + 无（ACC 单独寄存器）
 *
 * 角度-步进换算（FT 系列）：
 *   步进值 = angle / 300.0 * 1023
 *   量程：0° ~ 300°，对应步进 0 ~ 1023
 *
 * !! 注意：SMS/STS 系列量程 0~360°，步进 0~4095，换算公式不同。
 *    若混用两种型号，请在调用层区分并使用对应公式。
 *
 * @param id    目标舵机 ID
 * @param angle 目标角度（度，FT 系列 0~300°）
 * @param speed 运动速度（0=使用内部最大速度；单位见手册）
 */
void FTSTS_servo_write_pos(uint8_t id, float angle, float speed)
{
    uint8_t  tx_buffer[13];
    int16_t  pos_value   = (int16_t)(angle / 300.0f * 1023.0f);
    int16_t  speed_value = (int16_t)speed;

    tx_buffer[0]  = 0xFFU;
    tx_buffer[1]  = 0xFFU;
    tx_buffer[2]  = id;
    tx_buffer[3]  = 0x09U;           /* Length = WRITE(1)+addr(1)+7字节数据+CS(1) */
    tx_buffer[4]  = FTSTS_WRITE_INST;
    tx_buffer[5]  = 0x2AU;           /* Goal Position 寄存器起始地址 */
    tx_buffer[6]  = (uint8_t)(pos_value & 0xFF);            /* PosL  */
    tx_buffer[7]  = (uint8_t)((pos_value >> 8) & 0xFF);     /* PosH  */
    tx_buffer[8]  = 0x00U;           /* TimeL（不使用时间控制，设为 0） */
    tx_buffer[9]  = 0x00U;           /* TimeH */
    tx_buffer[10] = (uint8_t)(speed_value & 0xFF);          /* SpdL  */
    tx_buffer[11] = (uint8_t)((speed_value >> 8) & 0xFF);   /* SpdH  */
    tx_buffer[12] = ftsts_checksum(&tx_buffer[2], 10U);

    rFlushSCS();
    ftsts_send_discard_echo(tx_buffer, (uint8_t)sizeof(tx_buffer));
}

/**
 * @brief FT 系列同步写位置（预留接口，暂未实现）。
 *
 * 飞特 FT 系列同步写协议格式与 SMS/STS 系列有差异，
 * 实现前须核对对应型号手册中的同步写（Sync Write）章节。
 *
 * @param id    舵机 ID
 * @param angle 目标角度
 * @param time  运动时间（ms）
 * @param speed 速度
 */
void FTSTS_servo_Syncwrite_pos(uint8_t id, float angle, float time, float speed)
{
    /* TODO: 实现 FT 系列同步写指令，参考手册 Sync Write 章节 */
    (void)id;
    (void)angle;
    (void)time;
    (void)speed;
}

/**
 * @brief UART 接收完成/空闲回调（HAL 弱函数覆盖）。
 *
 * 本回调仅处理 USART3（上位机指令接收，DMA + 空闲中断方式）。
 * 舵机总线由 SCSerail.c 的 RXNE 中断环形缓冲区独立管理，
 * 不依赖此回调，无需在此处理。
 *
 * @param huart  触发回调的 UART 句柄
 * @param Size   本次接收到的字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* USART3：转发给上位机指令解析模块 */
    usart_cmd_rx_on_event(huart, Size);

    /* 舵机 UART 由 SCS_UART_IRQHandler 管理，无需在此回调中处理 */
}
