/**
 * @file    command_decode_4dof.c
 * @brief   4DOF 双臂串口指令解码模块
 *
 * ## 通信协议概述
 *
 * 本模块实现了一套基于帧的二进制串口协议，用于上位机 (PC/树莓派) 与
 * STM32 之间的实时控制通信。协议通过虚拟串口 (VCP) 传输。
 *
 * ### 帧格式 (统一)
 *
 *   ┌──────┬──────┬─────────────────┬──────┬──────┬──────┐
 *   │  HDR │  CMD │   DATA (变长)    │ TAIL1│ TAIL2│ CRC8 │
 *   │ 0xBB │ 1B   │   N bytes        │ 0xFF │ 0xEE │ 1B   │
 *   └──────┴──────┴─────────────────┴──────┴──────┴──────┘
 *
 *   - HDR (Header) : 帧头 0xBB，用于帧同步
 *   - CMD           : 命令字，决定 DATA 段的长度和含义
 *   - DATA          : 可变长度负载，小端字节序
 *   - TAIL1/TAIL2   : 帧尾 0xFF 0xEE，辅助帧完整性校验
 *   - CRC8          : 对整个帧 (含 HDR/CMD/DATA/TAIL) 的 CRC-8 校验
 *
 * ### 命令字一览
 *
 *   | 命令字 | 名称             | 方向       | 说明                         |
 *   |--------|------------------|------------|------------------------------|
 *   | 0x01   | CMD4_FEEDBACK    | STM32→PC   | 周期反馈当前双臂位姿+阀门状态 |
 *   | 0x02   | CMD4_POSE_CONTROL| PC→STM32   | 手动设定单臂目标位姿 (x,y,z,pitch) |
 *   | 0x03   | CMD4_ACTION_CONTROL| PC→STM32 | 触发预设动作 (抓取/放置/舞蹈)  |
 *   | 0x04   | CMD4_VALVE_CONTROL| PC→STM32   | 手动控制单个电磁阀开关         |
 *   | 0x05   | CMD4_ANSWER_CONTROL| PC→STM32  | 语音应答控制 (预留)            |
 *   | 0x06   | CMD4_PUMP_CONTROL | PC→STM32   | 气泵启停 + 转速设置            |
 *
 * ### RX 状态机
 *
 *   接收端使用逐字节状态机解析帧，流程如下：
 *
 *   WAIT_H1 ──(0xBB)──▶ WAIT_CMD ──(合法CMD)──▶ RECV_DATA
 *       ▲                                          │
 *       │                                      (收完N字节)
 *       │                                          ▼
 *       │                                    WAIT_T1 ──(0xFF)──▶ WAIT_T2
 *       │                                         │                │
 *       └──────────(CRC错/非法)──────────────────┘          (0xEE)│
 *                                                                ▼
 *                                                          VERIFY_CRC
 *                                                             │
 *                                                    ┌────────┴────────┐
 *                                                    │ CRC正确→派发     │
 *                                                    │ CRC错误→丢弃     │
 *                                                    └─────────────────┘
 *
 *   - 任何阶段遇到意外字节均复位到 WAIT_H1
 *   - 在 WAIT_T1/T2 阶段若遇到新帧头 0xBB，则放弃当前帧、开始新帧
 *
 * ### 手动控制与动作调度的互斥
 *
 *   - 当动作调度器 (action_4dof) 激活时，拒绝手动位姿指令
 *   - 触发动作时，自动清除手动位姿标记
 *   - 阀门/气泵控制不受动作调度影响，始终可独立操作
 */

#include "command_decode_4dof.h"

#include "Dof4_Arm.h"
#include "action_scheduler_4dof.h"
#include "pneumatic_control.h"
#include "virtual_serial_port.h"

/* ════════════════════════════════════════════════════════════════
 * float / uint8_t[4] 共用体 —— 用于小端字节序的浮点数编解码
 *
 * STM32F4 为小端架构 (Little Endian)，float 在内存中的 4 字节排列
 * 与协议要求一致，可直接通过此共用体进行逐字节读写。
 * ════════════════════════════════════════════════════════════════ */
typedef union {
    float f;          /**< IEEE 754 单精度浮点值 */
    uint8_t b[4];     /**< 小端字节序: b[0]=最低字节, b[3]=最高字节 */
} cmd4_float_bytes_u;

/* ── 外部引用 ── */
extern PumpCtrl g_pump;

/* ════════════════════════════════════════════════════════════════
 * 模块内部状态
 * ════════════════════════════════════════════════════════════════ */

/** @brief 电磁阀状态镜像 (valve shadow)
 *
 * 维护当前 4 路电磁阀 (吸盘) 的最新已知状态，用于反馈帧上报。
 * 索引: [0]=左臂吸盘, [1]=右臂吸盘, [2]=左背吸盘, [3]=右背吸盘
 * 值:   0=关闭, 1=开启
 */
static uint8_t s_valve_shadow[4] = {0u, 0u, 0u, 0u};

/** @brief 手动位姿控制激活标记
 *
 * 当上位机通过 CMD4_POSE_CONTROL 下发目标位姿时置 true；
 * 动作调度器触发后由 cmd4_clear_manual_pose() 统一清除。
 * [0]=左臂, [1]=右臂
 */
static bool s_manual_pose_active[2] = {false, false};

/* ════════════════════════════════════════════════════════════════
 * CRC-8 查表 (多项式 0x07, 初始值 0x00)
 *
 * 使用标准 CRC-8 算法:
 *   - 多项式: x^8 + x^2 + x^1 + 1  (0x07)
 *   - 初始值: 0x00
 *   - 不反序、不异或输出
 *
 * 查表法比逐位计算快约 8 倍，适合嵌入式实时场景。
 * ════════════════════════════════════════════════════════════════ */
static const uint8_t s_cmd4_crc8_table[256] = {
    0x00u, 0x07u, 0x0Eu, 0x09u, 0x1Cu, 0x1Bu, 0x12u, 0x15u,
    0x38u, 0x3Fu, 0x36u, 0x31u, 0x24u, 0x23u, 0x2Au, 0x2Du,
    0x70u, 0x77u, 0x7Eu, 0x79u, 0x6Cu, 0x6Bu, 0x62u, 0x65u,
    0x48u, 0x4Fu, 0x46u, 0x41u, 0x54u, 0x53u, 0x5Au, 0x5Du,
    0xE0u, 0xE7u, 0xEEu, 0xE9u, 0xFCu, 0xFBu, 0xF2u, 0xF5u,
    0xD8u, 0xDFu, 0xD6u, 0xD1u, 0xC4u, 0xC3u, 0xCAu, 0xCDu,
    0x90u, 0x97u, 0x9Eu, 0x99u, 0x8Cu, 0x8Bu, 0x82u, 0x85u,
    0xA8u, 0xAFu, 0xA6u, 0xA1u, 0xB4u, 0xB3u, 0xBAu, 0xBDu,
    0xC7u, 0xC0u, 0xC9u, 0xCEu, 0xDBu, 0xDCu, 0xD5u, 0xD2u,
    0xFFu, 0xF8u, 0xF1u, 0xF6u, 0xE3u, 0xE4u, 0xEDu, 0xEAu,
    0xB7u, 0xB0u, 0xB9u, 0xBEu, 0xABu, 0xACu, 0xA5u, 0xA2u,
    0x8Fu, 0x88u, 0x81u, 0x86u, 0x93u, 0x94u, 0x9Du, 0x9Au,
    0x27u, 0x20u, 0x29u, 0x2Eu, 0x3Bu, 0x3Cu, 0x35u, 0x32u,
    0x1Fu, 0x18u, 0x11u, 0x16u, 0x03u, 0x04u, 0x0Du, 0x0Au,
    0x57u, 0x50u, 0x59u, 0x5Eu, 0x4Bu, 0x4Cu, 0x45u, 0x42u,
    0x6Fu, 0x68u, 0x61u, 0x66u, 0x73u, 0x74u, 0x7Du, 0x7Au,
    0x89u, 0x8Eu, 0x87u, 0x80u, 0x95u, 0x92u, 0x9Bu, 0x9Cu,
    0xB1u, 0xB6u, 0xBFu, 0xB8u, 0xADu, 0xAAu, 0xA3u, 0xA4u,
    0xF9u, 0xFEu, 0xF7u, 0xF0u, 0xE5u, 0xE2u, 0xEBu, 0xECu,
    0xC1u, 0xC6u, 0xCFu, 0xC8u, 0xDDu, 0xDAu, 0xD3u, 0xD4u,
    0x69u, 0x6Eu, 0x67u, 0x60u, 0x75u, 0x72u, 0x7Bu, 0x7Cu,
    0x51u, 0x56u, 0x5Fu, 0x58u, 0x4Du, 0x4Au, 0x43u, 0x44u,
    0x19u, 0x1Eu, 0x17u, 0x10u, 0x05u, 0x02u, 0x0Bu, 0x0Cu,
    0x21u, 0x26u, 0x2Fu, 0x28u, 0x3Du, 0x3Au, 0x33u, 0x34u,
    0x4Eu, 0x49u, 0x40u, 0x47u, 0x52u, 0x55u, 0x5Cu, 0x5Bu,
    0x76u, 0x71u, 0x78u, 0x7Fu, 0x6Au, 0x6Du, 0x64u, 0x63u,
    0x3Eu, 0x39u, 0x30u, 0x37u, 0x22u, 0x25u, 0x2Cu, 0x2Bu,
    0x06u, 0x01u, 0x08u, 0x0Fu, 0x1Au, 0x1Du, 0x14u, 0x13u,
    0xAEu, 0xA9u, 0xA0u, 0xA7u, 0xB2u, 0xB5u, 0xBCu, 0xBBu,
    0x96u, 0x91u, 0x98u, 0x9Fu, 0x8Au, 0x8Du, 0x84u, 0x83u,
    0xDEu, 0xD9u, 0xD0u, 0xD7u, 0xC2u, 0xC5u, 0xCCu, 0xCBu,
    0xE6u, 0xE1u, 0xE8u, 0xEFu, 0xFAu, 0xFDu, 0xF4u, 0xF3u,
};

/**
 * @brief 计算 CRC-8 校验值
 *
 * 对长度为 len 的字节数组 data 计算 CRC-8。
 * 用于帧发送前附加校验、帧接收后完整性验证。
 *
 * @param data 待校验数据指针，可为 NULL (返回 0x00)
 * @param len  数据长度（字节数）
 * @retval uint8_t 8 位 CRC 校验值
 */
uint8_t cmd4_crc8_calc(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00u;

    if (data == NULL) {
        return crc;
    }

    for (uint16_t i = 0; i < len; i++) {
        crc = s_cmd4_crc8_table[crc ^ data[i]];
    }
    return crc;
}

/* ════════════════════════════════════════════════════════════════
 * 浮点数编解码辅助函数 (小端字节序)
 *
 * - cmd4_put_float_le: 将 float 按小端写入帧缓冲区
 * - cmd4_get_float_le: 从帧缓冲区按小端读取 float
 *
 * 所有多字节数值在协议中均为小端序 (Least Significant Byte First)。
 * ════════════════════════════════════════════════════════════════ */

/** @brief 将 float 以小端字节序写入缓冲区，idx 自增 4 */
static inline void cmd4_put_float_le(uint8_t *buf, uint8_t *idx, float v)
{
    cmd4_float_bytes_u fb;
    fb.f = v;
    buf[(*idx)++] = fb.b[0];
    buf[(*idx)++] = fb.b[1];
    buf[(*idx)++] = fb.b[2];
    buf[(*idx)++] = fb.b[3];
}

/** @brief 从缓冲区以小端字节序读取一个 float (不修改位置) */
static inline float cmd4_get_float_le(const uint8_t *data)
{
    cmd4_float_bytes_u fb;
    fb.b[0] = data[0];
    fb.b[1] = data[1];
    fb.b[2] = data[2];
    fb.b[3] = data[3];
    return fb.f;
}

/* ════════════════════════════════════════════════════════════════
 * 反馈帧发送
 *
 * cmd4_send_feedback() 周期性 (由 RTOS 任务调用) 向 PC 上报
 * 当前双臂 TCP 位姿和电磁阀状态，帧长固定 46 字节。
 *
 * 反馈帧格式 (CMD4_FEEDBACK, 0x01):
 *   ┌──────┬──────┬─────────────────────────────────────┬──────┬──────┬──────┬──────┬──────┐
 *   │ 0xBB │ 0x01 │ L_x L_y L_z L_pitch  R_x R_y R_z   │ V0.. │ SW   │ RSVD │ 0xFF │ 0xEE │ CRC8 │
 *   │      │      │ R_pitch (8×float32 LE = 32B)        │ V3   │ 5B   │      │      │      │
 *   └──────┴──────┴─────────────────────────────────────┴──────┴──────┴──────┴──────┴──────┘
 *
 * 仅在 VCP 已连接时发送，避免无意义的数据搬运。
 * ════════════════════════════════════════════════════════════════ */

/** @brief 发送双臂位姿 + 阀门状态反馈帧到上位机 */
void cmd4_send_feedback(void)
{
    /* VCP 未连接时不发送，节省带宽和 CPU */
    if (vcp_is_connected() == 0u) {
        return;
    }

    uint8_t frame[CMD4_FRAME_FEEDBACK_LEN];
    uint8_t idx = 0u;

    /* ── 帧头 + 命令字 ── */
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = CMD4_FEEDBACK;

    /* ── 左臂 TCP 位姿 (4 × float32 LE = 16 字节) ── */
    cmd4_put_float_le(frame, &idx, g_dof4_arm_left.current_pose.x);
    cmd4_put_float_le(frame, &idx, g_dof4_arm_left.current_pose.y);
    cmd4_put_float_le(frame, &idx, g_dof4_arm_left.current_pose.z);
    cmd4_put_float_le(frame, &idx, g_dof4_arm_left.current_pose.pitch);

    /* ── 右臂 TCP 位姿 (4 × float32 LE = 16 字节) ── */
    cmd4_put_float_le(frame, &idx, g_dof4_arm_right.current_pose.x);
    cmd4_put_float_le(frame, &idx, g_dof4_arm_right.current_pose.y);
    cmd4_put_float_le(frame, &idx, g_dof4_arm_right.current_pose.z);
    cmd4_put_float_le(frame, &idx, g_dof4_arm_right.current_pose.pitch);

    /* ── 电磁阀状态 (4 字节, 每字节仅低 1 位有效) ── */
    frame[idx++] = (uint8_t)(s_valve_shadow[0] & 0x01u);  /* 左臂吸盘 */
    frame[idx++] = (uint8_t)(s_valve_shadow[1] & 0x01u);  /* 右臂吸盘 */
    frame[idx++] = (uint8_t)(s_valve_shadow[2] & 0x01u);  /* 左背吸盘 */
    frame[idx++] = (uint8_t)(s_valve_shadow[3] & 0x01u);  /* 右背吸盘 */

    /* ── 微动开关 + 预留 (5 字节) ── */
    frame[idx++] = 0x00u;  /* SW0 (预留) */
    frame[idx++] = 0x00u;  /* SW1 (预留) */
    frame[idx++] = 0x00u;  /* SW2 (预留) */
    frame[idx++] = 0x00u;  /* SW3 (预留) */
    frame[idx++] = 0x00u;  /* 额外预留, 凑齐 46 字节帧长 */

    /* ── 帧尾 + CRC ── */
    frame[idx++] = CMD4_FRAME_TAIL_BYTE1;
    frame[idx++] = CMD4_FRAME_TAIL_BYTE2;
    frame[idx] = cmd4_crc8_calc(frame, (uint16_t)idx);  /* CRC 覆盖 idx 个字节 */

    (void)vcp_transmit(frame, CMD4_FRAME_FEEDBACK_LEN);
}

/* ════════════════════════════════════════════════════════════════
 * RX 帧接收状态机
 *
 * 状态机类型定义与内部变量。逐字节喂入 cmd4_rx_state_machine()，
 * 自动完成帧同步、数据收集、帧尾校验和 CRC 验证。
 *
 * 状态流转见文件头部的 Mermaid 图。
 * ════════════════════════════════════════════════════════════════ */

/** @brief RX 状态机状态枚举 */
typedef enum {
    CMD4_RX_WAIT_H1    = 0,  /**< 等待帧头 0xBB */
    CMD4_RX_WAIT_CMD   = 1,  /**< 等待命令字, 并据此确定数据段长度 */
    CMD4_RX_RECV_DATA  = 2,  /**< 接收 DATA 段 (长度由命令字决定) */
    CMD4_RX_WAIT_T1    = 3,  /**< 等待帧尾第一字节 0xFF */
    CMD4_RX_WAIT_T2    = 4,  /**< 等待帧尾第二字节 0xEE */
    CMD4_RX_VERIFY_CRC = 5,  /**< 核对 CRC8, 通过则派发帧 */
} cmd4_rx_state_e;

/** @brief 当前 RX 状态 */
static cmd4_rx_state_e s_rx_state = CMD4_RX_WAIT_H1;
/** @brief 帧接收缓冲区 (最大 46 字节, 即最长反馈帧) */
static uint8_t s_frame_buf[CMD4_FRAME_MAX_LEN];
/** @brief 当前已写入缓冲区的字节数 */
static uint8_t s_frame_idx = 0u;
/** @brief DATA 段剩余待接收字节数 (由命令字决定) */
static uint8_t s_data_remain = 0u;

/**
 * @brief 根据命令字返回 DATA 段长度 (不含帧头/命令字/帧尾/CRC)
 *
 * 帧总长 = 5 (HDR+CMD+TAIL1+TAIL2+CRC) + DATA_len
 * 所以 DATA_len = FRAME_LEN - 5
 *
 * 未知命令字返回 0，导致状态机立即复位。
 *
 * @param cmd 命令字 (0x01~0x06)
 * @retval uint8_t DATA 段字节数; 0 表示非法命令
 */
static uint8_t cmd4_data_len_by_cmd(uint8_t cmd)
{
    switch (cmd) {
        case CMD4_POSE_CONTROL:
            return (uint8_t)(CMD4_FRAME_POSE_LEN - 5u);
        case CMD4_ACTION_CONTROL:
            return (uint8_t)(CMD4_FRAME_ACTION_LEN - 5u);
        case CMD4_VALVE_CONTROL:
            return (uint8_t)(CMD4_FRAME_VALVE_LEN - 5u);
        case CMD4_ANSWER_CONTROL:
            return (uint8_t)(CMD4_FRAME_ANSWER_LEN - 5u);
        case CMD4_PUMP_CONTROL:
            return (uint8_t)(CMD4_FRAME_PUMP_LEN - 5u);
        default:
            return 0u;
    }
}

/** @brief 帧派发函数声明 (定义见后) */
static void cmd4_dispatch_frame(const uint8_t *buf, uint8_t len);

/**
 * @brief 重置 RX 状态机到初始状态 (WAIT_H1)
 *
 * 在帧同步丢失、CRC 错误或完成一次派发后调用。
 * 不清空缓冲区内容 (会被下次正确的帧头覆盖)。
 */
static void cmd4_rx_reset(void)
{
    s_rx_state = CMD4_RX_WAIT_H1;
    s_frame_idx = 0u;
    s_data_remain = 0u;
}

/**
 * @brief RX 逐字节状态机 —— 从字节流中提取完整帧
 *
 * 这是整个接收链路的核心。每收到一个字节就调用一次，
 * 状态机自动处理帧同步、数据收集和 CRC 校验。
 *
 * 关键设计:
 *   - 在 WAIT_T1/T2 阶段若检测到新帧头 0xBB，
 *     则放弃当前帧并立即开始解析新帧 (防止帧错位累积)
 *   - CRC 通过后调用 cmd4_dispatch_frame() 执行具体命令
 *   - CRC 失败或任何非法字节直接复位，避免错误扩散
 *
 * @param byte 新收到的单个字节
 */
static void cmd4_rx_state_machine(uint8_t byte)
{
    switch (s_rx_state) {
        case CMD4_RX_WAIT_H1:
            /* 等待帧头 0xBB — 在此之前的所有字节均丢弃 */
            if (byte == CMD4_FRAME_HEADER_BYTE) {
                s_frame_idx = 0u;
                s_frame_buf[s_frame_idx++] = byte;
                s_rx_state = CMD4_RX_WAIT_CMD;
            }
            break;

        case CMD4_RX_WAIT_CMD: {
            /* 接收命令字，查表获取 DATA 段长度 */
            uint8_t data_len = cmd4_data_len_by_cmd(byte);
            if (data_len == 0u) {
                /* 非法命令字 → 丢弃整帧 */
                cmd4_rx_reset();
                break;
            }
            s_frame_buf[s_frame_idx++] = byte;
            s_data_remain = data_len;
            s_rx_state = CMD4_RX_RECV_DATA;
            break;
        }

        case CMD4_RX_RECV_DATA:
            /* 逐字节收集 DATA 段，直到收满 */
            s_frame_buf[s_frame_idx++] = byte;
            s_data_remain--;
            if (s_data_remain == 0u) {
                s_rx_state = CMD4_RX_WAIT_T1;
            }
            break;

        case CMD4_RX_WAIT_T1:
            if (byte == CMD4_FRAME_TAIL_BYTE1) {
                s_frame_buf[s_frame_idx++] = byte;
                s_rx_state = CMD4_RX_WAIT_T2;
            } else {
                /* 帧尾错误 — 但允许在此处重同步: 若遇到新帧头则直接开始新帧 */
                cmd4_rx_reset();
                if (byte == CMD4_FRAME_HEADER_BYTE) {
                    s_frame_buf[s_frame_idx++] = byte;
                    s_rx_state = CMD4_RX_WAIT_CMD;
                }
            }
            break;

        case CMD4_RX_WAIT_T2:
            if (byte == CMD4_FRAME_TAIL_BYTE2) {
                s_frame_buf[s_frame_idx++] = byte;
                s_rx_state = CMD4_RX_VERIFY_CRC;
            } else {
                /* 同上: 帧尾错误但允许重同步 */
                cmd4_rx_reset();
                if (byte == CMD4_FRAME_HEADER_BYTE) {
                    s_frame_buf[s_frame_idx++] = byte;
                    s_rx_state = CMD4_RX_WAIT_CMD;
                }
            }
            break;

        case CMD4_RX_VERIFY_CRC: {
            /* CRC 校验: 对整个帧 (不含 CRC 字节本身) 计算 CRC-8 */
            uint8_t expected_crc = cmd4_crc8_calc(s_frame_buf, s_frame_idx);
            if (byte == expected_crc) {
                /* CRC 通过 → 将 CRC 字节也写入缓冲区，然后派发完整帧 */
                s_frame_buf[s_frame_idx] = byte;
                cmd4_dispatch_frame(s_frame_buf, (uint8_t)(s_frame_idx + 1u));
            }
            /* CRC 通过或失败均复位状态机，准备接收下一帧 */
            cmd4_rx_reset();
            break;
        }

        default:
            cmd4_rx_reset();
            break;
    }
}

/* ════════════════════════════════════════════════════════════════
 * 命令处理函数 (Handler)
 *
 * 以下函数由 cmd4_dispatch_frame() 根据命令字分发调用。
 * 各 handler 接收的 data 指针已跳过 HDR+CMD，直接指向 DATA 段首字节。
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 处理 CMD4_POSE_CONTROL (0x02) —— 手动设定单臂目标位姿
 *
 * DATA 段格式 (17 字节):
 *   [0]     arm_id   0=左臂, 1=右臂
 *   [1..4]  x        目标 X 坐标 (float32 LE, 米)
 *   [5..8]  y        目标 Y 坐标
 *   [9..12] z        目标 Z 坐标
 *   [13..16] pitch   目标末端俯仰角 (float32 LE, 弧度)
 *
 * 约束:
 *   - 动作调度器激活时拒绝 (避免与自动动作冲突)
 *   - IK 求解失败时静默忽略, 不阻塞
 *   - 成功设置后标记 s_manual_pose_active 为 true
 */
static void cmd4_handle_pose_control(const uint8_t *data)
{
    uint8_t arm_id = data[0];
    float x = cmd4_get_float_le(&data[1]);
    float y = cmd4_get_float_le(&data[5]);
    float z = cmd4_get_float_le(&data[9]);
    float pitch = cmd4_get_float_le(&data[13]);

    if (arm_id > CMD4_ARM_RIGHT) {
        return;
    }

    if (action_4dof_is_active()) {
        return;
    }

    Dof4_Arm *arm = (arm_id == CMD4_ARM_LEFT) ? &g_dof4_arm_left : &g_dof4_arm_right;
    Dof4_Status st = Dof4_arm_set_target(arm, x, y, z, pitch);
    if (st == DOF4_STATUS_OK) {
        s_manual_pose_active[arm_id] = true;
    }
}

/**
 * @brief 处理 CMD4_ACTION_CONTROL (0x03) —— 触发预设动作
 *
 * DATA 段格式 (1 字节):
 *   [0]  action_id  动作枚举值 (对应 action_state_4dof_e)
 *                   0 = 中止当前动作 (action_4dof_abort)
 *                   1~16 = 触发对应预设动作
 *
 * 触发前自动清除手动位姿标记 (cmd4_clear_manual_pose)。
 * 超出 ACTION_DANCE 范围的 ID 被忽略。
 */
static void cmd4_handle_action_control(const uint8_t *data)
{
    uint8_t action_id = data[0];

    cmd4_clear_manual_pose();

    if (action_id == 0u) {
        action_4dof_abort();
        return;
    }

    if (action_id > (uint8_t)ACTION_DANCE) {
        return;
    }

    (void)action_4dof_trigger((action_state_4dof_e)action_id);
}

/**
 * @brief 处理 CMD4_VALVE_CONTROL (0x04) —— 手动控制单个电磁阀
 *
 * DATA 段格式 (2 字节):
 *   [0]  valve_id  电磁阀编号 (0=左臂吸盘, 1=右臂吸盘, 2=左背吸盘, 3=右背吸盘)
 *   [1]  state     0=关闭/释放, 1=开启/吸附 (仅低 1 位有效)
 *
 * 操作后同步更新 s_valve_shadow 以保持反馈帧准确。
 */
static void cmd4_handle_valve_control(const uint8_t *data)
{
    uint8_t valve_id = data[0];
    uint8_t state = (uint8_t)(data[1] & 0x01u);

    if (valve_id >= 4u) {
        return;
    }

    relay_control(valve_id, state);
    s_valve_shadow[valve_id] = state;
}

/**
 * @brief 处理 CMD4_ANSWER_CONTROL (0x05) —— 语音应答控制 (预留)
 *
 * 当前为占位实现。DATA 段格式待定义。
 */
static void cmd4_handle_answer_control(const uint8_t *data)
{
    (void)data;
}

/**
 * @brief 处理 CMD4_PUMP_CONTROL (0x06) —— 气泵启停 + 转速设置
 *
 * DATA 段格式 (5 字节):
 *   [0]     on_off   0=停止, 非0=启动
 *   [1..4]  speed    目标转速 (float32 LE, RPM)
 *
 * 气泵用于为吸盘提供负压。停止时转速清零。
 */
static void cmd4_handle_pump_control(const uint8_t *data)
{
    uint8_t on_off = data[0];
    float speed = cmd4_get_float_le(&data[1]);

    if (on_off == 0u) {
        g_pump.is_running = false;
        g_pump.target_speed_rpm = 0.0f;
        pump_speed_set(0.0f);
    } else {
        g_pump.is_running = true;
        g_pump.target_speed_rpm = speed;
        pump_speed_set(speed);
    }
}

/**
 * @brief 帧派发 —— 校验通过后根据命令字路由到对应 handler
 *
 * 对帧长做二次校验 (防御性编程)，防止状态机异常导致越界。
 * 5 字节为最小帧长: HDR(1) + CMD(1) + TAIL1(1) + TAIL2(1) + CRC(1)
 *
 * @param buf 完整帧缓冲区 (含 HDR/CMD/DATA/TAIL/CRC)
 * @param len 帧总长度 (字节)
 */
static void cmd4_dispatch_frame(const uint8_t *buf, uint8_t len)
{
    /* 防御: 空指针或帧太短 (至少需要 HDR+CMD+TAIL1+TAIL2+CRC = 5B) */
    if (buf == NULL || len < 5u) {
        return;
    }

    uint8_t cmd = buf[1];            /* 命令字在帧的第 2 字节 */
    const uint8_t *data = &buf[2];   /* DATA 段从第 3 字节开始 */

    /* 根据命令字分发，同时对帧长做二次校验 */
    switch (cmd) {
        case CMD4_POSE_CONTROL:       /* 0x02: 位姿控制, 帧长 22B */
            if (len == CMD4_FRAME_POSE_LEN) {
                cmd4_handle_pose_control(data);
            }
            break;

        case CMD4_ACTION_CONTROL:     /* 0x03: 动作控制, 帧长 6B */
            if (len == CMD4_FRAME_ACTION_LEN) {
                cmd4_handle_action_control(data);
            }
            break;

        case CMD4_VALVE_CONTROL:      /* 0x04: 阀门控制, 帧长 7B */
            if (len == CMD4_FRAME_VALVE_LEN) {
                cmd4_handle_valve_control(data);
            }
            break;

        case CMD4_ANSWER_CONTROL:     /* 0x05: 语音应答 (预留), 帧长 8B */
            if (len == CMD4_FRAME_ANSWER_LEN) {
                cmd4_handle_answer_control(data);
            }
            break;

        case CMD4_PUMP_CONTROL:       /* 0x06: 气泵控制, 帧长 10B */
            if (len == CMD4_FRAME_PUMP_LEN) {
                cmd4_handle_pump_control(data);
            }
            break;

        default:                      /* 未知命令字 → 静默丢弃 */
            break;
    }
}

/* ════════════════════════════════════════════════════════════════
 * 公共 API 实现
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief RX 主循环 —— 从 VCP 读取所有可用字节并喂入状态机
 *
 * 应由 RTOS 任务周期性调用 (例如每 1ms 或 5ms)。
 * 内部循环读取直到 VCP 缓冲区为空，确保不丢字节。
 * VCP 未连接时直接返回，避免空转。
 */
void cmd4_rx_process(void)
{
    uint8_t byte;

    if (vcp_is_connected() == 0u) {
        return;
    }

    while (vcp_rx_read_byte(&byte)) {
        cmd4_rx_state_machine(byte);
    }
}

/**
 * @brief 批量喂入字节 (外部接口)
 *
 * 用于从其他数据源 (如 DMA 缓冲区、中断回调) 批量注入字节。
 * 内部逐字节调用 cmd4_rx_state_machine()。
 *
 * @param buf 数据缓冲区
 * @param len 缓冲区长度 (字节)
 */
void cmd4_rx_feed(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0u) {
        return;
    }

    for (uint32_t i = 0u; i < len; i++) {
        cmd4_rx_state_machine(buf[i]);
    }
}

/**
 * @brief 查询指定臂是否处于手动位姿控制模式
 *
 * 当上位机通过 CMD4_POSE_CONTROL 设置了目标位姿且未被动作覆盖时返回 true。
 * 动作调度器 (action_4dof) 可利用此标记判断是否需要避让手动控制。
 *
 * @param arm_id CMD4_ARM_LEFT (0) 或 CMD4_ARM_RIGHT (1)
 * @retval true  该臂当前由上位机手动控制位姿
 * @retval false 该臂未被手动控制, 或 arm_id 非法
 */
bool cmd4_manual_pose_active(uint8_t arm_id)
{
    if (arm_id > CMD4_ARM_RIGHT) {
        return false;
    }
    return s_manual_pose_active[arm_id];
}

/**
 * @brief 清除所有手动位姿控制标记
 *
 * 在以下场景调用:
 *   - 动作调度器触发新动作时 (让动作接管控制)
 *   - 系统复位 / 紧急停止时
 */
void cmd4_clear_manual_pose(void)
{
    s_manual_pose_active[CMD4_ARM_LEFT] = false;
    s_manual_pose_active[CMD4_ARM_RIGHT] = false;
}
