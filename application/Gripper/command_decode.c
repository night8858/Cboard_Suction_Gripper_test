#include <stdio.h>
#include <string.h>
#include "command_decode.h"
#include "virtual_serial_port.h"
#include "variables.h"
#include "Planar_Robot_Arm.h"
#include "gimbal.h"
#include "bsp_solenoid.h"

//帧头AA  01 代表 下位机向上位机的反馈，预计设定100hz
//帧头 AA 01
//           LFx高八位 LFy低八位 LFx高八位 LFy低八位 
//           RFx高八位 RFx低八位 RFp高八位 RF低八位
//           LBx高八位 LBx低八位 LBp高八位 LB低八位 
//           RBx高八位 RBx低八位 RBp高八位 RB低八位 
//           YAW高八位 YAW低八位 PITCH高八位 PITCH低八位
//           电磁阀1/2状态 电磁阀3/4状态 微动开关1/2状态 微动开关3/4状态
//帧尾 FF EE
// CRC8校验

//armID共0-3，x高八位 y低八位 x高八位 y低八位 例如：AA 01 00 64 00 C8 代表机械臂0的末端位置为(100,200)，AA 01 01 2C 01 90 代表机械臂1的末端位置为(300,400)
//帧头 AA 02 代表 上位机向下位机的机械臂控制命令
//帧头 AA 02  armID  x高八位 y低八位 x高八位 y低八位 
//帧尾 FF EE
// CRC8校验

//gimbal共一个，id为0
//帧头 AA 03 代表 上位机向下位机的云台控制命令
//帧头 AA 03  gimbalID  YAW高八位 YAW低八位 PITCH高八位 PITCH低八位
//帧尾 FF EE
// CRC8校验

//valveID共0-3，状态共0-1，例如：AA 04 01 01 代表控制电磁阀1打开，AA 04 01 00 代表控制电磁阀1关闭
//帧头 AA 04 代表 上位机向下位机的电磁阀控制命令
//帧头 AA 04  valveID  状态
//帧尾 FF EE
// CRC8校验

/* ────────────────────────────────────────────────────────────────
 * 外部全局变量声明
 * ──────────────────────────────────────────────────────────────── */
extern Planar_Robot_Arm Arm_LF;
extern Planar_Robot_Arm Arm_RF;
extern Planar_Robot_Arm Arm_LB;
extern Planar_Robot_Arm Arm_RB;
extern Gimbal_s Gimbal;

/* ════════════════════════════════════════════════════════════════
 * CRC8 查找表（poly = 0x07，SMBUS）
 * ════════════════════════════════════════════════════════════════ */
static const uint8_t s_crc8_table[256] = {
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
 * @brief 计算 CRC8（poly=0x07，SMBUS，init=0x00）
 *        计算范围应包含 AA…FF EE 全帧内容（不含 CRC 字节本身）
 * @param data  数据指针
 * @param len   数据长度（字节）
 * @return      计算得到的 CRC8 值
 */
uint8_t crc8_calc(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00u;

    if (data == NULL) {
        return crc;
    }

    for (uint16_t i = 0; i < len; i++) {
        crc = s_crc8_table[crc ^ data[i]];
    }
    return crc;
}

/* ════════════════════════════════════════════════════════════════
 * 辅助宏：将 float 四舍五入为 int16_t，并拆分为高/低字节，最大绝对误差为0.5
 * ════════════════════════════════════════════════════════════════ */
static inline int16_t float_to_int16(float v)
{
    int32_t i = (int32_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
    if      (i >  32767) { i =  32767; }
    else if (i < -32768) { i = -32768; }
    return (int16_t)i;
}

#define PUT_INT16_BE(buf, idx, val) \
    do { \
        (buf)[(idx)]     = (uint8_t)(((int16_t)(val)) >> 8); \
        (buf)[(idx) + 1] = (uint8_t)(((int16_t)(val)) & 0xFFu); \
    } while (0)

/* ════════════════════════════════════════════════════════════════
 * AA 01 反馈帧发送
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 发送 AA 01 状态反馈帧（MCU→Host，29字节，建议100Hz调用）
 *
 * 帧结构：
 *   [0]  0xAA  [1]  0x01
 *   [2-3]   LF x(int16 BE mm)   [4-5]   LF y
 *   [6-7]   RF x                [8-9]   RF y
 *   [10-11] LB x                [12-13] LB y
 *   [14-15] RB x                [16-17] RB y
 *   [18-19] YAW(int16 BE 度)    [20-21] PITCH
 *   [22] 电磁阀1/2状态 bits[0]=valve1, bits[1]=valve2
 *   [23] 电磁阀3/4状态 bits[0]=valve3, bits[1]=valve4
 *   [24] 微动开关1/2状态（暂填0）
 *   [25] 微动开关3/4状态（暂填0）
 *   [26] 0xFF  [27] 0xEE
 *   [28] CRC8（计算范围 [0]~[27]）
 */
void cmd_send_feedback(void)
{
    uint8_t frame[FRAME_FEEDBACK_LEN];
    uint8_t idx = 0;

    /* 帧头 */
    frame[idx++] = FRAME_HEADER_BYTE1;   /* 0xAA */
    frame[idx++] = CMD_FEEDBACK;          /* 0x01 */

    /* 四路机械臂末端坐标（float→int16_t，mm，大端序） */
    PUT_INT16_BE(frame, idx, float_to_int16(Arm_LF.end_effector_x)); idx += 2;
    PUT_INT16_BE(frame, idx, float_to_int16(Arm_LF.end_effector_y)); idx += 2;

    PUT_INT16_BE(frame, idx, float_to_int16(Arm_RF.end_effector_x)); idx += 2;
    PUT_INT16_BE(frame, idx, float_to_int16(Arm_RF.end_effector_y)); idx += 2;

    PUT_INT16_BE(frame, idx, float_to_int16(Arm_LB.end_effector_x)); idx += 2;
    PUT_INT16_BE(frame, idx, float_to_int16(Arm_LB.end_effector_y)); idx += 2;

    PUT_INT16_BE(frame, idx, float_to_int16(Arm_RB.end_effector_x)); idx += 2;
    PUT_INT16_BE(frame, idx, float_to_int16(Arm_RB.end_effector_y)); idx += 2;

    /* 云台当前角度（float→int16_t，度，大端序） */
    PUT_INT16_BE(frame, idx, float_to_int16(Gimbal.current_YAW));   idx += 2;
    PUT_INT16_BE(frame, idx, float_to_int16(Gimbal.current_PITCH)); idx += 2;

    /* 电磁阀状态字节
     *   [22]: bits[0]=valve1状态, bits[1]=valve2状态
     *   [23]: bits[0]=valve3状态, bits[1]=valve4状态
     */
    frame[idx++] = (uint8_t)((g_solenoid_state.state[0] & 0x01u) |
                              ((g_solenoid_state.state[1] & 0x01u) << 1));
    frame[idx++] = (uint8_t)((g_solenoid_state.state[2] & 0x01u) |
                              ((g_solenoid_state.state[3] & 0x01u) << 1));

    /* 微动开关状态（暂填0，预留字节） */
    frame[idx++] = 0x00u;  /* 微动开关1/2，待接入实际GPIO */
    frame[idx++] = 0x00u;  /* 微动开关3/4，待接入实际GPIO */

    /* 帧尾 */
    frame[idx++] = FRAME_TAIL_BYTE1;   /* 0xFF */
    frame[idx++] = FRAME_TAIL_BYTE2;   /* 0xEE */

    /* CRC8（覆盖 [0]~[idx-1]，即 AA 到 EE 全帧内容） */
    frame[idx] = crc8_calc(frame, (uint16_t)idx);
    /* idx 此时等于 FRAME_FEEDBACK_LEN - 1，加上 CRC 共29字节 */

    vcp_transmit(frame, FRAME_FEEDBACK_LEN);
}

/* ════════════════════════════════════════════════════════════════
 * 接收状态机
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 接收状态机状态枚举
 *
 *  WAIT_H1   → 等待帧头第1字节 0xAA
 *  WAIT_H2   → 等待帧头第2字节（命令字）
 *  RECV_DATA → 按命令字长度接收数据区
 *  WAIT_T1   → 等待帧尾第1字节 0xFF
 *  WAIT_T2   → 等待帧尾第2字节 0xEE
 *  VERIFY    → 接收并验证 CRC8
 */
typedef enum {
    RX_WAIT_H1 = 0,
    RX_WAIT_H2,
    RX_RECV_DATA,
    RX_WAIT_T1,
    RX_WAIT_T2,
    RX_VERIFY_CRC,
} rx_state_e;

/* 状态机内部静态上下文 */
static rx_state_e  s_rx_state     = RX_WAIT_H1;
static uint8_t     s_frame_buf[FRAME_MAX_LEN]; /* 当前帧缓冲（含帧头帧尾） */
static uint8_t     s_frame_idx    = 0;         /* 写入偏移 */
static uint8_t     s_data_remain  = 0;         /* 数据区剩余待收字节数 */

/**
 * @brief 获取命令字对应的数据区长度（帧头(2)～帧尾(2)之间的字节数）
 *        返回 0 表示不支持的命令字
 * @param cmd  命令字
 * @return 数据区字节数
 */
static uint8_t get_data_len_by_cmd(uint8_t cmd)
{
    switch (cmd) {
        case CMD_ARM_CONTROL:    return (uint8_t)(FRAME_ARM_CTRL_LEN    - 5u); /* 10-5=5: armID x_H x_L y_H y_L */
        case CMD_GIMBAL_CONTROL: return (uint8_t)(FRAME_GIMBAL_CTRL_LEN - 5u); /* 10-5=5: gimbalID YAW_H YAW_L PITCH_H PITCH_L */
        case CMD_VALVE_CONTROL:  return (uint8_t)(FRAME_VALVE_CTRL_LEN  - 5u); /* 7-5=2:  valveID state */
        default:                 return 0u;
    }
}

/* 前向声明 */
static void cmd_dispatch_frame(const uint8_t *buf, uint8_t len);

/**
 * @brief 将一个字节送入接收状态机
 *        完整有效帧（帧尾正确 + CRC8 通过）自动分发到 cmd_dispatch_frame
 * @param byte  输入字节
 */
static void cmd_rx_state_machine(uint8_t byte)
{
    switch (s_rx_state) {

        /* ── 等待帧头 0xAA ── */
        case RX_WAIT_H1:
            if (byte == FRAME_HEADER_BYTE1) {
                s_frame_idx          = 0;
                s_frame_buf[s_frame_idx++] = byte;  /* 存入 AA */
                s_rx_state           = RX_WAIT_H2;
            }
            break;

        /* ── 等待命令字 ── */
        case RX_WAIT_H2: {
            uint8_t data_len = get_data_len_by_cmd(byte);
            if (data_len == 0u) {
                /* 不支持的命令字，重新等待帧头 */
                s_rx_state = RX_WAIT_H1;
                s_frame_idx = 0;
                break;
            }
            s_frame_buf[s_frame_idx++] = byte;  /* 存入命令字 */
            s_data_remain = data_len;
            s_rx_state    = RX_RECV_DATA;
            break;
        }

        /* ── 接收数据区 ── */
        case RX_RECV_DATA:
            s_frame_buf[s_frame_idx++] = byte;
            s_data_remain--;
            if (s_data_remain == 0u) {
                s_rx_state = RX_WAIT_T1;
            }
            break;

        /* ── 等待帧尾 0xFF ── */
        case RX_WAIT_T1:
            if (byte == FRAME_TAIL_BYTE1) {
                s_frame_buf[s_frame_idx++] = byte;
                s_rx_state = RX_WAIT_T2;
            } else {
                /* 帧尾错误，丢弃当前帧，重新同步 */
                s_rx_state  = RX_WAIT_H1;
                s_frame_idx = 0;
            }
            break;

        /* ── 等待帧尾 0xEE ── */
        case RX_WAIT_T2:
            if (byte == FRAME_TAIL_BYTE2) {
                s_frame_buf[s_frame_idx++] = byte;
                s_rx_state = RX_VERIFY_CRC;
            } else {
                s_rx_state  = RX_WAIT_H1;
                s_frame_idx = 0;
            }
            break;

        /* ── 接收并验证 CRC8 ── */
        case RX_VERIFY_CRC: {
            /* byte 是接收到的 CRC8，s_frame_buf[0..s_frame_idx-1] 为待校验内容 */
            uint8_t expected_crc = crc8_calc(s_frame_buf, s_frame_idx);
            if (byte == expected_crc) {
                /* CRC 通过 —— 分发帧（含 CRC 字节存入缓冲供调试，但分发只看前 s_frame_idx 字节） */
                s_frame_buf[s_frame_idx] = byte;
                cmd_dispatch_frame(s_frame_buf, (uint8_t)(s_frame_idx + 1u));
            }
            /* 无论校验是否通过，重置状态机等待下一帧 */
            s_rx_state  = RX_WAIT_H1;
            s_frame_idx = 0;
            break;
        }

        default:
            s_rx_state  = RX_WAIT_H1;
            s_frame_idx = 0;
            break;
    }
}

/* ════════════════════════════════════════════════════════════════
 * 帧分发器：解包已通过 CRC 验证的帧，执行对应控制
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 解析并执行 AA 02 机械臂末端控制命令
 *        帧数据区：[armID] [x_H] [x_L] [y_H] [y_L]
 * @param data  数据区起始指针（命令字之后的第一字节）
 */
static void handle_arm_control(const uint8_t *data)
{
    uint8_t arm_id = data[0];
    int16_t x = (int16_t)(((uint16_t)data[1] << 8) | data[2]); /* 大端序还原 */
    int16_t y = (int16_t)(((uint16_t)data[3] << 8) | data[4]);

    if (arm_id > ARM_ID_RB) {
        return; /* 非法 ID，丢弃 */
    }

    /* 调用平面机械臂接口设置末端目标位置（mm） */
    planar_robot_arm_set_target((arm_id_e)arm_id, (float)x, (float)y);
}

/**
 * @brief 解析并执行 AA 03 云台控制命令
 *        帧数据区：[gimbalID] [YAW_H] [YAW_L] [PITCH_H] [PITCH_L]
 * @param data  数据区起始指针
 */
static void handle_gimbal_control(const uint8_t *data)
{
    /* gimbalID 当前仅支持0，此处保留字段但不强制判断 */
    int16_t yaw   = (int16_t)(((uint16_t)data[1] << 8) | data[2]);
    int16_t pitch = (int16_t)(((uint16_t)data[3] << 8) | data[4]);

    gimbal_set_target_position(&Gimbal, (float)yaw, (float)pitch);
}

/**
 * @brief 解析并执行 AA 04 电磁阀控制命令
 *        帧数据区：[valveID] [state]
 * @param data  数据区起始指针
 */
static void handle_valve_control(const uint8_t *data)
{
    uint8_t valve_id = data[0];
    uint8_t state    = data[1];

    if (valve_id >= 4u) {
        return; /* 非法 ID，丢弃 */
    }

    /* 更新全局电磁阀指令状态 */
    g_solenoid_state.command[valve_id] = state;

    /* 直接驱动硬件（bsp_solenoid 通道与 valveID 0-based 对应） */
    bsp_solenoid_set(valve_id, state);
}

/**
 * @brief 已校验帧的分发入口
 *        根据帧中命令字（buf[1]）分发给对应处理函数
 * @param buf  完整帧缓冲区指针（含帧头+数据+帧尾+CRC）
 * @param len  帧总长度（字节）
 */
static void cmd_dispatch_frame(const uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len < 5u) {
        return;
    }

    uint8_t cmd       = buf[1];         /* 命令字位于 buf[1] */
    const uint8_t *data = &buf[2];      /* 数据区从 buf[2] 开始 */

    switch (cmd) {
        case CMD_ARM_CONTROL:
            handle_arm_control(data);
            break;

        case CMD_GIMBAL_CONTROL:
            handle_gimbal_control(data);
            break;

        case CMD_VALVE_CONTROL:
            handle_valve_control(data);
            break;

        default:
            /* 未知命令字，忽略 */
            break;
    }
}

/* ════════════════════════════════════════════════════════════════
 * 公开接口：任务循环调用
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 处理接收到的数据流（在任务中周期调用）
 *        从 VCP 环形缓冲区逐字节读取，送入状态机解析，
 *        完整有效帧自动分发执行对应控制命令
 *
 * 使用示例（在任意已有任务循环中）：
 *   for (;;) {
 *       cmd_rx_process();
 *       osDelay(1);
 *   }
 */
void cmd_rx_process(void)
{
    uint8_t byte;

    /* 一次性消耗当前缓冲区中所有字节，避免单帧跨周期积压 */
    while (vcp_rx_read_byte(&byte)) {
        cmd_rx_state_machine(byte);
    }
}
