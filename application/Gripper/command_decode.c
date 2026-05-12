#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "command_decode.h"
#include "virtual_serial_port.h"
#include "variables.h"

#include "Planar_Robot_Arm.h"
#include "gimbal.h"
#include "pneumatic_control.h"

//帧头AA  01 代表 下位机向上位机的反馈，预计设定20hz
//帧头 AA 01
//           LFx占四个uint8_t LFy占四个uint8_t
//           RFx占四个uint8_t RFy占四个uint8_t 
//           LBx占四个uint8_t LBy占四个uint8_t 
//           RBx占四个uint8_t RBy占四个uint8_t 
//           YAW占四个uint8_t PITCH占四个uint8_t 
//           四个电磁阀状态各占一个  四个微动开关各占一个
//帧尾 FF EE
// CRC8校验

//armID共0-3，x高八位 y低八位 x高八位 y低八位 
//帧头 AA 02 代表 上位机向下位机的机械臂控制命令
//帧头 AA 02  
//armID  x占四个uint8_t    y占四个uint8_t     
//帧尾 FF EE
// CRC8校验
//并在接收后转换为

//gimbal共一个，id为0
//帧头 AA 03 代表 上位机向下位机的云台控制命令
//帧头 AA 03  gimbalID  YAW四个uint8_t     PITCH四个uint8_t
//帧尾 FF EE
// CRC8校验

//valveID共0-3，状态共0-1，例如：AA 04 01 01 代表控制电磁阀1打开，AA 04 01 00 代表控制电磁阀1关闭
//帧头 AA 04 代表 上位机向下位机的电磁阀控制命令
//帧头 AA 04  valveID  状态占个uint8_t
//帧尾 FF EE
// CRC8校验

//AA 05 代表 上位机向下位机的任务赛答案控制命令

//AA 06 代表 上位机向下位机的气泵控制命令
//AA 06 off/on:0/1 speed占四个uint8_t 代表float
/* ────────────────────────────────────────────────────────────────
 * 外部全局变量声明
 * ──────────────────────────────────────────────────────────────── */
extern Planar_Robot_Arm Arm_LF;
extern Planar_Robot_Arm Arm_RF;
extern Planar_Robot_Arm Arm_LB;
extern Planar_Robot_Arm Arm_RB;
extern Gimbal_s Gimbal;
extern Solenoid_state_s solenoid_state;

extern all_pc_command all_pc_command_t;


typedef union {
    float f;
    uint8_t b[4];
} float_bytes_u;

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

static inline void put_float_le(uint8_t *buf, uint8_t *idx, float v)
{
    float_bytes_u fb;
    fb.f = v;
    buf[(*idx)++] = fb.b[0];
    buf[(*idx)++] = fb.b[1];
    buf[(*idx)++] = fb.b[2];
    buf[(*idx)++] = fb.b[3];
}

static inline float get_float_le(const uint8_t *data)
{
    float_bytes_u fb;
    fb.b[0] = data[0];
    fb.b[1] = data[1];
    fb.b[2] = data[2];
    fb.b[3] = data[3];
    return fb.f;
}

/* ════════════════════════════════════════════════════════════════
 * AA 01 反馈帧发送
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 发送 AA 01 状态反馈帧（MCU→Host，53字节，建议20Hz调用）
 *
 * 帧结构：
 *   [0]  0xAA  [1]  0x01
 *   [2..41]  LFx/LFy/RFx/RFy/LBx/LBy/RBx/RBy/YAW/PITCH（每个 float 占 4 字节）
 *   [42..45] 四个电磁阀状态（每个 1 字节）
 *   [46..49] 四个微动开关状态（每个 1 字节，当前填 0）
 *   [50] 0xFF  [51] 0xEE
 *   [52] CRC8（计算范围 [0]~[51]）
 */
void cmd_send_feedback(void)
{
    /* 未与上位机连通时，不执行反馈帧组包与发送 */
    if (vcp_is_connected() == 0u) {
        return;
    }

    uint8_t frame[FRAME_FEEDBACK_LEN];
    uint8_t idx = 0;

    /* 帧头 */
    frame[idx++] = FRAME_HEADER_BYTE1;   /* 0xAA */
    frame[idx++] = CMD_FEEDBACK;          /* 0x01 */

    // float a = 2.547f; /* 测试用，验证大端序存储 */

    /* 四路机械臂末端坐标与云台角度按 float 原始字节发送（LE） */
    put_float_le(frame, &idx, Arm_LF.end_effector_x);
    put_float_le(frame, &idx, Arm_LF.end_effector_y);

    put_float_le(frame, &idx, Arm_RF.end_effector_x);
    put_float_le(frame, &idx, Arm_RF.end_effector_y);

    put_float_le(frame, &idx, Arm_LB.end_effector_x);
    put_float_le(frame, &idx, Arm_LB.end_effector_y);

    put_float_le(frame, &idx, Arm_RB.end_effector_x);
    put_float_le(frame, &idx, Arm_RB.end_effector_y);
    
    put_float_le(frame, &idx, Gimbal.current_YAW);
    put_float_le(frame, &idx, Gimbal.current_PITCH);

    /* 四个电磁阀状态（每个 1 字节） */
    frame[idx++] = (uint8_t)(solenoid_state.state[0] & 0x01u);
    frame[idx++] = (uint8_t)(solenoid_state.state[1] & 0x01u);
    frame[idx++] = (uint8_t)(solenoid_state.state[2] & 0x01u);
    frame[idx++] = (uint8_t)(solenoid_state.state[3] & 0x01u);

    /* 微动开关状态（暂填0，预留4字节） */
    frame[idx++] = 0x00u;
    frame[idx++] = 0x00u;
    frame[idx++] = 0x00u;
    frame[idx++] = 0x00u;

    /* 帧尾 */
    frame[idx++] = FRAME_TAIL_BYTE1;   /* 0xFF */
    frame[idx++] = FRAME_TAIL_BYTE2;   /* 0xEE */

    /* CRC8（覆盖 [0]~[idx-1]，即 AA 到 EE 全帧内容） */
    frame[idx] = crc8_calc(frame, (uint16_t)idx);
    /* idx 此时等于 FRAME_FEEDBACK_LEN - 1，加上 CRC 共53字节 */

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
        case CMD_ARM_CONTROL:         return (uint8_t)(FRAME_ARM_CTRL_LEN         - 5u); /* 14-5=9:  armID x(4B) y(4B) */
        case CMD_GIMBAL_CONTROL:      return (uint8_t)(FRAME_GIMBAL_CTRL_LEN      - 5u); /* 14-5=9:  gimbalID yaw(4B) pitch(4B) */
        case CMD_VALVE_CONTROL:       return (uint8_t)(FRAME_VALVE_CTRL_LEN       - 5u); /* 7-5=2:   valveID state */
        case CMD_PUMP_CONTROL:        return (uint8_t)(FRAME_PUMP_CTRL_LEN        - 5u); /* 10-5=5:  pump_state(1B) speed(4B) */
        case CMD_ANSWER_CONTROL:      return (uint8_t)(FRAME_ANSWER_CTRL_LEN      - 5u); /* 8-5=3:   answer(1B) 00 00 */
        default:                      return 0u;
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
                /* 帧尾错误：若当前字节恰为帧头 0xAA，直接进入 WAIT_H2 加速重同步 */
                s_rx_state  = RX_WAIT_H1;
                s_frame_idx = 0;
                if (byte == FRAME_HEADER_BYTE1) {
                    s_frame_buf[s_frame_idx++] = byte;
                    s_rx_state = RX_WAIT_H2;
                }
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
                if (byte == FRAME_HEADER_BYTE1) {
                    s_frame_buf[s_frame_idx++] = byte;
                    s_rx_state = RX_WAIT_H2;
                }
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
 * 帧分发器：解包已通过 CRC 验证的帧，将解析结果存入 all_pc_command_t
 * 由 cmd_execute_all() 在主循环中统一执行硬件控制
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 解析 AA 02 机械臂末端控制命令并将结果存入 all_pc_command_t
 *        帧数据区：[armID] [x(4B)] [y(4B)]
 * @param data  数据区起始指针（命令字之后的第一字节）
 */
static void handle_arm_control(const uint8_t *data)
{
    uint8_t arm_id = data[0];
    float x = get_float_le(&data[1]);
    float y = get_float_le(&data[5]);

    if (arm_id > ARM_ID_RB) {
        return; /* 非法 ID，丢弃 */
    }

    /* 写入 all_pc_command_t，由 cmd_execute_all 统一执行 */
    switch (arm_id) {
        case ARM_LF:
            all_pc_command_t.LF_target_x = x;
            all_pc_command_t.LF_target_y = y;
            break;
        case ARM_RF:
            all_pc_command_t.RF_target_x = x;
            all_pc_command_t.RF_target_y = y;
            break;
        case ARM_LB:
            all_pc_command_t.LB_target_x = x;
            all_pc_command_t.LB_target_y = y;
            break;
        case ARM_RB:
            all_pc_command_t.RB_target_x = x;
            all_pc_command_t.RB_target_y = y;
            break;
        default:
            break;
    }
}

/**
 * @brief 解析 AA 03 云台控制命令并将结果存入 all_pc_command_t
 *        帧数据区：[gimbalID] [yaw(4B)] [pitch(4B)]
 * @param data  数据区起始指针
 */
static void handle_gimbal_control(const uint8_t *data)
{
    /* gimbalID 当前仅支持0，此处保留字段但不强制判断 */
    float yaw   = get_float_le(&data[1]);
    float pitch = get_float_le(&data[5]);

    /* 写入 all_pc_command_t，由 cmd_execute_all 统一执行 */
    all_pc_command_t.gimbal_target_yaw   = yaw;
    all_pc_command_t.gimbal_target_pitch = pitch;
}

/**
 * @brief 解析 AA 04 电磁阀控制命令并将结果存入 all_pc_command_t
 *        帧数据区：[valveID] [state]
 * @param data  数据区起始指针
 */
static void handle_valve_control(const uint8_t *data)
{
    uint8_t valve_id = data[0];
    uint8_t state    = data[1] & 0x01u;

    if (valve_id >= 4u) {
        return; /* 非法 ID，丢弃 */
    }

    /* 写入 all_pc_command_t，由 cmd_execute_all 统一执行 */
    switch (valve_id) {
        case 0:
            all_pc_command_t.LF_solenoid_target_state = state;
            break;
        case 1:
            all_pc_command_t.RF_solenoid_target_state = state;
            break;
        case 2:
            all_pc_command_t.LB_solenoid_target_state = state;
            break;
        case 3:
            all_pc_command_t.RB_solenoid_target_state = state;
            break;
        default:
            break;
    }
}

static void handle_answer_control(const uint8_t *data)
{
    ////占位，AA 05 代表 上位机向下位机的任务赛答案控制命令，接收到后以此利用IIC或者串口控制语音模块播放对应语音

}


/**
 * @brief 解析 AA 06 气泵控制命令并将结果存入 all_pc_command_t
 *        帧数据区：[pump_on(1B)] [speed(4B float)]
 * @param data  数据区起始指针
 */
static void handle_pump_control(const uint8_t *data)
{
    uint8_t pump_on = data[0];
    float   speed   = get_float_le(&data[1]);

    /* 写入 all_pc_command_t，由 cmd_execute_all 统一执行 */
    all_pc_command_t.pump_target_speed = (pump_on != 0u) ? speed : 0.0f;
}



/* ════════════════════════════════════════════════════════════════
 * cmd_execute_all —— 统一控制执行函数
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 统一执行函数：根据 all_pc_command 结构体中的全部指令，
 *        一次性控制所有执行器（机械臂、云台、电磁阀、电磁铁、气泵）。
 *
 * 该函数可在控制循环中周期调用（如 50~100Hz），
 * 各执行器内部有轨迹规划/缓启动机制，可安全频繁调用。
 * 传入 NULL 时使用内部的 all_pc_command_t 全局实例。
 *
 * @param cmd  指向 all_pc_command 的指针，传入 NULL 则使用全局实例
 */
void cmd_execute_all(const all_pc_command *cmd)
{
    if (cmd == NULL) {
        cmd = &all_pc_command_t;
    }

    /* ── 机械臂控制（四路，通过模块 API） ── */
    planar_robot_arm_set_target(ARM_ID_LF, cmd->LF_target_x, cmd->LF_target_y);
    planar_robot_arm_set_target(ARM_ID_RF, cmd->RF_target_x, cmd->RF_target_y);
    planar_robot_arm_set_target(ARM_ID_LB, cmd->LB_target_x, cmd->LB_target_y);
    planar_robot_arm_set_target(ARM_ID_RB, cmd->RB_target_x, cmd->RB_target_y);

    /* ── 云台控制（通过模块 API） ── */
    gimbal_set_target_position(&Gimbal, cmd->gimbal_target_yaw, cmd->gimbal_target_pitch);

    /* ── 电磁阀控制（4路继电器，通过模块 API） ── */
    relay_control(0, cmd->LF_solenoid_target_state);
    relay_control(1, cmd->RF_solenoid_target_state);
    relay_control(2, cmd->LB_solenoid_target_state);
    relay_control(3, cmd->RB_solenoid_target_state);

    /* ── 电磁铁控制（与电磁阀共享同一组继电器GPIO，二者互斥使用） ──
     *  若当前系统使用电磁铁而非电磁阀，将下面注释取消，
     *  并将上面 relay_control 调用注释掉即可。
     */
    /*  relay_control(0, cmd->LF_Electromagnet_target_state);
     *  relay_control(1, cmd->RF_Electromagnet_target_state);
     *  relay_control(2, cmd->LB_Electromagnet_target_state);
     *  relay_control(3, cmd->RB_Electromagnet_target_state);
     */

    /* ── 气泵控制（通过模块 API） ── */
    pump_speed_set(cmd->pump_target_speed);
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

        case CMD_PUMP_CONTROL:
            handle_pump_control(data);
            break;

        case CMD_ANSWER_CONTROL:
            handle_answer_control(data);
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

    /* 未连通时不执行接收解析，直接返回 */
    if (vcp_is_connected() == 0u) {
        return;
    }

    /* 一次性消耗当前缓冲区中所有字节，避免单帧跨周期积压 */
    while (vcp_rx_read_byte(&byte)) {
        cmd_rx_state_machine(byte);
    }
}

/**
 * @brief 直接将原始数据缓冲区送入接收状态机（绕过环形缓冲区）
 *        由 CDC_Receive_FS 回调直接调用，实现零拷贝即时解析
 * @param buf  数据指针
 * @param len  数据字节数
 */
void cmd_rx_feed(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0u) {
        return;
    }
    for (uint32_t i = 0u; i < len; i++) {
        cmd_rx_state_machine(buf[i]);
    }
}


