#ifndef COMMAND_DECODE_H
#define COMMAND_DECODE_H

#include <stdint.h>

/* ════════════════════════════════════════════════════════════════
 * 通讯协议帧格式常量
 * ════════════════════════════════════════════════════════════════
 *
 * 所有帧的结构：
 *   [帧头 AA CMD] [数据区] [帧尾 FF EE] [CRC8]
 *
 * CRC8（poly=0x07，SMBUS），计算范围：从帧头 AA 到帧尾 FF EE（含）
 * ──────────────────────────────────────────────────────────────── */

#define FRAME_HEADER_BYTE1   0xAAu  /* 帧头第1字节 */
#define FRAME_HEADER_BYTE2   0x01u  /* 预留，AA 01 反馈帧命令字 */
#define FRAME_TAIL_BYTE1     0xFFu  /* 帧尾第1字节 */
#define FRAME_TAIL_BYTE2     0xEEu  /* 帧尾第2字节 */

/* 各命令字 */
#define CMD_FEEDBACK         0x01u  /* AA 01：MCU→Host 状态反馈 */
#define CMD_ARM_CONTROL      0x02u  /* AA 02：Host→MCU 机械臂控制 */
#define CMD_GIMBAL_CONTROL   0x03u  /* AA 03：Host→MCU 云台控制   */
#define CMD_VALVE_CONTROL    0x04u  /* AA 04：Host→MCU 电磁阀控制 */

/**
 * 各帧总长度（字节）: 帧头(2) + 数据区 + 帧尾(2) + CRC8(1)
 *
 * AA 01 反馈帧（29字节）:
 *   AA 01
 *   LFx_H LFx_L LFy_H LFy_L   (4B)
 *   RFx_H RFx_L RFy_H RFy_L   (4B)
 *   LBx_H LBx_L LBy_H LBy_L   (4B)
 *   RBx_H RBx_L RBy_H RBy_L   (4B)
 *   YAW_H YAW_L PITCH_H PITCH_L (4B)
 *   SOL12 SOL34 SW12 SW34      (4B)
 *   FF EE CRC8
 */
#define FRAME_FEEDBACK_LEN   29u

/**
 * AA 02 机械臂控制帧（10字节）:
 *   AA 02 armID x_H x_L y_H y_L FF EE CRC8
 */
#define FRAME_ARM_CTRL_LEN   10u

/**
 * AA 03 云台控制帧（10字节）:
 *   AA 03 gimbalID YAW_H YAW_L PITCH_H PITCH_L FF EE CRC8
 */
#define FRAME_GIMBAL_CTRL_LEN 10u

/**
 * AA 04 电磁阀控制帧（7字节）:
 *   AA 04 valveID state FF EE CRC8
 */
#define FRAME_VALVE_CTRL_LEN  7u

/* 接收状态机能处理的最大帧长 */
#define FRAME_MAX_LEN        FRAME_FEEDBACK_LEN

/* 机械臂 ID（与 arm_id_e 一致，此处重定义供上位机协议使用） */
#define ARM_LF            0u  /* 左前 */
#define ARM_RF            1u  /* 右前 */
#define ARM_LB            2u  /* 左后 */
#define ARM_RB            3u  /* 右后 */

/* ════════════════════════════════════════════════════════════════
 * 解析结果载体结构体
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief AA 02：机械臂末端目标位置命令
 */
typedef struct {
    uint8_t arm_id;   /* 机械臂 ID，取值 ARM_LF ~ ARM_RB */
    int16_t target_x; /* 末端目标 X 坐标，单位 mm */
    int16_t target_y; /* 末端目标 Y 坐标，单位 mm */
} cmd_arm_t;

/**
 * @brief AA 03：云台目标角度命令
 */
typedef struct {
    uint8_t gimbal_id; /* 云台 ID（当前仅0） */
    int16_t target_yaw;   /* 目标 YAW 角，单位度 */
    int16_t target_pitch; /* 目标 PITCH 角，单位度 */
} cmd_gimbal_t;

/**
 * @brief AA 04：电磁阀控制命令
 */
typedef struct {
    uint8_t valve_id; /* 电磁阀 ID，取值 0~3 */
    uint8_t state;    /* 状态：1=打开，0=关闭 */
} cmd_valve_t;

/* ════════════════════════════════════════════════════════════════
 * 公开接口
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 计算 CRC8（poly=0x07，SMBUS，init=0x00）
 * @param data  数据指针
 * @param len   数据长度（字节）
 * @return      计算得到的 CRC8 值
 */
uint8_t crc8_calc(const uint8_t *data, uint16_t len);

/**
 * @brief 发送 AA 01 状态反馈帧（MCU→Host）
 *        读取 Arm_LF/RF/LB/RB 末端坐标、Gimbal 当前角度、g_solenoid_state
 *        拼装29字节帧并通过虚拟串口发送，建议以100Hz周期调用
 */
void cmd_send_feedback(void);

/**
 * @brief 处理接收到的数据流（任务循环中周期调用）
 *        内部从 VCP 缓冲区逐字节读取并送入状态机，
 *        解析完整有效帧后自动分发执行对应控制命令
 */
void cmd_rx_process(void);

#endif /* COMMAND_DECODE_H */
