#ifndef COMMAND_DECODE_4DOF_H
#define COMMAND_DECODE_4DOF_H

#include <stdbool.h>
#include <stdint.h>

#include "Dof4_Arm.h"

#define CMD4_FRAME_HEADER_BYTE  0xBBu
#define CMD4_FRAME_TAIL_BYTE1   0xFFu
#define CMD4_FRAME_TAIL_BYTE2   0xEEu

#define CMD4_FEEDBACK           0x01u
#define CMD4_POSE_CONTROL       0x02u
#define CMD4_ACTION_CONTROL     0x03u
#define CMD4_VALVE_CONTROL      0x04u
#define CMD4_ANSWER_CONTROL     0x05u
#define CMD4_PUMP_CONTROL       0x06u
#define CMD4_TARGET_ACTION_CONTROL 0x07u
#define CMD4_DIAGNOSTIC         0x08u

/* PC 下发的 4DOF 动作命令。
 * 0x11/0x12:      DATA = arm_id + x/y/z，arm_id=0 左臂、1 右臂，xyz 为 float32 小端，单位 m。
 * 0x14/0x15:      DATA = arm_id，固定背部动作，关节路径由 PC 专用状态机决定。
 * 0x21:           DATA = left_x/y/z + right_x/y/z，双臂动态取块，六个坐标均为 float32 小端，单位 m。
 * 0x22:           无 DATA 段，双臂固定放块到背部。 */
#define CMD4_PICK_BLOCK         0x11u
#define CMD4_PLACE_BLOCK        0x12u
#define CMD4_PUT_BLOCK_BACK     0x14u
#define CMD4_GET_BLOCK_BACK     0x15u
#define CMD4_PICK_BLOCK_ALL     0x21u
#define CMD4_PUT_BLOCK_BACK_ALL 0x22u

#define CMD4_ARM_START          0x99u
#define CMD4_ACTION_DONE        0xCCu

/*
 * BB 01 feedback:
 *   BB 01
 *   L_x L_y L_z L_pitch R_x R_y R_z R_pitch (8 float, m/rad)
 *   valve0..3 sw0..3 reserved FF EE CRC8
 */
#define CMD4_FRAME_FEEDBACK_LEN  46u

/*
 * BB 02 pose control:
 *   BB 02 arm_id x y z pitch FF EE CRC8
 *   arm_id + 4 floats requires 22 bytes total.
 */
#define CMD4_FRAME_POSE_LEN      22u
#define CMD4_FRAME_ACTION_LEN     6u
#define CMD4_FRAME_VALVE_LEN      7u
#define CMD4_FRAME_ANSWER_LEN     8u
#define CMD4_FRAME_PUMP_LEN      10u

/*
 * BB 07 dynamic target action:
 *   BB 07 arm_id operation x y z FF EE CRC8
 *   arm_id/operation + 3 float32 LE requires 19 bytes total.
 */
#define CMD4_FRAME_TARGET_ACTION_LEN 19u

/* PC 专用 4DOF 动作帧长。
 * - 单臂可控目标点: BB cmd arm_id x y z FF EE CRC8 = 18B
 * - 单臂背部固定动作: BB cmd arm_id FF EE CRC8 = 6B
 * - 双臂可控目标点: BB 21 Lxyz Rxyz FF EE CRC8 = 29B
 * - 双臂背部固定动作: BB 22 FF EE CRC8 = 5B */
#define CMD4_FRAME_SINGLE_TARGET_ACTION_LEN 18u
#define CMD4_FRAME_SINGLE_BACK_ACTION_LEN    6u
#define CMD4_FRAME_DUAL_TARGET_ACTION_LEN   29u
#define CMD4_FRAME_DUAL_BACK_ACTION_LEN      5u

/*
 * BB 08 clipping diagnostic (STM32 -> PC only):
 *   arm_id mode reason joint_mask
 *   requested_pose[4] requested_joints[4] limited_joints[4]
 *   limited_pose[4] target_servo_pos[4] FF EE CRC8
 */
#define CMD4_FRAME_DIAGNOSTIC_LEN 81u
#define CMD4_FRAME_ARM_START_LEN  17u  /* BB 99 offX offY offZ FF EE CRC8，offset float32 LE 单位 mm */
#define CMD4_FRAME_ACTION_DONE_LEN 5u  /* BB CC FF EE CRC8，无 DATA 段 */

#define CMD4_FRAME_MAX_LEN       CMD4_FRAME_DIAGNOSTIC_LEN

#define CMD4_ARM_LEFT            0u
#define CMD4_ARM_RIGHT           1u

uint8_t cmd4_crc8_calc(const uint8_t *data, uint16_t len);
uint16_t cmd4_build_diagnostic_frame(uint8_t arm_id,
                                     const Dof4_ClipDiagnostic *diagnostic,
                                     uint8_t *frame,
                                     uint16_t frame_size);

void cmd4_send_feedback(void);
void cmd4_send_diagnostic(void);
void cmd4_send_action_done(void);
void cmd4_rx_process(void);
void cmd4_rx_feed(const uint8_t *buf, uint32_t len);

bool cmd4_manual_pose_active(uint8_t arm_id);
void cmd4_clear_manual_pose(void);

#endif /* COMMAND_DECODE_4DOF_H */
