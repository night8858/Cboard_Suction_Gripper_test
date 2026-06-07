#ifndef COMMAND_DECODE_4DOF_H
#define COMMAND_DECODE_4DOF_H

#include <stdbool.h>
#include <stdint.h>

#define CMD4_FRAME_HEADER_BYTE  0xBBu
#define CMD4_FRAME_TAIL_BYTE1   0xFFu
#define CMD4_FRAME_TAIL_BYTE2   0xEEu

#define CMD4_FEEDBACK           0x01u
#define CMD4_POSE_CONTROL       0x02u
#define CMD4_ACTION_CONTROL     0x03u
#define CMD4_VALVE_CONTROL      0x04u
#define CMD4_ANSWER_CONTROL     0x05u
#define CMD4_PUMP_CONTROL       0x06u
#define CMD4_ARM_START          0x99u

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
#define CMD4_FRAME_ARM_START_LEN  5u  /* BB 99 FF EE CRC8，无 DATA 段 */

#define CMD4_FRAME_MAX_LEN       CMD4_FRAME_FEEDBACK_LEN

#define CMD4_ARM_LEFT            0u
#define CMD4_ARM_RIGHT           1u

uint8_t cmd4_crc8_calc(const uint8_t *data, uint16_t len);

void cmd4_send_feedback(void);
void cmd4_rx_process(void);
void cmd4_rx_feed(const uint8_t *buf, uint32_t len);

bool cmd4_manual_pose_active(uint8_t arm_id);
void cmd4_clear_manual_pose(void);

#endif /* COMMAND_DECODE_4DOF_H */
