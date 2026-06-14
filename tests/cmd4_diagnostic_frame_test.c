#include "command_decode_4dof.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void require_true(bool value, const char *message)
{
    if (!value) {
        printf("FAIL: %s\n", message);
        exit(1);
    }
}

static float get_float_le(const uint8_t *data)
{
    float value = 0.0f;
    uint8_t bytes[sizeof(float)] = {data[0], data[1], data[2], data[3]};
    memcpy(&value, bytes, sizeof(value));
    return value;
}

int main(void)
{
    Dof4_ClipDiagnostic diagnostic;
    memset(&diagnostic, 0, sizeof(diagnostic));
    diagnostic.control_mode = DOF4_CONTROL_MODE_JOINT;
    diagnostic.reason = DOF4_CLIP_REASON_JOINT_LIMIT | DOF4_CLIP_REASON_SERVO_LIMIT;
    diagnostic.joint_mask = 0x0AU;
    diagnostic.requested_pose = (Dof4_Pose){1.0f, -2.0f, 3.5f, -0.25f};
    diagnostic.requested_joints = (Dof4_JointState){{0.1f, 0.2f, 0.3f, 0.4f}};
    diagnostic.limited_joints = (Dof4_JointState){{-0.1f, -0.2f, -0.3f, -0.4f}};
    diagnostic.limited_pose = (Dof4_Pose){4.0f, 5.0f, 6.0f, 0.75f};
    diagnostic.target_servo_pos[0] = 0;
    diagnostic.target_servo_pos[1] = 1024;
    diagnostic.target_servo_pos[2] = 2048;
    diagnostic.target_servo_pos[3] = 4095;

    uint8_t frame[CMD4_FRAME_DIAGNOSTIC_LEN];
    const uint16_t len = cmd4_build_diagnostic_frame(CMD4_ARM_RIGHT,
                                                      &diagnostic,
                                                      frame,
                                                      sizeof(frame));
    require_true(len == CMD4_FRAME_DIAGNOSTIC_LEN, "diagnostic frame length");
    require_true(frame[0] == CMD4_FRAME_HEADER_BYTE, "diagnostic header");
    require_true(frame[1] == CMD4_DIAGNOSTIC, "diagnostic command");
    require_true(frame[2] == CMD4_ARM_RIGHT, "diagnostic arm id");
    require_true(frame[3] == DOF4_CONTROL_MODE_JOINT, "diagnostic mode");
    require_true(frame[4] == diagnostic.reason, "diagnostic reason");
    require_true(frame[5] == diagnostic.joint_mask, "diagnostic joint mask");
    require_true(fabsf(get_float_le(&frame[6]) - 1.0f) < 1.0e-6f,
                 "requested pose little endian");
    require_true(fabsf(get_float_le(&frame[22]) - 0.1f) < 1.0e-6f,
                 "requested joints offset");
    require_true(fabsf(get_float_le(&frame[38]) + 0.1f) < 1.0e-6f,
                 "limited joints offset");
    require_true(fabsf(get_float_le(&frame[54]) - 4.0f) < 1.0e-6f,
                 "limited pose offset");
    require_true(frame[70] == 0x00U && frame[71] == 0x00U,
                 "servo 0 little endian");
    require_true(frame[72] == 0x00U && frame[73] == 0x04U,
                 "servo 1024 little endian");
    require_true(frame[74] == 0x00U && frame[75] == 0x08U,
                 "servo 2048 little endian");
    require_true(frame[76] == 0xFFU && frame[77] == 0x0FU,
                 "servo 4095 little endian");
    require_true(frame[78] == CMD4_FRAME_TAIL_BYTE1 &&
                 frame[79] == CMD4_FRAME_TAIL_BYTE2,
                 "diagnostic tail");
    require_true(frame[80] == cmd4_crc8_calc(frame, 80U), "diagnostic CRC");
    require_true(cmd4_build_diagnostic_frame(CMD4_ARM_LEFT,
                                             &diagnostic,
                                             frame,
                                             CMD4_FRAME_DIAGNOSTIC_LEN - 1U) == 0U,
                 "short diagnostic buffer rejected");

    printf("cmd4 diagnostic frame test passed\n");
    return 0;
}
