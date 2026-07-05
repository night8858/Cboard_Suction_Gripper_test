#include "command_decode_4dof.h"
#include "cmd4_host_mocks.h"

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

static void require_near(float actual, float expected, const char *message)
{
    if (fabsf(actual - expected) > 1.0e-6f) {
        printf("FAIL: %s expected=%.6f actual=%.6f\n",
               message,
               (double)expected,
               (double)actual);
        exit(1);
    }
}

static void put_float_le(uint8_t *frame, uint8_t *idx, float value)
{
    uint8_t raw[sizeof(float)];
    memcpy(raw, &value, sizeof(raw));
    frame[(*idx)++] = raw[0];
    frame[(*idx)++] = raw[1];
    frame[(*idx)++] = raw[2];
    frame[(*idx)++] = raw[3];
}

static uint8_t finish_frame(uint8_t *frame, uint8_t idx)
{
    frame[idx++] = CMD4_FRAME_TAIL_BYTE1;
    frame[idx++] = CMD4_FRAME_TAIL_BYTE2;
    frame[idx] = cmd4_crc8_calc(frame, idx);
    return (uint8_t)(idx + 1U);
}

static uint8_t build_arm_start_frame(uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = CMD4_ARM_START;
    put_float_le(frame, &idx, 0.0f);
    put_float_le(frame, &idx, 0.0f);
    put_float_le(frame, &idx, 0.0f);
    return finish_frame(frame, idx);
}

static uint8_t build_pose_frame(uint8_t arm_id,
                                float x,
                                float y,
                                float z,
                                float pitch,
                                uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = CMD4_POSE_CONTROL;
    frame[idx++] = arm_id;
    put_float_le(frame, &idx, x);
    put_float_le(frame, &idx, y);
    put_float_le(frame, &idx, z);
    put_float_le(frame, &idx, pitch);
    return finish_frame(frame, idx);
}

static uint8_t build_single_target_frame(uint8_t cmd,
                                         uint8_t arm_id,
                                         float x,
                                         float y,
                                         float z,
                                         uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = cmd;
    frame[idx++] = arm_id;
    put_float_le(frame, &idx, x);
    put_float_le(frame, &idx, y);
    put_float_le(frame, &idx, z);
    return finish_frame(frame, idx);
}

static uint8_t build_dual_target_frame(uint8_t cmd,
                                       float lx, float ly, float lz,
                                       float rx, float ry, float rz,
                                       uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = cmd;
    put_float_le(frame, &idx, lx);
    put_float_le(frame, &idx, ly);
    put_float_le(frame, &idx, lz);
    put_float_le(frame, &idx, rx);
    put_float_le(frame, &idx, ry);
    put_float_le(frame, &idx, rz);
    return finish_frame(frame, idx);
}

int main(void)
{
    uint8_t frame[CMD4_FRAME_MAX_LEN];
    Dof4_Pose pending_pose;
    const Cmd4HostMockState *state;
    uint8_t len;

    cmd4_host_mocks_reset();
    len = build_arm_start_frame(frame);
    cmd4_rx_feed(frame, len);

    len = build_pose_frame(CMD4_ARM_LEFT,
                           1.00f, 2.00f, 3.00f, 0.50f,
                           frame);
    cmd4_rx_feed(frame, len);
    require_true(cmd4_manual_pose_take_pending(CMD4_ARM_LEFT, &pending_pose),
                 "left BB02 pending");
    require_near(pending_pose.x, 1.01f, "left BB02 x bias");
    require_near(pending_pose.y, 1.98f, "left BB02 y bias");
    require_near(pending_pose.z, 3.03f, "left BB02 z bias");
    require_near(pending_pose.pitch, 0.50f, "left BB02 pitch unchanged");

    len = build_pose_frame(CMD4_ARM_RIGHT,
                           -1.00f, -2.00f, -3.00f, -0.50f,
                           frame);
    cmd4_rx_feed(frame, len);
    require_true(cmd4_manual_pose_take_pending(CMD4_ARM_RIGHT, &pending_pose),
                 "right BB02 pending");
    require_near(pending_pose.x, -1.04f, "right BB02 x bias");
    require_near(pending_pose.y, -1.95f, "right BB02 y bias");
    require_near(pending_pose.z, -3.06f, "right BB02 z bias");
    require_near(pending_pose.pitch, -0.50f, "right BB02 pitch unchanged");

    cmd4_host_mocks_reset();
    len = build_single_target_frame(CMD4_PICK_BLOCK,
                                    CMD4_ARM_RIGHT,
                                    0.10f, 0.20f, 0.30f,
                                    frame);
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->single_pick_calls == 1U, "single pick dispatched");
    require_near(state->last_single_target.x, 0.06f, "single right pick x bias");
    require_near(state->last_single_target.y, 0.25f, "single right pick y bias");
    require_near(state->last_single_target.z, 0.24f, "single right pick z bias");
    require_near(state->last_single_target.pitch, 0.0f, "single pick pitch unchanged");

    len = build_single_target_frame(CMD4_PLACE_BLOCK,
                                    CMD4_ARM_LEFT,
                                    0.40f, 0.50f, 0.60f,
                                    frame);
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->single_place_calls == 1U, "single place dispatched");
    require_near(state->last_single_target.x, 0.41f, "single left place x bias");
    require_near(state->last_single_target.y, 0.48f, "single left place y bias");
    require_near(state->last_single_target.z, 0.63f, "single left place z bias");

    len = build_dual_target_frame(CMD4_PICK_BLOCK_ALL,
                                  0.10f, 0.20f, 0.30f,
                                  -0.10f, -0.20f, -0.30f,
                                  frame);
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->dual_pick_calls == 1U, "dual pick dispatched");
    require_near(state->last_left_target.x, 0.11f, "dual pick left x bias");
    require_near(state->last_left_target.y, 0.18f, "dual pick left y bias");
    require_near(state->last_left_target.z, 0.33f, "dual pick left z bias");
    require_near(state->last_right_target.x, -0.14f, "dual pick right x bias");
    require_near(state->last_right_target.y, -0.15f, "dual pick right y bias");
    require_near(state->last_right_target.z, -0.36f, "dual pick right z bias");

    len = build_dual_target_frame(CMD4_PLACE_BLOCK_ALL,
                                  0.50f, 0.60f, 0.70f,
                                  -0.50f, -0.60f, -0.70f,
                                  frame);
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->dual_place_calls == 1U, "dual place dispatched");
    require_near(state->last_left_target.x, 0.51f, "dual place left x bias");
    require_near(state->last_left_target.y, 0.58f, "dual place left y bias");
    require_near(state->last_left_target.z, 0.73f, "dual place left z bias");
    require_near(state->last_right_target.x, -0.54f, "dual place right x bias");
    require_near(state->last_right_target.y, -0.55f, "dual place right y bias");
    require_near(state->last_right_target.z, -0.76f, "dual place right z bias");

    printf("cmd4 target bias test passed\n");
    return 0;
}
