#include "command_decode_4dof.h"
#include "cmd4_host_mocks.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern PumpCtrl g_pump;

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

static uint8_t build_action_frame(uint8_t action_id, uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = CMD4_ACTION_CONTROL;
    frame[idx++] = action_id;
    return finish_frame(frame, idx);
}

static uint8_t build_valve_frame(uint8_t valve_id, uint8_t state, uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = CMD4_VALVE_CONTROL;
    frame[idx++] = valve_id;
    frame[idx++] = state;
    return finish_frame(frame, idx);
}

static uint8_t build_answer_frame(uint8_t answer, uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = CMD4_ANSWER_CONTROL;
    frame[idx++] = answer;
    frame[idx++] = 0U;
    frame[idx++] = 0U;
    return finish_frame(frame, idx);
}

static uint8_t build_pump_frame(uint8_t on_off, float speed, uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = CMD4_PUMP_CONTROL;
    frame[idx++] = on_off;
    put_float_le(frame, &idx, speed);
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

static uint8_t build_single_back_frame(uint8_t cmd, uint8_t arm_id, uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = cmd;
    frame[idx++] = arm_id;
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

static uint8_t build_dual_back_frame(uint8_t cmd, uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = cmd;
    return finish_frame(frame, idx);
}

static uint8_t build_arm_start_frame(float off_x_mm,
                                     float off_y_mm,
                                     float off_z_mm,
                                     uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = CMD4_ARM_START;
    put_float_le(frame, &idx, off_x_mm);
    put_float_le(frame, &idx, off_y_mm);
    put_float_le(frame, &idx, off_z_mm);
    return finish_frame(frame, idx);
}

static uint8_t build_legacy_target_frame(uint8_t *frame)
{
    uint8_t idx = 0U;
    frame[idx++] = CMD4_FRAME_HEADER_BYTE;
    frame[idx++] = 0x07U;
    for (uint8_t i = 0U; i < 14U; ++i) {
        frame[idx++] = (uint8_t)(i + 1U);
    }
    return finish_frame(frame, idx);
}

static void assert_no_pc_action_dispatch(const Cmd4HostMockState *state,
                                         const char *message)
{
    require_true(state->single_pick_calls == 0U &&
                 state->single_place_calls == 0U &&
                 state->single_put_back_calls == 0U &&
                 state->single_get_back_calls == 0U &&
                 state->dual_pick_calls == 0U &&
                 state->dual_place_calls == 0U &&
                 state->dual_put_back_calls == 0U &&
                 state->dual_get_back_calls == 0U,
                 message);
}

static void assert_all_valves_open(const Cmd4HostMockState *state,
                                   const char *message)
{
    require_true(state->relay_calls >= 4U &&
                 state->relay_state[0] == 1U &&
                 state->relay_state[1] == 1U &&
                 state->relay_state[2] == 1U &&
                 state->relay_state[3] == 1U,
                 message);
}

int main(void)
{
    uint8_t frame[CMD4_FRAME_MAX_LEN];
    const Cmd4HostMockState *state;
    uint8_t len;

    cmd4_host_mocks_reset();
    len = build_pose_frame(CMD4_ARM_LEFT, 0.10f, 0.20f, -0.30f, 0.40f, frame);
    require_true(len == CMD4_FRAME_POSE_LEN, "pose frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->pose_set_calls == 1U, "pose dispatch");
    require_true(state->arm_start_calls == 1U, "pose auto-starts arm");
    assert_all_valves_open(state, "pose auto-start opens all valves");
    require_true(cmd4_manual_pose_active(CMD4_ARM_LEFT), "manual pose active");
    require_near(g_dof4_arm_left.current_pose.x, 0.10f, "pose left x");
    require_near(g_dof4_arm_left.current_pose.pitch, 0.40f, "pose left pitch");

    len = build_action_frame(5U, frame);
    require_true(len == CMD4_FRAME_ACTION_LEN, "action frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->action_trigger_calls == 1U, "preset action dispatch");
    require_true(state->last_action == (action_state_4dof_e)5U, "preset action id");
    require_true(!cmd4_manual_pose_active(CMD4_ARM_LEFT), "action clears manual pose");

    len = build_action_frame(0U, frame);
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->action_abort_calls == 1U, "action abort dispatch");

    len = build_valve_frame(3U, 1U, frame);
    require_true(len == CMD4_FRAME_VALVE_LEN, "valve frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->relay_calls == 5U, "valve dispatch after start valves");
    require_true(state->last_relay_id == 3U && state->last_relay_state == 1U,
                 "valve relay state");

    len = build_answer_frame(2U, frame);
    require_true(len == CMD4_FRAME_ANSWER_LEN, "answer frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->answer_calls == 1U && state->last_answer == 2U,
                 "answer dispatch");

    len = build_pump_frame(1U, 3200.0f, frame);
    require_true(len == CMD4_FRAME_PUMP_LEN, "pump frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(g_pump.is_running, "pump running");
    require_near(g_pump.target_speed_rpm, 3200.0f, "pump target speed");
    require_true(state->pump_speed_calls == 1U, "pump speed dispatch");
    require_near(state->last_pump_speed, 3200.0f, "pump speed output");

    len = build_pump_frame(0U, 1234.0f, frame);
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(!g_pump.is_running, "pump stopped");
    require_near(g_pump.target_speed_rpm, 0.0f, "pump stop target");
    require_near(state->last_pump_speed, 0.0f, "pump stop output");

    cmd4_host_mocks_reset();
    len = build_single_target_frame(CMD4_PICK_BLOCK,
                                    CMD4_ARM_RIGHT,
                                    0.11f, 0.22f, -0.33f,
                                    frame);
    require_true(len == CMD4_FRAME_SINGLE_TARGET_ACTION_LEN,
                 "single pick frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->single_pick_calls == 1U, "single pick dispatch");
    require_true(state->arm_start_calls == 1U, "single pick auto-starts arm");
    assert_all_valves_open(state, "single pick auto-start opens all valves");
    require_true(state->last_arm_id == DOF4_ARM_RIGHT, "single pick arm");
    require_near(state->last_single_target.z, -0.33f, "single pick z");

    len = build_single_target_frame(CMD4_PLACE_BLOCK,
                                    CMD4_ARM_LEFT,
                                    0.44f, -0.55f, -0.66f,
                                    frame);
    require_true(len == CMD4_FRAME_SINGLE_TARGET_ACTION_LEN,
                 "single place frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->single_place_calls == 1U, "single place dispatch");
    require_true(state->last_arm_id == DOF4_ARM_LEFT, "single place arm");
    require_near(state->last_single_target.y, -0.55f, "single place y");

    len = build_single_back_frame(CMD4_PUT_BLOCK_BACK, CMD4_ARM_LEFT, frame);
    require_true(len == CMD4_FRAME_SINGLE_BACK_ACTION_LEN,
                 "single put-back frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->single_put_back_calls == 1U, "single put-back dispatch");

    len = build_single_back_frame(CMD4_GET_BLOCK_BACK, CMD4_ARM_RIGHT, frame);
    require_true(len == CMD4_FRAME_SINGLE_BACK_ACTION_LEN,
                 "single get-back frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->single_get_back_calls == 1U, "single get-back dispatch");
    require_true(state->last_arm_id == DOF4_ARM_RIGHT, "single get-back arm");

    len = build_dual_target_frame(CMD4_PICK_BLOCK_ALL,
                                  0.10f, 0.20f, -0.30f,
                                  0.40f, -0.50f, -0.60f,
                                  frame);
    require_true(len == CMD4_FRAME_DUAL_TARGET_ACTION_LEN,
                 "dual pick frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->dual_pick_calls == 1U, "dual pick dispatch");
    require_near(state->last_left_target.x, 0.10f, "dual pick left x");
    require_near(state->last_right_target.y, -0.50f, "dual pick right y");

    len = build_dual_back_frame(CMD4_PUT_BLOCK_BACK_ALL, frame);
    require_true(len == CMD4_FRAME_DUAL_BACK_ACTION_LEN,
                 "dual put-back frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->dual_put_back_calls == 1U, "dual put-back dispatch");

    len = build_dual_target_frame(CMD4_PLACE_BLOCK_ALL,
                                  0.11f, 0.22f, -0.33f,
                                  0.44f, -0.55f, -0.66f,
                                  frame);
    require_true(len == CMD4_FRAME_DUAL_TARGET_ACTION_LEN,
                 "dual place frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->dual_place_calls == 1U, "dual place dispatch");
    require_near(state->last_left_target.z, -0.33f, "dual place left z");
    require_near(state->last_right_target.x, 0.44f, "dual place right x");

    len = build_dual_back_frame(CMD4_GET_BLOCK_BACK_ALL, frame);
    require_true(len == CMD4_FRAME_DUAL_BACK_ACTION_LEN,
                 "dual get-back frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->dual_get_back_calls == 1U, "dual get-back dispatch");

    cmd4_host_mocks_reset();
    len = build_arm_start_frame(10.0f, -20.0f, 30.0f, frame);
    require_true(len == CMD4_FRAME_ARM_START_LEN, "arm start frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    require_true(state->world_offset_calls == 1U, "world offset dispatch");
    require_true(state->arm_start_calls == 1U, "arm start dispatch");
    assert_all_valves_open(state, "arm start opens all valves");
    require_near(state->last_offset_x, 0.010f, "offset x mm to m");
    require_near(state->last_offset_y, -0.020f, "offset y mm to m");
    require_near(state->last_offset_z, 0.030f, "offset z mm to m");

    cmd4_host_mocks_reset();
    len = build_single_target_frame(CMD4_PICK_BLOCK,
                                    3U,
                                    0.10f, 0.20f, 0.30f,
                                    frame);
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    assert_no_pc_action_dispatch(state, "invalid arm id rejected");
    require_true(state->reject_calls == 1U, "invalid arm records reject");
    require_true(state->last_reject_reason == PC_ACTION_4DOF_REJECT_INVALID_ARM,
                 "invalid arm reject reason");
    require_true(g_dof4_arm_left.clip_diagnostic.pending,
                 "invalid arm diagnostic pending");
    require_true(g_dof4_arm_left.clip_diagnostic.reason ==
                     DOF4_CLIP_REASON_PC_ACTION_REJECT,
                 "invalid arm diagnostic reason");
    require_true(g_dof4_arm_left.clip_diagnostic.joint_mask ==
                     PC_ACTION_4DOF_REJECT_INVALID_ARM,
                 "invalid arm diagnostic code");

    cmd4_host_mocks_reset();
    len = build_dual_back_frame(CMD4_GET_BLOCK_BACK_ALL, frame);
    frame[len - 1U] ^= 0x5AU;
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    assert_no_pc_action_dispatch(state, "bad CRC rejected");

    frame[0] = CMD4_FRAME_HEADER_BYTE;
    frame[1] = CMD4_PUT_BLOCK_BACK_ALL;
    frame[2] = 0x00U;
    len = finish_frame(frame, 3U);
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    assert_no_pc_action_dispatch(state, "wrong frame length rejected");

    len = build_legacy_target_frame(frame);
    require_true(len == 19U, "legacy 0x07 frame length");
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    assert_no_pc_action_dispatch(state, "removed 0x07 rejected");

    frame[0] = CMD4_FRAME_HEADER_BYTE;
    frame[1] = 0x55U;
    len = finish_frame(frame, 2U);
    cmd4_rx_feed(frame, len);
    state = cmd4_host_mocks_state();
    assert_no_pc_action_dispatch(state, "unknown command rejected");

    printf("cmd4 action dispatch test passed\n");
    return 0;
}
