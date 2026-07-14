#include "cmd4_host_mocks.h"

#include "command_decode_4dof.h"
#include "Dof4_Arm_Calibration.h"
#include "gimbal.h"
#include "stm32f4xx_hal.h"
#include "usart_interface.h"
#include "virtual_serial_port.h"

#include <string.h>

Dof4_Arm g_dof4_arm_left;
Dof4_Arm g_dof4_arm_right;
bool g_dof4_arm_started;
PumpCtrl g_pump;
UART_HandleTypeDef huart1;
Gimbal_s Gimbal;

static Cmd4HostMockState s_state;
static bool s_action_active;
static bool s_pc_action_active;
static const Dof4ArmCalibration s_arm_calibration = {
    .target_bias = {
        {0.01f, -0.02f, 0.03f},
        {-0.04f, 0.05f, -0.06f},
    },
};

void cmd4_host_mocks_reset(void)
{
    memset(&s_state, 0, sizeof(s_state));
    memset(&g_dof4_arm_left, 0, sizeof(g_dof4_arm_left));
    memset(&g_dof4_arm_right, 0, sizeof(g_dof4_arm_right));
    memset(&g_pump, 0, sizeof(g_pump));
    g_dof4_arm_started = false;
    s_action_active = false;
    s_pc_action_active = false;
    cmd4_clear_manual_pose();
    (void)cmd4_startup_request_take();
}

const Cmd4HostMockState *cmd4_host_mocks_state(void)
{
    return &s_state;
}

const Dof4ArmCalibration *Dof4_calibration_get_arm(void)
{
    return &s_arm_calibration;
}

void cmd4_host_mocks_set_action_active(bool active)
{
    s_action_active = active;
}

void cmd4_host_mocks_set_pc_action_active(bool active)
{
    s_pc_action_active = active;
}

bool pc_action_4dof_start_pick(Dof4_ArmId arm_id, const Dof4_Pose *target_world)
{
    ++s_state.single_pick_calls;
    s_state.last_arm_id = arm_id;
    s_state.last_single_target = *target_world;
    return true;
}

bool pc_action_4dof_start_place(Dof4_ArmId arm_id, const Dof4_Pose *target_world)
{
    ++s_state.single_place_calls;
    s_state.last_arm_id = arm_id;
    s_state.last_single_target = *target_world;
    return true;
}

bool pc_action_4dof_start_put_back(Dof4_ArmId arm_id)
{
    ++s_state.single_put_back_calls;
    s_state.last_arm_id = arm_id;
    return true;
}

bool pc_action_4dof_start_get_back(Dof4_ArmId arm_id)
{
    ++s_state.single_get_back_calls;
    s_state.last_arm_id = arm_id;
    return true;
}

bool pc_action_4dof_start_dual_pick(const Dof4_Pose *left_target_world,
                                    const Dof4_Pose *right_target_world)
{
    ++s_state.dual_pick_calls;
    s_state.last_left_target = *left_target_world;
    s_state.last_right_target = *right_target_world;
    return true;
}

bool pc_action_4dof_start_dual_place(const Dof4_Pose *left_target_world,
                                     const Dof4_Pose *right_target_world)
{
    ++s_state.dual_place_calls;
    s_state.last_left_target = *left_target_world;
    s_state.last_right_target = *right_target_world;
    return true;
}

bool pc_action_4dof_start_dual_put_back(void)
{
    ++s_state.dual_put_back_calls;
    return true;
}

bool pc_action_4dof_start_dual_get_back(void)
{
    ++s_state.dual_get_back_calls;
    return true;
}

bool pc_action_4dof_is_active(void)
{
    return s_pc_action_active;
}

bool pc_action_4dof_completion_pending(void)
{
    return false;
}

void pc_action_4dof_completion_acknowledge(void)
{
}

void pc_action_4dof_abort(void)
{
    ++s_state.pc_action_abort_calls;
    s_pc_action_active = false;
}

void pc_action_4dof_record_reject(Dof4_ArmId arm_id,
                                  PcAction4DOF_RejectReason reason,
                                  const Dof4_Pose *target_world)
{
    Dof4_Arm *arm = (arm_id == DOF4_ARM_RIGHT)
        ? &g_dof4_arm_right
        : &g_dof4_arm_left;

    ++s_state.reject_calls;
    s_state.last_arm_id = arm_id;
    s_state.last_reject_reason = reason;
    arm->clip_diagnostic.pending = true;
    arm->clip_diagnostic.reason = DOF4_CLIP_REASON_PC_ACTION_REJECT;
    arm->clip_diagnostic.joint_mask = (uint8_t)reason;
    if (target_world != NULL) {
        arm->clip_diagnostic.requested_pose = *target_world;
    }
}

bool action_4dof_is_active(void)
{
    return s_action_active;
}

bool action_4dof_trigger(action_state_4dof_e action)
{
    ++s_state.action_trigger_calls;
    s_state.last_action = action;
    return true;
}

void action_4dof_abort(void)
{
    ++s_state.action_abort_calls;
    s_action_active = false;
}

Dof4_Status Dof4_arm_set_target(Dof4_Arm *arm, float x, float y, float z, float pitch)
{
    ++s_state.pose_set_calls;
    arm->current_pose = (Dof4_Pose){x, y, z, pitch};
    s_state.last_pose_target = arm->current_pose;
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_set_world_offset(float dx, float dy, float dz)
{
    ++s_state.world_offset_calls;
    s_state.last_offset_x = dx;
    s_state.last_offset_y = dy;
    s_state.last_offset_z = dz;
    return DOF4_STATUS_OK;
}

void Dof4_double_arm_start(void)
{
    ++s_state.arm_start_calls;
    g_dof4_arm_started = true;
    relay_control(0U, 1U);
    relay_control(1U, 1U);
    relay_control(2U, 1U);
    relay_control(3U, 1U);
}

void pump_speed_set(float target_speed)
{
    ++s_state.pump_speed_calls;
    s_state.last_pump_speed = target_speed;
}

void relay_control(uint8_t relay_id, uint8_t state)
{
    ++s_state.relay_calls;
    s_state.last_relay_id = relay_id;
    s_state.last_relay_state = state;
    if (relay_id < 4U) {
        s_state.relay_state[relay_id] = (uint8_t)(state & 0x01U);
    }
}

uint8_t vcp_is_connected(void)
{
    return 0U;
}

uint8_t vcp_transmit(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
    return 0U;
}

bool vcp_rx_read_byte(uint8_t *byte)
{
    (void)byte;
    return false;
}

int xiao_R_usart_send_answer(UART_HandleTypeDef *huart, uint8_t answer)
{
    (void)huart;
    ++s_state.answer_calls;
    s_state.last_answer = answer;
    return 0;
}

void led_indicate_answer_notify(uint8_t answer)
{
    ++s_state.led_answer_calls;
    s_state.last_led_answer = answer;
}

uint32_t HAL_GetTick(void)
{
    return 0U;
}

void gimbal_init(void)
{
}

void gimbal_start(void)
{
}

void gimbal_set_target_position(Gimbal_s *gimbal,
                                float j1,
                                float pitch,
                                float yaw)
{
    if (gimbal != NULL) {
        gimbal->j1 = j1;
        gimbal->pitch = pitch;
        gimbal->yaw = yaw;
    }
}
