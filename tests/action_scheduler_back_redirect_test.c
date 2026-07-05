#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "action_scheduler_4dof.h"
#include "pc_action_executor_4dof.h"
#include "pneumatic_control.h"

Dof4_Arm g_dof4_arm_left;
Dof4_Arm g_dof4_arm_right;
bool g_dof4_arm_started;

static bool s_pc_active;
static bool s_pc_accept = true;
static unsigned s_dual_put_calls;
static unsigned s_left_put_calls;
static unsigned s_right_put_calls;
static unsigned s_left_get_calls;
static unsigned s_right_get_calls;

static void require_true(bool value, const char *message)
{
    if (!value) {
        printf("FAIL: %s\n", message);
        exit(1);
    }
}

static void reset_state(void)
{
    memset(&g_dof4_arm_left, 0, sizeof(g_dof4_arm_left));
    memset(&g_dof4_arm_right, 0, sizeof(g_dof4_arm_right));
    g_dof4_arm_left.cfg.base[0] = 0.10f;
    g_dof4_arm_left.cfg.base[1] = 0.13f;
    g_dof4_arm_right.cfg.base[0] = 0.10f;
    g_dof4_arm_right.cfg.base[1] = -0.13f;
    g_dof4_arm_left.cfg.cart_vel_mps = 1.0f;
    g_dof4_arm_right.cfg.cart_vel_mps = 1.0f;
    g_dof4_arm_left.cfg.pitch_vel_rps = 1.0f;
    g_dof4_arm_right.cfg.pitch_vel_rps = 1.0f;
    s_pc_active = false;
    s_pc_accept = true;
    s_dual_put_calls = 0U;
    s_left_put_calls = 0U;
    s_right_put_calls = 0U;
    s_left_get_calls = 0U;
    s_right_get_calls = 0U;
    action_4dof_abort();
}

bool pc_action_4dof_is_active(void)
{
    return s_pc_active;
}

bool pc_action_4dof_start_put_back(Dof4_ArmId arm_id)
{
    if (arm_id == DOF4_ARM_LEFT) {
        ++s_left_put_calls;
    } else if (arm_id == DOF4_ARM_RIGHT) {
        ++s_right_put_calls;
    }
    return s_pc_accept;
}

bool pc_action_4dof_start_get_back(Dof4_ArmId arm_id)
{
    if (arm_id == DOF4_ARM_LEFT) {
        ++s_left_get_calls;
    } else if (arm_id == DOF4_ARM_RIGHT) {
        ++s_right_get_calls;
    }
    return s_pc_accept;
}

bool pc_action_4dof_start_dual_put_back(void)
{
    ++s_dual_put_calls;
    return s_pc_accept;
}

bool pc_action_4dof_start_dual_get_back(void)
{
    return false;
}

Dof4_Status Dof4_arm_set_target(Dof4_Arm *arm,
                                float x,
                                float y,
                                float z,
                                float pitch)
{
    if (arm == NULL) {
        return DOF4_STATUS_IK_UNREACHABLE;
    }
    arm->target_pose = (Dof4_Pose){x, y, z, pitch};
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_arm_set_target_via(Dof4_Arm *arm,
                                    float x,
                                    float y,
                                    float z,
                                    float pitch,
                                    const float via_vel[4])
{
    (void)via_vel;
    return Dof4_arm_set_target(arm, x, y, z, pitch);
}

Dof4_Status Dof4_arm_set_joint_target(Dof4_Arm *arm,
                                      const Dof4_JointState *joints)
{
    if (arm == NULL || joints == NULL) {
        return DOF4_STATUS_IK_UNREACHABLE;
    }
    arm->joint_target = *joints;
    arm->joint_actual = *joints;
    return DOF4_STATUS_OK;
}

float Dof4_normalize_angle(float angle_rad)
{
    return angle_rad;
}

void Dof4_compute_via_velocity(const Dof4_Pose *from,
                               const Dof4_Pose *to,
                               float speed_factor,
                               float cart_vel_mps,
                               float pitch_vel_rps,
                               float via_vel[4])
{
    (void)from;
    (void)to;
    (void)speed_factor;
    (void)cart_vel_mps;
    (void)pitch_vel_rps;
    if (via_vel != NULL) {
        memset(via_vel, 0, 4U * sizeof(via_vel[0]));
    }
}

void relay_control(uint8_t relay_id, uint8_t state)
{
    (void)relay_id;
    (void)state;
}

void pump_speed_set(float target_speed)
{
    (void)target_speed;
}

uint32_t HAL_GetTick(void)
{
    return 0U;
}

int main(void)
{
    reset_state();
    require_true(action_4dof_trigger(ACTION_BLOCK_PLACE_BACK),
                 "dual back place redirects to PC action");
    require_true(s_dual_put_calls == 1U, "dual put-back PC action called");
    require_true(!action_4dof_is_active(),
                 "scheduler stays inactive for dual back place");

    reset_state();
    require_true(action_4dof_trigger(ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_BACK),
                 "left back place redirects to PC action");
    require_true(s_left_put_calls == 1U, "left put-back PC action called");
    require_true(!action_4dof_is_active(),
                 "scheduler stays inactive for left back place");

    reset_state();
    require_true(action_4dof_trigger(ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_BACK),
                 "right back place redirects to PC action");
    require_true(s_right_put_calls == 1U, "right put-back PC action called");
    require_true(!action_4dof_is_active(),
                 "scheduler stays inactive for right back place");

    reset_state();
    require_true(action_4dof_trigger(ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_LEFT_ARM),
                 "left back get redirects to PC action");
    require_true(s_left_get_calls == 1U, "left get-back PC action called");
    require_true(!action_4dof_is_active(),
                 "scheduler stays inactive for left back get");

    reset_state();
    require_true(action_4dof_trigger(ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_RIGHT_ARM),
                 "right back get redirects to PC action");
    require_true(s_right_get_calls == 1U, "right get-back PC action called");
    require_true(!action_4dof_is_active(),
                 "scheduler stays inactive for right back get");

    reset_state();
    require_true(action_4dof_trigger(ACTION_BLOCK_GET_FORWARD_LEFT_ARM),
                 "non-back action still uses scheduler");
    require_true(action_4dof_is_active(),
                 "scheduler becomes active for non-back action");
    require_true(s_dual_put_calls == 0U &&
                 s_left_put_calls == 0U &&
                 s_right_put_calls == 0U &&
                 s_left_get_calls == 0U &&
                 s_right_get_calls == 0U,
                 "non-back action does not call PC back actions");

    printf("action scheduler back redirect test passed\n");
    return 0;
}
