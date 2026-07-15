#include "pc_action_executor_4dof.h"

#include "action_scheduler_4dof.h"
#include "pneumatic_control.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Dof4_Arm g_dof4_arm_left;
Dof4_Arm g_dof4_arm_right;
bool g_dof4_arm_started;
volatile uint32_t g_pc_action_error_event_count;

static uint32_t s_tick_ms;
static unsigned s_relay_on_calls;
static unsigned s_relay_calls;
static unsigned s_set_target_calls;
static unsigned s_set_joint_calls;
static bool s_action_active;
static uint8_t s_valve_shadow[4];
static bool s_back_occupied[2];
static bool s_hold_pose_feedback;
static bool s_hold_joint_feedback;
static bool s_limit_exact_path_y;
static float s_exact_path_y_limit;
static float s_projection_y_adjust;
static unsigned s_resolve_path_calls;
static float s_last_resolve_max_adjust_m;
static Dof4_Pose s_last_resolve_requested;

typedef struct {
    Dof4_ArmId arm_id;
    Dof4_Pose pose;
} TargetCall;

typedef struct {
    uint8_t relay_id;
    uint8_t state;
} RelayCall;

typedef struct {
    Dof4_ArmId arm_id;
    Dof4_JointState joints;
} JointCall;

static TargetCall s_target_calls[64];
static RelayCall s_relay_history[64];
static JointCall s_joint_calls[64];

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

static void init_arm(Dof4_Arm *arm)
{
    memset(arm, 0, sizeof(*arm));
    arm->current_pose = (Dof4_Pose){0.0f, 0.0f, 0.0f, -1.50f};
    arm->cfg.servo_speed = 3000U;
    arm->cfg.servo_acc = 25U;
    for (uint8_t i = 0U; i < DOF4_JOINT_COUNT; ++i) {
        arm->cfg.joint_min[i] = -10.0f;
        arm->cfg.joint_max[i] = 10.0f;
    }
}

static void reset_test_state(void)
{
    init_arm(&g_dof4_arm_left);
    init_arm(&g_dof4_arm_right);
    g_dof4_arm_started = true;
    g_pc_action_error_event_count = 0U;
    s_tick_ms = 0U;
    s_relay_on_calls = 0U;
    s_relay_calls = 0U;
    s_set_target_calls = 0U;
    s_set_joint_calls = 0U;
    memset(s_valve_shadow, 0, sizeof(s_valve_shadow));
    memset(s_back_occupied, 0, sizeof(s_back_occupied));
    memset(s_target_calls, 0, sizeof(s_target_calls));
    memset(s_relay_history, 0, sizeof(s_relay_history));
    memset(s_joint_calls, 0, sizeof(s_joint_calls));
    s_hold_pose_feedback = false;
    s_hold_joint_feedback = false;
    s_limit_exact_path_y = false;
    s_exact_path_y_limit = 0.80f;
    s_projection_y_adjust = 0.0f;
    s_resolve_path_calls = 0U;
    s_last_resolve_max_adjust_m = 0.0f;
    memset(&s_last_resolve_requested, 0, sizeof(s_last_resolve_requested));
    s_action_active = false;
    pc_action_4dof_init();
}

bool action_4dof_is_active(void)
{
    return s_action_active;
}

bool action_4dof_trigger(action_state_4dof_e action)
{
    (void)action;
    return true;
}

void action_4dof_abort(void)
{
}

Dof4_Pose action_4dof_get_idle_pose(Dof4_ArmId arm_id)
{
    (void)arm_id;
    return (Dof4_Pose){0.0f, 0.0f, 0.0f, -1.50f};
}

void action_4dof_set_back_occupied(Dof4_ArmId arm_id, bool occupied)
{
    s_back_occupied[(uint8_t)arm_id] = occupied;
}

void cmd4_update_valve_shadow(uint8_t valve_id, uint8_t state)
{
    if (valve_id < (sizeof(s_valve_shadow) / sizeof(s_valve_shadow[0]))) {
        s_valve_shadow[valve_id] = state;
    }
}

void relay_control(uint8_t relay_id, uint8_t state)
{
    if (s_relay_calls < (sizeof(s_relay_history) / sizeof(s_relay_history[0]))) {
        s_relay_history[s_relay_calls] = (RelayCall){relay_id, state};
    }
    ++s_relay_calls;
    if (state != 0U) {
        ++s_relay_on_calls;
    }
}

Dof4_Status Dof4_arm_set_target(Dof4_Arm *arm,
                                float x,
                                float y,
                                float z,
                                float pitch)
{
    if (s_set_target_calls < (sizeof(s_target_calls) / sizeof(s_target_calls[0]))) {
        s_target_calls[s_set_target_calls].arm_id =
            (arm == &g_dof4_arm_right) ? DOF4_ARM_RIGHT : DOF4_ARM_LEFT;
        s_target_calls[s_set_target_calls].pose = (Dof4_Pose){x, y, z, pitch};
    }
    ++s_set_target_calls;
    arm->target_pose = (Dof4_Pose){x, y, z, pitch};
    if (!s_hold_pose_feedback) {
        arm->current_pose = arm->target_pose;
    }
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_arm_set_joint_target(Dof4_Arm *arm,
                                      const Dof4_JointState *joints)
{
    if (s_set_joint_calls < (sizeof(s_joint_calls) / sizeof(s_joint_calls[0]))) {
        s_joint_calls[s_set_joint_calls].arm_id =
            (arm == &g_dof4_arm_right) ? DOF4_ARM_RIGHT : DOF4_ARM_LEFT;
        s_joint_calls[s_set_joint_calls].joints = *joints;
    }
    ++s_set_joint_calls;
    arm->joint_target = *joints;
    if (!s_hold_joint_feedback) {
        arm->joint_actual = *joints;
    }
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_arm_inverse_kinematics(Dof4_Arm *arm,
                                        const Dof4_Pose *target,
                                        float elbow_sign,
                                        Dof4_JointState *joints)
{
    (void)arm;
    (void)elbow_sign;
    if (target == NULL || joints == NULL ||
        !isfinite(target->x) || !isfinite(target->y) ||
        !isfinite(target->z) || !isfinite(target->pitch) ||
        fabsf(target->x) > 0.80f || fabsf(target->y) > 0.80f ||
        (s_limit_exact_path_y && fabsf(target->y) > s_exact_path_y_limit) ||
        target->z < -0.60f || target->z > 0.60f) {
        return DOF4_STATUS_IK_UNREACHABLE;
    }
    memset(joints, 0, sizeof(*joints));
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_arm_resolve_reachable_pose(Dof4_Arm *arm,
                                            const Dof4_Pose *requested,
                                            float elbow_sign,
                                            float max_adjust_m,
                                            Dof4_Pose *resolved)
{
    ++s_resolve_path_calls;
    s_last_resolve_max_adjust_m = max_adjust_m;
    s_last_resolve_requested = *requested;

    if (fabsf(s_projection_y_adjust) > max_adjust_m) {
        return DOF4_STATUS_IK_UNREACHABLE;
    }

    *resolved = *requested;
    resolved->y += s_projection_y_adjust;

    Dof4_JointState joints;
    return Dof4_arm_inverse_kinematics(arm, resolved, elbow_sign, &joints);
}

Dof4_Status Dof4_angle_to_servo(const Dof4_Arm *arm,
                                uint8_t joint_index,
                                float angle_rad,
                                int16_t *servo_pos)
{
    (void)arm;
    (void)joint_index;
    *servo_pos = (int16_t)((angle_rad >= 0.0f)
                               ? (angle_rad * 1000.0f + 0.5f)
                               : (angle_rad * 1000.0f - 0.5f));
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_servo_to_angle(const Dof4_Arm *arm,
                                uint8_t joint_index,
                                int16_t servo_pos,
                                float *angle_rad)
{
    (void)arm;
    (void)joint_index;
    *angle_rad = (float)servo_pos / 1000.0f;
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_clamp_to_workspace(const Dof4_Arm *arm, Dof4_Pose *pose)
{
    (void)arm;
    if (pose->z > 0.60f) {
        pose->z = 0.60f;
    } else if (pose->z < -0.60f) {
        pose->z = -0.60f;
    }
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_arm_set_base_offset(Dof4_Arm *arm, float dx, float dy, float dz)
{
    if (arm != NULL) {
        arm->cfg.base_offset[0] = dx;
        arm->cfg.base_offset[1] = dy;
        arm->cfg.base_offset[2] = dz;
    }
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_arm_set_tcp_offset(Dof4_Arm *arm, float dx, float dy, float dz)
{
    if (arm != NULL) {
        arm->cfg.tcp_offset[0] = dx;
        arm->cfg.tcp_offset[1] = dy;
        arm->cfg.tcp_offset[2] = dz;
    }
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_arm_set_pitch_offset(Dof4_Arm *arm, float pitch_offset)
{
    if (arm != NULL) {
        arm->cfg.pitch_offset = pitch_offset;
    }
    return DOF4_STATUS_OK;
}

Dof4_Status Dof4_set_world_offset(float dx, float dy, float dz)
{
    (void)dx;
    (void)dy;
    (void)dz;
    return DOF4_STATUS_OK;
}

float Dof4_normalize_angle(float angle_rad)
{
    return angle_rad;
}

void Dof4_double_arm_start(void)
{
    g_dof4_arm_started = true;
}

uint32_t HAL_GetTick(void)
{
    return s_tick_ms;
}

static bool relay_seen_after(unsigned start_index, uint8_t relay_id, uint8_t state)
{
    for (unsigned i = start_index; i < s_relay_calls &&
                              i < (sizeof(s_relay_history) / sizeof(s_relay_history[0])); ++i) {
        if (s_relay_history[i].relay_id == relay_id &&
            s_relay_history[i].state == state) {
            return true;
        }
    }
    return false;
}

static bool relay_seen_range(unsigned start_index,
                             unsigned end_index,
                             uint8_t relay_id,
                             uint8_t state)
{
    if (end_index > s_relay_calls) {
        end_index = s_relay_calls;
    }
    for (unsigned i = start_index; i < end_index &&
                              i < (sizeof(s_relay_history) / sizeof(s_relay_history[0])); ++i) {
        if (s_relay_history[i].relay_id == relay_id &&
            s_relay_history[i].state == state) {
            return true;
        }
    }
    return false;
}

static bool target_pose_seen(float x, float z)
{
    for (unsigned i = 0U; i < s_set_target_calls &&
                         i < (sizeof(s_target_calls) / sizeof(s_target_calls[0])); ++i) {
        if (fabsf(s_target_calls[i].pose.x - x) <= 1.0e-6f &&
            fabsf(s_target_calls[i].pose.z - z) <= 1.0e-6f) {
            return true;
        }
    }
    return false;
}

static bool arm_target_pose_seen(Dof4_ArmId arm_id,
                                 float x,
                                 float y,
                                 float z)
{
    for (unsigned i = 0U; i < s_set_target_calls &&
                         i < (sizeof(s_target_calls) / sizeof(s_target_calls[0])); ++i) {
        if (s_target_calls[i].arm_id == arm_id &&
            fabsf(s_target_calls[i].pose.x - x) <= 1.0e-6f &&
            fabsf(s_target_calls[i].pose.y - y) <= 1.0e-6f &&
            fabsf(s_target_calls[i].pose.z - z) <= 1.0e-6f) {
            return true;
        }
    }
    return false;
}

static void advance_dynamic_to_target(void)
{
    pc_action_4dof_loop();
    require_true(s_set_target_calls > 0U, "dynamic action drives hover point");
    s_tick_ms = 1000U;
    for (uint8_t i = 0U; i < 6U && !target_pose_seen(0.20f, 0.30f); ++i) {
        pc_action_4dof_loop();
    }
    require_true(target_pose_seen(0.20f, 0.30f), "dynamic action drives target point");
}

static void advance_dynamic_target_settle(void)
{
    s_tick_ms += 800U;
    pc_action_4dof_loop();
}

static void finish_dynamic_after_operation(uint32_t hold_ms)
{
    s_tick_ms += hold_ms;
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    s_tick_ms += 100U;
    pc_action_4dof_loop();
}

static void advance_joint_to_operation_target(void)
{
    for (uint8_t i = 0U; i < 10U; ++i) {
        pc_action_4dof_loop();
    }
}

static void require_joint_near(const Dof4_JointState *actual,
                               const float expected[DOF4_JOINT_COUNT],
                               const char *message)
{
    for (uint8_t i = 0U; i < DOF4_JOINT_COUNT; ++i) {
        if (fabsf(actual->q[i] - expected[i]) > 1.0e-6f) {
            printf("FAIL: %s joint%u expected=%.6f actual=%.6f\n",
                   message,
                   (unsigned)i,
                   (double)expected[i],
                   (double)actual->q[i]);
            exit(1);
        }
    }
}

static void finish_joint_action_from_retreat(void)
{
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    pc_action_4dof_loop();
}

int main(void)
{
    Dof4_Pose target = {0.20f, 0.10f, 0.30f, 0.0f};
    Dof4_Pose left_target = {0.20f, 0.10f, 0.30f, 0.0f};
    Dof4_Pose right_target = {0.30f, -0.10f, 0.20f, 0.0f};

    reset_test_state();
    require_true(pc_action_4dof_start_pick(DOF4_ARM_LEFT, &target),
                 "pick request is queued");
    require_true(pc_action_4dof_is_active(), "pending request marks PC active");
    require_true(s_set_target_calls == 0U, "start does not build path immediately");
    pc_action_4dof_loop();
    require_true(s_relay_on_calls > 0U, "queued request starts suction in loop");
    require_true(s_set_target_calls == 1U, "first loop drives P1");
    require_near(s_target_calls[0].pose.x, 0.20f, "hover target x");
    require_near(s_target_calls[0].pose.y, 0.10f, "hover target y");
    require_near(s_target_calls[0].pose.z, 0.42f, "hover target z plus clearance");
    s_tick_ms = 1000U;
    for (uint8_t i = 0U; i < 6U && !target_pose_seen(0.20f, 0.30f); ++i) {
        pc_action_4dof_loop();
    }
    require_true(target_pose_seen(0.20f, 0.30f), "target driven after hover point");
    require_true(!g_dof4_arm_left.clip_diagnostic.pending,
                 "dynamic two-point path accepted");
    advance_dynamic_target_settle();
    pc_action_4dof_loop();
    finish_dynamic_after_operation(1500U);
    require_true(arm_target_pose_seen(DOF4_ARM_LEFT, 0.20f, 0.15f, 0.42f),
                 "single left pick retreats 5 cm toward +Y");
    require_true(!relay_seen_after(0U, 0U, 0U),
                 "dynamic pick never closes working arm valve");
    require_true(!relay_seen_after(0U, 2U, 0U) &&
                 !relay_seen_after(0U, 2U, 1U),
                 "dynamic pick does not touch back valve");

    reset_test_state();
    left_target = (Dof4_Pose){0.20f, 0.10f, 0.30f, 0.0f};
    right_target = (Dof4_Pose){0.30f, -0.10f, 0.20f, 0.0f};
    require_true(pc_action_4dof_start_dual_pick(&left_target, &right_target),
                 "dual pick request is queued");
    advance_dynamic_to_target();
    advance_dynamic_target_settle();
    pc_action_4dof_loop();
    finish_dynamic_after_operation(1500U);
    require_true(arm_target_pose_seen(DOF4_ARM_LEFT, 0.20f, 0.15f, 0.42f),
                 "dual pick left arm retreats toward +Y");
    require_true(arm_target_pose_seen(DOF4_ARM_RIGHT, 0.30f, -0.15f, 0.32f),
                 "dual pick right arm retreats toward -Y");
    require_true(!relay_seen_after(0U, 0U, 0U) &&
                 !relay_seen_after(0U, 1U, 0U),
                 "dual dynamic pick never closes working arm valves");
    require_true(!relay_seen_after(0U, 2U, 0U) &&
                 !relay_seen_after(0U, 2U, 1U) &&
                 !relay_seen_after(0U, 3U, 0U) &&
                 !relay_seen_after(0U, 3U, 1U),
                 "dual dynamic pick does not touch back valves");

    reset_test_state();
    target = (Dof4_Pose){0.30f, -0.10f, 0.20f, 0.0f};
    require_true(pc_action_4dof_start_pick(DOF4_ARM_RIGHT, &target),
                 "single right pick request is queued");
    pc_action_4dof_loop();
    s_tick_ms = 1000U;
    for (uint8_t i = 0U; i < 6U && !target_pose_seen(0.30f, 0.20f); ++i) {
        pc_action_4dof_loop();
    }
    require_true(target_pose_seen(0.30f, 0.20f),
                 "single right pick reaches target");
    advance_dynamic_target_settle();
    pc_action_4dof_loop();
    finish_dynamic_after_operation(1500U);
    require_true(arm_target_pose_seen(DOF4_ARM_RIGHT, 0.30f, -0.15f, 0.32f),
                 "single right pick retreats 5 cm toward -Y");

    reset_test_state();
    target = (Dof4_Pose){0.20f, 0.10f, 0.58f, 0.0f};
    require_true(pc_action_4dof_start_pick(DOF4_ARM_LEFT, &target),
                 "clamped P2 request accepted");
    pc_action_4dof_loop();
    s_tick_ms = 1000U;
    pc_action_4dof_loop();
    require_true(s_set_target_calls >= 1U, "clamped hover driven");
    require_near(s_target_calls[0].pose.z, 0.60f, "hover z clamps to workspace");

    reset_test_state();
    target = (Dof4_Pose){0.20f, 0.10f, 0.30f, 0.0f};
    require_true(pc_action_4dof_start_place(DOF4_ARM_LEFT, &target),
                 "place request is queued");
    advance_dynamic_to_target();
    advance_dynamic_target_settle();
    const unsigned relay_calls_before_release = s_relay_calls;
    pc_action_4dof_loop();
    require_true(s_relay_calls > relay_calls_before_release,
                 "place operation controls relay");
    require_true(s_relay_history[s_relay_calls - 1U].relay_id == 0U &&
                 s_relay_history[s_relay_calls - 1U].state == 0U,
                 "place target closes left arm suction");
    require_true(!relay_seen_range(relay_calls_before_release, s_relay_calls, 2U, 0U) &&
                 !relay_seen_range(relay_calls_before_release, s_relay_calls, 2U, 1U),
                 "dynamic place does not touch back valve");
    const unsigned relay_calls_after_release = s_relay_calls;
    s_tick_ms += 1999U;
    pc_action_4dof_loop();
    require_true(!relay_seen_after(relay_calls_after_release, 0U, 1U),
                 "place release hold keeps working arm valve closed");
    s_tick_ms += 1U;
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    s_tick_ms += 100U;
    pc_action_4dof_loop();
    require_true(pc_action_4dof_completion_pending(),
                 "finish sets action done pending");
    pc_action_4dof_completion_acknowledge();
    require_true(!pc_action_4dof_completion_pending(),
                 "action done acknowledge clears pending");
    require_true(s_relay_history[s_relay_calls - 1U].relay_id == 0U &&
                 s_relay_history[s_relay_calls - 1U].state == 1U,
                 "finish reopens left arm suction");

    reset_test_state();
    target = (Dof4_Pose){0.20f, 0.10f, 0.30f, 0.0f};
    require_true(pc_action_4dof_start_place(DOF4_ARM_LEFT, &target),
                 "place timeout request is queued");
    pc_action_4dof_loop();
    s_tick_ms += 1600U;
    pc_action_4dof_loop();
    s_hold_pose_feedback = true;
    pc_action_4dof_loop();
    const unsigned timeout_place_before_release = s_relay_calls;
    s_tick_ms += 2500U;
    pc_action_4dof_loop();
    s_tick_ms += 800U;
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    require_true(relay_seen_after(timeout_place_before_release, 0U, 0U),
                 "dynamic place timeout still closes working arm valve");

    reset_test_state();
    target = (Dof4_Pose){0.20f, 0.10f, 0.30f, 0.0f};
    require_true(pc_action_4dof_start_place(DOF4_ARM_LEFT, &target),
                 "place finish helper setup request is queued");
    advance_dynamic_to_target();
    advance_dynamic_target_settle();
    pc_action_4dof_loop();
    finish_dynamic_after_operation(2000U);
    require_true(arm_target_pose_seen(DOF4_ARM_LEFT, 0.20f, 0.15f, 0.42f),
                 "single left place retreats 5 cm toward +Y");
    require_true(s_relay_history[s_relay_calls - 1U].relay_id == 0U &&
                 s_relay_history[s_relay_calls - 1U].state == 1U,
                 "finish reopens left arm suction");

    reset_test_state();
    left_target = (Dof4_Pose){0.20f, 0.10f, 0.30f, 0.0f};
    right_target = (Dof4_Pose){0.30f, -0.10f, 0.20f, 0.0f};
    require_true(pc_action_4dof_start_dual_place(&left_target, &right_target),
                 "dual dynamic request is queued");
    pc_action_4dof_loop();
    require_true(s_set_target_calls == 2U, "dual hover drives both arms");
    require_true(s_target_calls[0].arm_id == DOF4_ARM_LEFT &&
                 s_target_calls[1].arm_id == DOF4_ARM_RIGHT,
                 "dual hover arm order");
    require_near(s_target_calls[0].pose.x, 0.20f, "dual left hover x");
    require_near(s_target_calls[1].pose.x, 0.30f, "dual right hover x");
    s_tick_ms = 1000U;
    for (uint8_t i = 0U; i < 4U && s_set_target_calls < 4U; ++i) {
        pc_action_4dof_loop();
    }
    require_true(s_set_target_calls >= 4U, "dual target drives both arms");
    advance_dynamic_target_settle();
    pc_action_4dof_loop();
    require_true(relay_seen_after(0U, 0U, 0U) &&
                 relay_seen_after(0U, 1U, 0U),
                 "dual dynamic place closes both working arm valves");
    finish_dynamic_after_operation(1600U);
    require_true(arm_target_pose_seen(DOF4_ARM_LEFT, 0.20f, 0.15f, 0.42f),
                 "dual place left arm retreats toward +Y");
    require_true(arm_target_pose_seen(DOF4_ARM_RIGHT, 0.30f, -0.15f, 0.32f),
                 "dual place right arm retreats toward -Y");

    reset_test_state();
    require_true(pc_action_4dof_start_put_back(DOF4_ARM_LEFT),
                 "put-back request is queued");
    require_true(g_dof4_arm_left.cfg.servo_speed == 3000U,
                 "put-back speed unchanged while pending");
    pc_action_4dof_loop();
    require_true(g_dof4_arm_left.cfg.servo_speed == 8000U,
                 "put-back applies dedicated left servo speed");
    require_true(g_dof4_arm_left.cfg.servo_acc == 40U,
                 "put-back applies fast left servo acc");
    require_true(g_dof4_arm_right.cfg.servo_speed == 3000U,
                 "put-back leaves unused right servo speed unchanged");
    require_true(g_dof4_arm_right.cfg.servo_acc == 25U,
                 "put-back leaves unused right servo acc unchanged");
    require_true(s_set_joint_calls > 0U &&
                 s_joint_calls[0].arm_id == DOF4_ARM_LEFT,
                 "put-back first joint target recorded");
    const float put_back_left_wp2[DOF4_JOINT_COUNT] = {
        2.500f, 1.50f, -1.20f, -1.680f
    };
    require_joint_near(&s_joint_calls[0].joints,
                       put_back_left_wp2,
                       "put-back first point is first pre point");
    s_set_joint_calls = 0U;
    advance_joint_to_operation_target();
    require_true(g_dof4_arm_left.cfg.servo_speed == 2800U,
                 "put-back uses final low servo speed at target");
    require_true(g_dof4_arm_left.cfg.servo_acc == 14U,
                 "put-back uses final low servo acc at target");
    pc_action_4dof_loop();
    require_true(s_valve_shadow[2] == 1U &&
                 s_relay_history[s_relay_calls - 1U].relay_id == 2U &&
                 s_relay_history[s_relay_calls - 1U].state == 1U,
                 "put-back opens target back valve at target");
    s_tick_ms += 2000U;
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    require_true(s_valve_shadow[0] == 0U &&
                 s_relay_history[s_relay_calls - 1U].relay_id == 0U &&
                 s_relay_history[s_relay_calls - 1U].state == 0U,
                 "put-back closes working arm after back suction hold");
    require_true(s_back_occupied[DOF4_ARM_LEFT], "put-back marks back occupied");
    require_true(s_valve_shadow[2] == 1U, "put-back leaves target back valve open");
    s_tick_ms += 2000U;
    pc_action_4dof_loop();
    finish_joint_action_from_retreat();
    require_true(g_dof4_arm_left.cfg.servo_speed == 3000U,
                 "put-back restores left servo speed after finish");
    require_true(g_dof4_arm_left.cfg.servo_acc == 25U,
                 "put-back restores left servo acc after finish");
    require_true(g_dof4_arm_right.cfg.servo_speed == 3000U,
                 "put-back keeps right servo speed restored");
    require_true(g_dof4_arm_right.cfg.servo_acc == 25U,
                 "put-back keeps right servo acc restored");

    reset_test_state();
    require_true(pc_action_4dof_start_put_back(DOF4_ARM_LEFT),
                 "put-back timeout request is queued");
    pc_action_4dof_loop();
    s_hold_joint_feedback = true;
    pc_action_4dof_loop();
    s_tick_ms += 2500U;
    pc_action_4dof_loop();
    const unsigned timeout_put_before_back_suction = s_relay_calls;
    pc_action_4dof_loop();
    s_tick_ms += 2500U;
    pc_action_4dof_loop();
    require_true(!relay_seen_after(timeout_put_before_back_suction, 2U, 1U),
                 "put-back target timeout must not open target back valve");
    require_true(!relay_seen_after(timeout_put_before_back_suction, 0U, 0U),
                 "put-back target timeout must not close working arm valve");
    require_true(!s_back_occupied[DOF4_ARM_LEFT],
                 "put-back target timeout must not mark back occupied");
    require_true(g_pc_action_error_event_count > 0U,
                 "put-back target timeout reports error");
    s_hold_pose_feedback = false;
    pc_action_4dof_loop();
    s_tick_ms += 100U;
    pc_action_4dof_loop();
    require_true(g_dof4_arm_left.cfg.servo_speed == 3000U,
                 "put-back timeout restores left servo speed");
    require_true(g_dof4_arm_left.cfg.servo_acc == 25U,
                 "put-back timeout restores left servo acc");

    reset_test_state();
    require_true(pc_action_4dof_start_get_back(DOF4_ARM_LEFT),
                 "get-back request is queued");
    pc_action_4dof_loop();
    require_true(g_dof4_arm_left.cfg.servo_speed == 8000U,
                 "get-back applies dedicated left servo speed");
    require_true(g_dof4_arm_left.cfg.servo_acc == 40U,
                 "get-back applies fast left servo acc");
    require_true(s_set_joint_calls > 0U &&
                 s_joint_calls[0].arm_id == DOF4_ARM_LEFT,
                 "get-back first joint target recorded");
    const float get_back_left_wp2[DOF4_JOINT_COUNT] = {
        2.500f, 1.50f, -1.00f, -1.680f
    };
    require_joint_near(&s_joint_calls[0].joints,
                       get_back_left_wp2,
                       "get-back first point is first pre point");
    s_set_joint_calls = 0U;
    advance_joint_to_operation_target();
    require_true(g_dof4_arm_left.cfg.servo_speed == 2800U,
                 "get-back uses final low servo speed at target");
    require_true(g_dof4_arm_left.cfg.servo_acc == 14U,
                 "get-back uses final low servo acc at target");
    pc_action_4dof_loop();
    require_true(!relay_seen_after(0U, 0U, 0U),
                 "get-back keeps working arm valve open before source release");
    s_tick_ms += 1600U;
    pc_action_4dof_loop();
    require_true(s_valve_shadow[2] == 0U &&
                 s_relay_history[s_relay_calls - 1U].relay_id == 2U &&
                 s_relay_history[s_relay_calls - 1U].state == 0U,
                 "get-back closes source back valve after arm suction hold");
    require_true(!s_back_occupied[DOF4_ARM_LEFT], "get-back marks back empty");
    require_true(!relay_seen_after(0U, 0U, 0U),
                 "get-back never closes working arm valve");

    reset_test_state();
    require_true(pc_action_4dof_start_dual_put_back(),
                 "dual put-back request is queued");
    pc_action_4dof_loop();
    require_true(g_dof4_arm_left.cfg.servo_speed == 8000U &&
                 g_dof4_arm_right.cfg.servo_speed == 8000U,
                 "dual put-back applies dedicated speed to both arms");
    require_true(g_dof4_arm_left.cfg.servo_acc == 40U &&
                 g_dof4_arm_right.cfg.servo_acc == 40U,
                 "dual put-back applies fast acc to both arms");
    require_true(s_set_joint_calls >= 2U &&
                 s_joint_calls[0].arm_id == DOF4_ARM_LEFT &&
                 s_joint_calls[1].arm_id == DOF4_ARM_RIGHT,
                 "dual put-back first joint targets recorded");
    const float put_back_right_wp2[DOF4_JOINT_COUNT] = {
        -2.500f, 1.50f, -1.20f, -1.680f
    };
    require_joint_near(&s_joint_calls[0].joints,
                       put_back_left_wp2,
                       "dual put-back left first point is first pre point");
    require_joint_near(&s_joint_calls[1].joints,
                       put_back_right_wp2,
                       "dual put-back right first point is first pre point");
    s_set_joint_calls = 0U;
    advance_joint_to_operation_target();
    require_true(g_dof4_arm_left.cfg.servo_speed == 2800U &&
                 g_dof4_arm_right.cfg.servo_speed == 2800U,
                 "dual put-back uses final low speed on both arms");
    require_true(g_dof4_arm_left.cfg.servo_acc == 14U &&
                 g_dof4_arm_right.cfg.servo_acc == 14U,
                 "dual put-back uses final low acc on both arms");
    pc_action_4dof_loop();
    require_true(s_valve_shadow[2] == 1U && s_valve_shadow[3] == 1U &&
                 relay_seen_after(0U, 2U, 1U) &&
                 relay_seen_after(0U, 3U, 1U),
                 "dual put-back opens both target back valves");
    s_tick_ms += 2000U;
    pc_action_4dof_loop();
    const unsigned dual_put_before_arm_release = s_relay_calls;
    pc_action_4dof_loop();
    require_true(relay_seen_after(dual_put_before_arm_release, 0U, 0U) &&
                 relay_seen_after(dual_put_before_arm_release, 1U, 0U),
                 "dual put-back closes both working arm valves");
    require_true(s_back_occupied[DOF4_ARM_LEFT] &&
                 s_back_occupied[DOF4_ARM_RIGHT],
                 "dual put-back marks both backs occupied");
    require_true(s_valve_shadow[2] == 1U && s_valve_shadow[3] == 1U,
                 "dual put-back leaves both back valves open");

    reset_test_state();
    require_true(pc_action_4dof_start_dual_get_back(),
                 "dual get-back request is queued");
    pc_action_4dof_loop();
    require_true(g_dof4_arm_left.cfg.servo_speed == 8000U &&
                 g_dof4_arm_right.cfg.servo_speed == 8000U,
                 "dual get-back applies dedicated speed to both arms");
    require_true(g_dof4_arm_left.cfg.servo_acc == 40U &&
                 g_dof4_arm_right.cfg.servo_acc == 40U,
                 "dual get-back applies fast acc to both arms");
    require_true(s_set_joint_calls >= 2U &&
                 s_joint_calls[0].arm_id == DOF4_ARM_LEFT &&
                 s_joint_calls[1].arm_id == DOF4_ARM_RIGHT,
                 "dual get-back first joint targets recorded");
    const float get_back_right_wp2[DOF4_JOINT_COUNT] = {
        -2.500f, 1.50f, -1.00f, -1.680f
    };
    require_joint_near(&s_joint_calls[0].joints,
                       get_back_left_wp2,
                       "dual get-back left first point is first pre point");
    require_joint_near(&s_joint_calls[1].joints,
                       get_back_right_wp2,
                       "dual get-back right first point is first pre point");
    s_set_joint_calls = 0U;
    advance_joint_to_operation_target();
    require_true(g_dof4_arm_left.cfg.servo_speed == 2800U &&
                 g_dof4_arm_right.cfg.servo_speed == 2800U,
                 "dual get-back uses final low speed on both arms");
    require_true(g_dof4_arm_left.cfg.servo_acc == 14U &&
                 g_dof4_arm_right.cfg.servo_acc == 14U,
                 "dual get-back uses final low acc on both arms");
    pc_action_4dof_loop();
    s_tick_ms += 1600U;
    pc_action_4dof_loop();
    pc_action_4dof_loop();
    require_true(s_valve_shadow[2] == 0U && s_valve_shadow[3] == 0U &&
                 relay_seen_after(0U, 2U, 0U) &&
                 relay_seen_after(0U, 3U, 0U),
                 "dual get-back closes both source back valves");
    require_true(!relay_seen_after(0U, 0U, 0U) &&
                 !relay_seen_after(0U, 1U, 0U),
                 "dual get-back never closes working arm valves");

    reset_test_state();
    s_limit_exact_path_y = true;
    s_exact_path_y_limit = 0.22f;
    s_projection_y_adjust = -0.03f;
    target = (Dof4_Pose){0.20f, 0.20f, 0.30f, 0.0f};
    require_true(pc_action_4dof_start_pick(DOF4_ARM_LEFT, &target),
                 "projectable path point request is queued");
    advance_dynamic_to_target();
    require_true(s_resolve_path_calls == 1U,
                 "unreachable outward point invokes reachable projection");
    require_near(s_last_resolve_max_adjust_m, 0.10f,
                 "path point projection uses 10 cm tolerance");
    require_near(s_last_resolve_requested.y, 0.25f,
                 "projection receives requested 5 cm outward point");
    advance_dynamic_target_settle();
    pc_action_4dof_loop();
    finish_dynamic_after_operation(1500U);
    require_true(arm_target_pose_seen(DOF4_ARM_LEFT, 0.20f, 0.22f, 0.42f),
                 "projected reachable outward point is executed");

    reset_test_state();
    s_limit_exact_path_y = true;
    s_exact_path_y_limit = 0.22f;
    s_projection_y_adjust = -0.11f;
    target = (Dof4_Pose){0.20f, 0.20f, 0.30f, 0.0f};
    require_true(pc_action_4dof_start_pick(DOF4_ARM_LEFT, &target),
                 "over-tolerance path point request is queued");
    pc_action_4dof_loop();
    require_true(!pc_action_4dof_is_active(),
                 "path point beyond 10 cm tolerance is rejected");
    require_true(s_resolve_path_calls == 1U,
                 "over-tolerance path point attempts one projection");
    require_true(g_dof4_arm_left.clip_diagnostic.joint_mask ==
                     PC_ACTION_4DOF_REJECT_PATH_POINT_UNREACHABLE,
                 "over-tolerance path point rejection reason");

    reset_test_state();
    require_true(pc_action_4dof_start_pick(DOF4_ARM_LEFT, &target),
                 "first pending request accepted");
    require_true(!pc_action_4dof_start_pick(DOF4_ARM_RIGHT, &target),
                 "second pending request rejected");
    require_true(g_dof4_arm_right.clip_diagnostic.pending,
                 "pending full diagnostic emitted");
    require_true(g_dof4_arm_right.clip_diagnostic.joint_mask ==
                     PC_ACTION_4DOF_REJECT_PENDING_FULL,
                 "pending full reason");

    reset_test_state();
    target = (Dof4_Pose){0.20f, 0.10f, 0.70f, 0.0f};
    require_true(pc_action_4dof_start_pick(DOF4_ARM_LEFT, &target),
                 "unreachable target is accepted into pending");
    pc_action_4dof_loop();
    require_true(!pc_action_4dof_is_active(),
                 "unreachable pending request is cleared");
    require_true(g_dof4_arm_left.clip_diagnostic.pending,
                 "unreachable diagnostic emitted");
    require_true(g_dof4_arm_left.clip_diagnostic.joint_mask ==
                     PC_ACTION_4DOF_REJECT_TARGET_UNREACHABLE,
                 "unreachable reason");
    require_true(s_resolve_path_calls == 0U,
                 "final unreachable target is never projected");

    reset_test_state();
    target = (Dof4_Pose){0.20f, 0.10f, 0.10f, 0.0f};
    require_true(pc_action_4dof_start_pick(DOF4_ARM_LEFT, &target),
                 "busy setup request accepted");
    pc_action_4dof_loop();
    require_true(!pc_action_4dof_start_pick(DOF4_ARM_RIGHT, &target),
                 "active PC action rejects new request");
    require_true(g_dof4_arm_right.clip_diagnostic.joint_mask ==
                     PC_ACTION_4DOF_REJECT_BUSY,
                 "busy reason");

    reset_test_state();
    s_action_active = true;
    require_true(!pc_action_4dof_start_pick(DOF4_ARM_LEFT, &target),
                 "preset action rejects PC request");
    require_true(g_dof4_arm_left.clip_diagnostic.joint_mask ==
                     PC_ACTION_4DOF_REJECT_ACTION_ACTIVE,
                 "action active reason");

    printf("pc action executor pending test passed\n");
    return 0;
}
