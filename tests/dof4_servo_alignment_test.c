#include "Dof4_Arm.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_FLOAT_EPS 1.0e-5f

static void fail(const char *message)
{
    printf("FAIL: %s\n", message);
    exit(1);
}

static void require_ok(Dof4_Status status, const char *message)
{
    if (status != DOF4_STATUS_OK) {
        printf("FAIL: %s status=%d\n", message, (int)status);
        exit(1);
    }
}

static void require_near(float actual, float expected, float tolerance, const char *message)
{
    if (fabsf(actual - expected) > tolerance) {
        printf("FAIL: %s expected=%.6f actual=%.6f\n",
               message,
               (double)expected,
               (double)actual);
        exit(1);
    }
}

static void require_true(bool value, const char *message)
{
    if (!value) {
        fail(message);
    }
}

static float pose_position_error(const Dof4_Pose *a, const Dof4_Pose *b)
{
    const float dx = a->x - b->x;
    const float dy = a->y - b->y;
    const float dz = a->z - b->z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static void simulate_servo_feedback(Dof4_Arm *arm)
{
    for (uint8_t i = 0U; i < DOF4_JOINT_COUNT; ++i) {
        arm->servo_pos[i] = arm->target_servo_pos[i];
    }
    require_ok(Dof4_arm_read_servo_pos(arm), "read simulated servo feedback");
}

static void verify_pose_round_trip(Dof4_Arm *arm,
                                   const Dof4_Pose *target,
                                   const char *name)
{
    Dof4_JointState joints;
    require_ok(Dof4_arm_inverse_kinematics(arm, target, -1.0f, &joints), name);
    require_ok(Dof4_arm_set_joint_target(arm, &joints), name);

    if (arm->active_clip_joint_mask != 0U) {
        printf("FAIL: %s unexpectedly clipped mask=0x%02X reason=0x%02X\n",
               name,
               arm->active_clip_joint_mask,
               arm->active_clip_reason);
        exit(1);
    }

    simulate_servo_feedback(arm);
    require_near(pose_position_error(&arm->current_pose, target),
                 0.0f,
                 0.0015f,
                 "IK-servo-feedback-FK position");
    require_near(Dof4_normalize_angle(arm->current_pose.pitch - target->pitch),
                 0.0f,
                 0.0035f,
                 "IK-servo-feedback-FK pitch");
}

int main(void)
{
    Dof4_Arm left;
    Dof4_Arm right;
    require_ok(Dof4_dual_arm_init(&left, &right), "dual arm init");

    require_near(left.cfg.servo_offset[0], L_J1_ZERO_BIAS_RAD, TEST_FLOAT_EPS, "L J1 offset");
    require_near(left.cfg.servo_offset[1], L_J2_ZERO_BIAS_RAD, TEST_FLOAT_EPS, "L J2 offset");
    require_near(left.cfg.servo_offset[2], L_J3_ZERO_BIAS_RAD, TEST_FLOAT_EPS, "L J3 offset");
    require_near(left.cfg.servo_offset[3], L_J4_ZERO_BIAS_RAD, TEST_FLOAT_EPS, "L J4 offset");
    require_near(right.cfg.servo_offset[0], R_J1_ZERO_BIAS_RAD, TEST_FLOAT_EPS, "R J1 offset");
    require_near(right.cfg.servo_offset[1], R_J2_ZERO_BIAS_RAD, TEST_FLOAT_EPS, "R J2 offset");
    require_near(right.cfg.servo_offset[2], R_J3_ZERO_BIAS_RAD, TEST_FLOAT_EPS, "R J3 offset");
    require_near(right.cfg.servo_offset[3], R_J4_ZERO_BIAS_RAD, TEST_FLOAT_EPS, "R J4 offset");

    for (uint8_t i = 0U; i < DOF4_JOINT_COUNT; ++i) {
        require_near(left.joint_actual.q[i],
                     left.cfg.servo_offset[i],
                     TEST_FLOAT_EPS,
                     "left initial feedback angle");
        require_near(right.joint_actual.q[i],
                     right.cfg.servo_offset[i],
                     TEST_FLOAT_EPS,
                     "right initial feedback angle");
    }

    require_near(left.cfg.joint_min[1], L_J2_ZERO_BIAS_RAD - 3.25f, TEST_FLOAT_EPS, "L J2 min");
    require_near(left.cfg.joint_max[1], L_J2_ZERO_BIAS_RAD + 2.25f, TEST_FLOAT_EPS, "L J2 max");
    require_near(left.cfg.joint_min[3], L_J4_ZERO_BIAS_RAD - 2.35f, TEST_FLOAT_EPS, "L J4 min");
    require_near(left.cfg.joint_max[3], L_J4_ZERO_BIAS_RAD + 2.35f, TEST_FLOAT_EPS, "L J4 max");
    require_near(left.cfg.joint_max[2], 0.50f, TEST_FLOAT_EPS, "L J3 straight max");
    require_near(right.cfg.joint_max[2], 0.50f, TEST_FLOAT_EPS, "R J3 straight max");

    int16_t left_j3_servo = 0;
    int16_t right_j3_servo = 0;
    require_ok(Dof4_angle_to_servo(&left, 2U, 0.0f, &left_j3_servo), "L J3 straight mapping");
    require_ok(Dof4_angle_to_servo(&right, 2U, 0.0f, &right_j3_servo), "R J3 straight mapping");
    require_true(abs((int)left_j3_servo - 1024) <= 1, "L J3 straight servo near 1024");
    require_true(abs((int)right_j3_servo - 1024) <= 1, "R J3 straight servo near 1024");

    Dof4_JointState straight = left.joint_actual;
    straight.q[2] = 0.0f;
    require_ok(Dof4_arm_set_joint_target(&left, &straight), "L J3 straight target");
    require_true((left.active_clip_joint_mask & (1U << 2)) == 0U,
                 "L J3 straight target must not clip");

    const Dof4_Pose left_grab = {0.4f, 0.425f, -0.19f, -1.45f};
    const Dof4_Pose right_grab = {0.4f, -0.425f, -0.21f, -1.45f};
    const Dof4_Pose left_place = {0.425f, 0.40f, -0.19f, -1.45f};
    const Dof4_Pose right_place = {0.425f, -0.40f, -0.19f, -1.45f};
    const Dof4_Pose left_low_pick = {0.2224f, 0.2854f, -0.4500f, -1.57f};
    const Dof4_Pose right_pc_edge_biased = {0.51f, -0.22f, -0.03f, -1.48f};
    verify_pose_round_trip(&left, &left_grab, "left front grab");
    verify_pose_round_trip(&right, &right_grab, "right front grab");
    verify_pose_round_trip(&left, &left_place, "left front place");
    verify_pose_round_trip(&right, &right_place, "right front place");
    verify_pose_round_trip(&left, &left_low_pick, "left low pick");
    require_ok(Dof4_arm_set_tcp_offset(&right, 0.0f, 0.0f, -0.05f),
               "right runtime TCP offset");
    verify_pose_round_trip(&right, &right_pc_edge_biased, "right PC edge target after bias");

    Dof4_JointState over_limit = right.joint_actual;
    over_limit.q[1] = right.cfg.joint_max[1] + 0.2f;
    require_ok(Dof4_arm_set_joint_target(&right, &over_limit), "joint clipping continues");
    require_true((right.active_clip_reason & DOF4_CLIP_REASON_JOINT_LIMIT) != 0U,
                 "joint clip reason recorded");
    require_true((right.active_clip_joint_mask & (1U << 1)) != 0U,
                 "J2 clip mask recorded");
    require_near(right.joint_target.q[1],
                 right.cfg.joint_max[1],
                 TEST_FLOAT_EPS,
                 "J2 clipped to absolute limit");
    require_true(right.clip_diagnostic.pending, "clip diagnostic pending");

    const uint32_t first_event_id = right.clip_diagnostic.event_id;
    require_ok(Dof4_arm_set_joint_target(&right, &over_limit), "repeat same clipping");
    require_true(right.clip_diagnostic.event_id == first_event_id,
                 "same clip state must not create another event");

    Dof4_JointState valid = right.joint_actual;
    require_ok(Dof4_arm_set_joint_target(&right, &valid), "clear clipping state");
    require_true(right.active_clip_joint_mask == 0U, "valid target clears active clip state");
    require_ok(Dof4_arm_set_joint_target(&right, &over_limit), "clip after clear");
    require_true(right.clip_diagnostic.event_id == first_event_id + 1U,
                 "new clipping transition creates another event");

    Dof4_JointState servo_limited = left.joint_actual;
    servo_limited.q[2] = left.cfg.joint_min[2];
    require_ok(Dof4_arm_set_joint_target(&left, &servo_limited), "servo clipping continues");
    require_true((left.active_clip_reason & DOF4_CLIP_REASON_SERVO_LIMIT) != 0U,
                 "servo clip reason recorded");
    require_true((left.active_clip_joint_mask & (1U << 2)) != 0U,
                 "servo-limited J3 mask recorded");
    float expected_servo_limited_angle = 0.0f;
    require_ok(Dof4_servo_to_angle(&left,
                                   2U,
                                   left.cfg.servo_max[2],
                                   &expected_servo_limited_angle),
               "servo limit feedback conversion");
    require_near(left.joint_target.q[2],
                 expected_servo_limited_angle,
                 TEST_FLOAT_EPS,
                 "joint target reflects clamped servo position");

    printf("dof4 servo alignment test passed\n");
    return 0;
}
