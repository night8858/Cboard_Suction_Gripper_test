#include "Dof4_Arm.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void require_ok(Dof4_Status st, const char *what)
{
    if (st != DOF4_STATUS_OK) {
        printf("%s failed: status=%d\n", what, (int)st);
        exit(1);
    }
}

static void require_servo_in_range(const Dof4_Arm *arm,
                                   int16_t servo,
                                   const char *name)
{
    if (servo < arm->cfg.servo_min[0] || servo > arm->cfg.servo_max[0]) {
        printf("%s J1 servo out of range: %d\n", name, (int)servo);
        exit(1);
    }
}

static int16_t solve_left_j1_servo(Dof4_Arm *left,
                                   const Dof4_Pose *pose,
                                   const char *name)
{
    Dof4_JointState joints;
    require_ok(Dof4_arm_inverse_kinematics(left, pose, -1.0f, &joints), name);

    int16_t servo = 0;
    require_ok(Dof4_angle_to_servo(left, 0U, joints.q[0], &servo), name);
    require_servo_in_range(left, servo, name);

    float feedback_angle = 0.0f;
    require_ok(Dof4_servo_to_angle(left, 0U, servo, &feedback_angle), name);
    const float equivalent_error =
        fabsf(Dof4_normalize_angle(feedback_angle - joints.q[0]));
    if (equivalent_error > 0.002f) {
        printf("%s J1 equivalent angle error too large: %.6f rad\n",
               name,
               (double)equivalent_error);
        exit(1);
    }
    return servo;
}

int main(void)
{
    Dof4_Arm left;
    Dof4_Arm right;
    require_ok(Dof4_dual_arm_init(&left, &right), "dual init");

    const Dof4_Pose approach = {0.24f, 0.13f, 0.13f, 0.0f};
    const Dof4_Pose waypoint = {0.12f, 0.26f, 0.13f, 0.0f};
    const Dof4_Pose target   = {-0.22f, 0.185f, 0.30f, -1.4f};
    const Dof4_Pose retreat  = {0.11f, 0.29f, 0.155f, -0.10f};
    const Dof4_Pose complete = {0.02f, 0.00f, 0.20f, -0.02f};

    (void)solve_left_j1_servo(&left, &approach, "approach");
    (void)solve_left_j1_servo(&left, &waypoint, "waypoint");
    (void)solve_left_j1_servo(&left, &target, "target");
    (void)solve_left_j1_servo(&left, &retreat, "retreat");
    (void)solve_left_j1_servo(&left, &complete, "complete");

    printf("dof4 J1 wrap test passed\n");
    return 0;
}
