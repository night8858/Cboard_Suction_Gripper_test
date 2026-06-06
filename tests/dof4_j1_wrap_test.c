#include "Dof4_Arm.h"

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
    return servo;
}

static void require_near_i16(int16_t actual,
                             int16_t expected,
                             int16_t tolerance,
                             const char *name)
{
    const int diff = abs((int)actual - (int)expected);
    if (diff > tolerance) {
        printf("%s J1 servo expected near %d, got %d\n",
               name,
               (int)expected,
               (int)actual);
        exit(1);
    }
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

    const int16_t approach_servo = solve_left_j1_servo(&left, &approach, "approach");
    const int16_t waypoint_servo = solve_left_j1_servo(&left, &waypoint, "waypoint");
    const int16_t target_servo   = solve_left_j1_servo(&left, &target, "target");
    const int16_t retreat_servo  = solve_left_j1_servo(&left, &retreat, "retreat");
    const int16_t complete_servo = solve_left_j1_servo(&left, &complete, "complete");

    require_near_i16(approach_servo, 3088, 3, "approach");
    require_near_i16(waypoint_servo, 2111, 3, "waypoint");
    require_near_i16(target_servo, 1125, 3, "target");
    require_near_i16(retreat_servo, 2058, 3, "retreat");
    require_near_i16(complete_servo, 378, 3, "complete");

    printf("dof4 J1 wrap test passed\n");
    return 0;
}
