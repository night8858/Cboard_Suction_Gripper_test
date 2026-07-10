#include "Trajectory_Planning.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void require_near(float actual,
                         float expected,
                         float tolerance,
                         const char *message)
{
    if (fabsf(actual - expected) > tolerance) {
        printf("FAIL: %s expected=%.6f actual=%.6f\n",
               message,
               (double)expected,
               (double)actual);
        exit(1);
    }
}

static void require_true(int value, const char *message)
{
    if (!value) {
        printf("FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    const Dof4_Pose origin = {0.0f, 0.0f, 0.0f, 0.0f};
    const Dof4_Pose tiny = {0.001f, 0.0f, 0.0f, 0.0f};
    const Dof4_Pose medium = {0.4f, 0.0f, 0.0f, 0.0f};
    const Dof4_Pose long_move = {1.0f, 0.0f, 0.0f, 0.0f};
    const Dof4_Pose pitch_move = {0.0f, 0.0f, 0.0f, 1.0f};

    require_near(Dof4_cartesian_compute_duration(&origin,
                                                 &tiny,
                                                 DOF4_DEFAULT_CART_VEL_MPS,
                                                 DOF4_DEFAULT_PITCH_VEL_RPS),
                 DOF4_TRAJ_MIN_DURATION_S,
                 1.0e-5f,
                 "tiny move uses minimum duration");

    const float medium_duration = Dof4_cartesian_compute_duration(&origin,
                                                                  &medium,
                                                                  DOF4_DEFAULT_CART_VEL_MPS,
                                                                  DOF4_DEFAULT_PITCH_VEL_RPS);
    require_true(medium_duration > 0.37f && medium_duration < 0.39f,
                 "medium move duration respects quintic peak limits");

    const float long_duration = Dof4_cartesian_compute_duration(&origin,
                                                                &long_move,
                                                                DOF4_DEFAULT_CART_VEL_MPS,
                                                                DOF4_DEFAULT_PITCH_VEL_RPS);
    require_true(long_duration > 0.93f && long_duration < DOF4_TRAJ_MAX_DURATION_S,
                 "long move is no longer clipped by the old 0.8s cap");

    const float pitch_duration = Dof4_cartesian_compute_duration(&origin,
                                                                 &pitch_move,
                                                                 DOF4_DEFAULT_CART_VEL_MPS,
                                                                 DOF4_DEFAULT_PITCH_VEL_RPS);
    require_true(pitch_duration > 0.93f && pitch_duration < 0.95f,
                 "pitch move duration respects quintic peak velocity");

    printf("dof4 trajectory profile test passed\n");
    return 0;
}
