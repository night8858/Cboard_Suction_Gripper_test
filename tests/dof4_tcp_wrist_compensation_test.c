#include "Dof4_Arm.h"
#include "Dof4_Arm_Calibration.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_TOOL_DOWN_PITCH_RAD (-1.5707963267948966f)

static void require_true(bool value, const char *message)
{
    if (!value) {
        printf("FAIL: %s\n", message);
        exit(1);
    }
}

static void require_near(float actual,
                         float expected,
                         float tolerance,
                         const char *message)
{
    if (fabsf(actual - expected) > tolerance) {
        printf("FAIL: %s expected=%.7f actual=%.7f\n",
               message,
               (double)expected,
               (double)actual);
        exit(1);
    }
}

static void verify_downward_tcp_compensation(Dof4_Arm *arm,
                                              const Dof4_Pose *target,
                                              const char *label)
{
    Dof4_JointState joints;
    require_true(Dof4_arm_inverse_kinematics(arm,
                                             target,
                                             -1.0f,
                                             &joints) == DOF4_STATUS_OK,
                 label);

    Dof4_Pose actual;
    require_true(Dof4_arm_forward_kinematics(arm,
                                             &joints,
                                             &actual) == DOF4_STATUS_OK,
                 label);

    require_near(actual.x, target->x, 2.0e-5f, "TCP x round trip");
    require_near(actual.y, target->y, 2.0e-5f, "TCP y round trip");
    require_near(actual.z, target->z, 2.0e-5f, "TCP z round trip");
    require_near(actual.pitch,
                 TEST_TOOL_DOWN_PITCH_RAD,
                 2.0e-5f,
                 "TCP pitch remains downward");

    /* 零附加偏移且工具竖直向下时，腕部/J4 正好位于 TCP 上方 LT。 */
    require_near(arm->joint_world[3][0], target->x, 2.0e-5f,
                 "wrist x equals downward TCP x");
    require_near(arm->joint_world[3][1], target->y, 2.0e-5f,
                 "wrist y equals downward TCP y");
    require_near(arm->joint_world[3][2],
                 target->z + arm->cfg.link_len[2],
                 2.0e-5f,
                 "wrist z compensates tool length once");
}

int main(void)
{
    Dof4_Arm left;
    Dof4_Arm right;
    require_true(Dof4_dual_arm_init(&left, &right) == DOF4_STATUS_OK,
                 "dual arm init");

    const Dof4_Pose left_target = {
        0.30f, 0.15f, 0.10f, TEST_TOOL_DOWN_PITCH_RAD
    };
    const Dof4_Pose right_target = {
        0.30f, -0.15f, 0.10f, TEST_TOOL_DOWN_PITCH_RAD
    };
    verify_downward_tcp_compensation(&left, &left_target,
                                     "left downward TCP IK");
    verify_downward_tcp_compensation(&right, &right_target,
                                     "right downward TCP IK");

    const Dof4PcActionCalibration *pc = Dof4_calibration_get_pc_action();
    for (uint8_t arm_index = 0U; arm_index < DOF4_CALIB_ARM_COUNT;
         ++arm_index) {
        require_near(pc->pick[arm_index].target_pitch,
                     TEST_TOOL_DOWN_PITCH_RAD,
                     1.0e-7f,
                     "single pick pitch is downward");
        require_near(pc->dual_pick[arm_index].target_pitch,
                     TEST_TOOL_DOWN_PITCH_RAD,
                     1.0e-7f,
                     "dual pick pitch is downward");
        require_near(pc->place[arm_index].target_pitch,
                     TEST_TOOL_DOWN_PITCH_RAD,
                     1.0e-7f,
                     "place pitch is downward");
    }

    printf("dof4 TCP wrist compensation test passed\n");
    return 0;
}
