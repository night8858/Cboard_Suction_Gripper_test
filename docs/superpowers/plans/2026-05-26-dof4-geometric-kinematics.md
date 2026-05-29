# DOF4 Geometric Kinematics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the simplified/garbled dual-arm module with a documented low-compute geometric FK/IK, Cartesian quintic trajectory planner, and capsule collision checks for STM32F407.

**Architecture:** Keep the existing module boundaries: `Dof4_Arm.*` owns configuration, FK, IK, servo conversion, and the one-step control loop; `Trajectory_Planning.*` owns quintic Cartesian sampling; `Dof4_Collision.*` owns capsule distance checks. Host-side tests validate the pure math before firmware integration.

**Tech Stack:** C11, STM32 HAL/FreeRTOS integration points, single-precision `math.h`, host GCC/Clang if available for tests.

---

### Task 1: Add Host-Side Math Tests

**Files:**
- Create: `tests/dof4_math_tests.c`

- [ ] **Step 1: Write failing tests for quintic boundaries, IK/FK round-trip, and segment distance**

```c
/* tests/dof4_math_tests.c */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Trajectory_Planning.h"
#include "Dof4_Arm.h"
#include "Dof4_Collision.h"

static void expect_close(float actual, float expected, float tol, const char *name)
{
    if (fabsf(actual - expected) > tol) {
        printf("FAIL %s: actual=%f expected=%f tol=%f\n", name, actual, expected, tol);
        exit(1);
    }
}

static void test_quintic_boundaries(void)
{
    Traj_QuinticSegment seg;
    Dof4_Status st = Traj_quintic_init(&seg, 0.0f, 0.0f, 0.0f,
                                       1.0f, 0.0f, 0.0f, 2.0f);
    if (st != DOF4_STATUS_OK) exit(1);
    float p, v, a;
    Traj_quintic_sample(&seg, 0.0f, &p, &v, &a);
    expect_close(p, 0.0f, 1.0e-5f, "p0");
    expect_close(v, 0.0f, 1.0e-5f, "v0");
    expect_close(a, 0.0f, 1.0e-5f, "a0");
    Traj_quintic_sample(&seg, 2.0f, &p, &v, &a);
    expect_close(p, 1.0f, 1.0e-5f, "pf");
    expect_close(v, 0.0f, 1.0e-4f, "vf");
    expect_close(a, 0.0f, 1.0e-4f, "af");
}

static void test_ik_fk_round_trip(void)
{
    Dof4_Arm arm;
    Dof4_ArmConfig cfg = Dof4_arm_default_config(DOF4_ARM_LEFT);
    cfg.ik_branch = DOF4_IK_BRANCH_NEAREST;
    if (Dof4_arm_config_init(&arm, &cfg) != DOF4_STATUS_OK) exit(1);

    Dof4_Pose target = {0.35f, 0.22f, 0.12f, -0.6f};
    Dof4_JointState joints;
    if (Dof4_arm_inverse_kinematics(&arm, &target, &joints) != DOF4_STATUS_OK) exit(1);
    Dof4_Pose actual;
    if (Dof4_arm_forward_kinematics(&arm, &joints, &actual) != DOF4_STATUS_OK) exit(1);
    expect_close(actual.x, target.x, 0.015f, "ik_fk_x");
    expect_close(actual.y, target.y, 0.015f, "ik_fk_y");
    expect_close(actual.z, target.z, 0.015f, "ik_fk_z");
    expect_close(actual.pitch, target.pitch, 0.03f, "ik_fk_pitch");
}

static void test_segment_distance(void)
{
    const float a[3] = {0.0f, 0.0f, 0.0f};
    const float b[3] = {1.0f, 0.0f, 0.0f};
    const float c[3] = {0.5f, 1.0f, 0.0f};
    const float d[3] = {0.5f, 1.0f, 1.0f};
    float dist = Dof4_col_segment_distance(a, b, c, d, NULL, NULL);
    expect_close(dist, 1.0f, 1.0e-5f, "segment_distance");
}

int main(void)
{
    test_quintic_boundaries();
    test_ik_fk_round_trip();
    test_segment_distance();
    printf("dof4 math tests passed\n");
    return 0;
}
```

- [ ] **Step 2: Run tests and verify they fail before implementation**

Run: `gcc -std=c11 -Iapplication/Gripper tests/dof4_math_tests.c application/Gripper/Trajectory_Planning.c application/Gripper/Dof4_Arm.c application/Gripper/Dof4_Collision.c -lm -o build/dof4_math_tests.exe`

Expected: FAIL because the new API types and functions are not yet defined.

### Task 2: Define Clean Public APIs

**Files:**
- Modify: `application/Gripper/Dof4_Arm.h`
- Modify: `application/Gripper/Trajectory_Planning.h`
- Modify: `application/Gripper/Dof4_Collision.h`

- [ ] **Step 1: Replace garbled comments and define types**

Define `Dof4_Status`, `Dof4_Pose`, `Dof4_JointState`, `Dof4_ArmConfig`, `Dof4_Arm`, `Traj_QuinticSegment`, `Dof4_CartesianPlanner`, and collision detail types with Doxygen comments.

- [ ] **Step 2: Keep compatibility wrappers where practical**

Keep `Dof4_dual_arm_control_loop`, `Dof4_arm_set_target`, and `Dof4_arm_get_by_id` names so existing callers can migrate gradually.

### Task 3: Implement Quintic Cartesian Trajectory

**Files:**
- Modify: `application/Gripper/Trajectory_Planning.c`

- [ ] **Step 1: Implement scalar quintic init/sample**

Implement `Traj_quintic_init` and `Traj_quintic_sample` with boundary validation and Horner evaluation.

- [ ] **Step 2: Implement Cartesian planner**

Implement `Dof4_cartesian_planner_init`, `Dof4_cartesian_planner_plan`, `Dof4_cartesian_planner_sample`, and target-change tracking for `x/y/z/pitch`.

- [ ] **Step 3: Run host test**

Expected: quintic test passes; IK/FK still fails until Task 4.

### Task 4: Implement Geometric FK/IK

**Files:**
- Modify: `application/Gripper/Dof4_Arm.c`

- [ ] **Step 1: Implement default configs from URDF**

Create `Dof4_arm_default_config` with left/right URDF base, limits, link lengths, workspace, servo defaults, and IK branch.

- [ ] **Step 2: Implement FK**

Implement `Dof4_arm_forward_kinematics` and internal helpers to fill `joint_world`.

- [ ] **Step 3: Implement IK candidates**

Implement elbow-up and elbow-down candidate generation, joint-limit validation, nearest-branch scoring, and servo conversion.

- [ ] **Step 4: Run host test**

Expected: quintic, IK/FK round-trip, and segment distance tests pass.

### Task 5: Implement Capsule Collision and Control Loop

**Files:**
- Modify: `application/Gripper/Dof4_Collision.c`
- Modify: `application/Gripper/Dof4_Arm.c`

- [ ] **Step 1: Implement segment distance and capsule checks**

Use line-segment minimum distance and per-link radii. Return `DOF4_STATUS_COLLISION_RISK` when unsafe.

- [ ] **Step 2: Implement one-step control loop**

Implement `Dof4_dual_arm_control_loop(left, right, now_ms)` without any internal frequency scheduling. It should sample trajectories, solve IK, predict FK, check collision, and update command servo positions.

- [ ] **Step 3: Check blocking calls**

Run: `Select-String -Path application/Gripper/*.c -Pattern 'HAL_Delay|osDelay'`

Expected: no blocking delay inside the periodic control loop.

### Task 6: Firmware Build / Static Verification

**Files:**
- Modify only if required: `CMakeLists.txt`

- [ ] **Step 1: Configure or build the project**

Run: `cmake --build build`

Expected: either successful build or a clear toolchain-related failure.

- [ ] **Step 2: Review requirement checklist**

Check Doxygen comments, config constants at file top, error returns, no ISR blocking delay, geometric IK, Cartesian trajectory, capsule collision, and `doc/detailed-design.md`.

