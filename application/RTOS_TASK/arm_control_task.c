/**
 * @file    arm_control_task.c
 * @brief   四自由度双臂机械臂主控任务 — 仅响应 PC 下发的动作指令
 *
 * 执行周期：~5ms (200Hz)
 * 流程：初始化 → 等待启动 → 开机姿态 → PC 动作 → 空闲回 IDLE → 底层舵机控制
 */

#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "Dof4_Arm.h"
#include "action_scheduler_4dof.h"
#include "pc_action_executor_4dof.h"

#include <math.h>

#define ARM_STARTUP_TIMEOUT_MS     3000U
#define ARM_STARTUP_POS_TOL_M      0.025f
#define ARM_STARTUP_PITCH_TOL_RAD  0.05f

static bool arm_control_pose_reached(const Dof4_Arm *arm,
                                     const Dof4_Pose *target,
                                     float pos_tol_m,
                                     float pitch_tol_rad)
{
    if (arm == NULL || target == NULL) {
        return false;
    }

    const float dx = arm->current_pose.x - target->x;
    const float dy = arm->current_pose.y - target->y;
    const float dz = arm->current_pose.z - target->z;
    const float pos_err = sqrtf(dx * dx + dy * dy + dz * dz);
    const float pitch_err = fabsf(Dof4_normalize_angle(arm->current_pose.pitch -
                                                       target->pitch));
    return (pos_err <= pos_tol_m) && (pitch_err <= pitch_tol_rad);
}

void arm_control_task(void *argument)
{
    (void)argument;
    osDelay(500);

    Dof4_dual_arm_init(&g_dof4_arm_left, &g_dof4_arm_right);
    Dof4_double_arm_Desable();
    (void)Dof4_servo_comm_check(1U, &g_dof4_servo_comm_diagnostic);
    Dof4_set_world_offset(0.0f, 0.0f, 0.0f);
    pc_action_4dof_init();
    osDelay(100);

    bool startup_executed = false;
    bool startup_running = false;
    uint32_t startup_start_ms = 0U;
    Dof4_Pose startup_idle_left = {0.0f, 0.0f, 0.0f, 0.0f};
    Dof4_Pose startup_idle_right = {0.0f, 0.0f, 0.0f, 0.0f};

    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        if (!g_dof4_arm_started) {
            Dof4_batch_read_all_servo(&g_dof4_arm_left, &g_dof4_arm_right);
            osDelay(5);
            continue;
        }

        if (!startup_executed) {
            if (!startup_running) {
                Dof4_double_arm_Enable();
                startup_idle_left  = action_4dof_get_idle_pose(DOF4_ARM_LEFT);
                startup_idle_right = action_4dof_get_idle_pose(DOF4_ARM_RIGHT);
                (void)Dof4_arm_set_target(&g_dof4_arm_left,
                                          startup_idle_left.x,
                                          startup_idle_left.y,
                                          startup_idle_left.z,
                                          startup_idle_left.pitch);
                (void)Dof4_arm_set_target(&g_dof4_arm_right,
                                          startup_idle_right.x,
                                          startup_idle_right.y,
                                          startup_idle_right.z,
                                          startup_idle_right.pitch);
                startup_start_ms = now_ms;
                startup_running = true;
            }

            (void)Dof4_dual_arm_control_loop(&g_dof4_arm_left,
                                             &g_dof4_arm_right,
                                             now_ms);

            const bool left_ok = arm_control_pose_reached(&g_dof4_arm_left,
                                                          &startup_idle_left,
                                                          ARM_STARTUP_POS_TOL_M,
                                                          ARM_STARTUP_PITCH_TOL_RAD);
            const bool right_ok = arm_control_pose_reached(&g_dof4_arm_right,
                                                           &startup_idle_right,
                                                           ARM_STARTUP_POS_TOL_M,
                                                           ARM_STARTUP_PITCH_TOL_RAD);
            const bool timed_out = ((uint32_t)(now_ms - startup_start_ms) >=
                                    ARM_STARTUP_TIMEOUT_MS);
            if ((left_ok && right_ok) || timed_out) {
                startup_executed = true;
                startup_running = false;
            }

            osDelay(5);
            continue;
        }

        pc_action_4dof_loop();

        if (!pc_action_4dof_is_active()) {
            const Dof4_Pose idle_left  = action_4dof_get_idle_pose(DOF4_ARM_LEFT);
            const Dof4_Pose idle_right = action_4dof_get_idle_pose(DOF4_ARM_RIGHT);
            (void)Dof4_arm_set_target(&g_dof4_arm_left,  idle_left.x,  idle_left.y,  idle_left.z,  idle_left.pitch);
            (void)Dof4_arm_set_target(&g_dof4_arm_right, idle_right.x, idle_right.y, idle_right.z, idle_right.pitch);
        }

        (void)Dof4_dual_arm_control_loop(&g_dof4_arm_left, &g_dof4_arm_right, now_ms);
        osDelay(5);
    }
}
