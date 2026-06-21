#ifndef PC_ACTION_EXECUTOR_4DOF_H
#define PC_ACTION_EXECUTOR_4DOF_H

#include <stdbool.h>

#include "Dof4_Arm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PC 动态取放目标正上方点的统一垂直净空，单位 m。 */
#define PC_ACTION_4DOF_VERTICAL_CLEARANCE_M 0.20f

/**
 * @brief 初始化 PC 专用 4DOF 动作状态机。
 *
 * 该状态机只处理 0x11/0x12/0x14/0x15/0x21/0x22，不执行 RC 预设动作。
 */
void pc_action_4dof_init(void);

/** @brief 周期推进 PC 动作状态机，应在 4DOF 控制任务中每周期调用一次。 */
void pc_action_4dof_loop(void);

/** @brief 查询 PC 动作是否正在执行，用于 RC/IDLE 目标仲裁。 */
bool pc_action_4dof_is_active(void);

/** @brief 启动单臂动态取块动作（0x11）。 */
bool pc_action_4dof_start_pick(Dof4_ArmId arm_id,
                               const Dof4_Pose *target_world);

/** @brief 启动单臂动态放块动作（0x12）。 */
bool pc_action_4dof_start_place(Dof4_ArmId arm_id,
                                const Dof4_Pose *target_world);

/** @brief 启动单臂放块到对应背部动作（0x14）。 */
bool pc_action_4dof_start_put_back(Dof4_ArmId arm_id);

/** @brief 启动单臂从对应背部取块动作（0x15）。 */
bool pc_action_4dof_start_get_back(Dof4_ArmId arm_id);

/** @brief 启动双臂动态取块动作（0x21）。 */
bool pc_action_4dof_start_dual_pick(const Dof4_Pose *left_target_world,
                                    const Dof4_Pose *right_target_world);

/** @brief 启动双臂同时放块到对应背部动作（0x22）。 */
bool pc_action_4dof_start_dual_put_back(void);

/** @brief 查询是否有一条待发送的 PC 动作结束事件。 */
bool pc_action_4dof_completion_pending(void);

/** @brief 0xCC 发送成功后确认并清除结束事件。 */
void pc_action_4dof_completion_acknowledge(void);

#ifdef __cplusplus
}
#endif

#endif /* PC_ACTION_EXECUTOR_4DOF_H */
