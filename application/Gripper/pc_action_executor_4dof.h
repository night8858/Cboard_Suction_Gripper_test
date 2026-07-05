#ifndef PC_ACTION_EXECUTOR_4DOF_H
#define PC_ACTION_EXECUTOR_4DOF_H

#include <stdbool.h>
#include <stdint.h>

#include "Dof4_Arm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PC 动态取放悬停高度和俯仰角由各模板独立控制，不再使用全局宏。 */

typedef enum {
    PC_ACTION_4DOF_REJECT_NONE = 0,                     /**< 无拒绝 */
    PC_ACTION_4DOF_REJECT_INVALID_ARM = 1,              /**< 非法手臂 ID */
    PC_ACTION_4DOF_REJECT_BAD_TARGET = 2,               /**< 非法目标位姿（非有限值） */
    PC_ACTION_4DOF_REJECT_TARGET_UNREACHABLE = 3,       /**< 目标位姿不可达 */
    PC_ACTION_4DOF_REJECT_TARGET_ABOVE_UNREACHABLE = 4, /**< 目标正上方悬停点不可达 */
    PC_ACTION_4DOF_REJECT_JOINT_PATH_INVALID = 5,       /**< 关节路径点不可达或非法 */
    PC_ACTION_4DOF_REJECT_BUSY = 6,                     /**< PC 动作状态机正忙，无法接受新请求 */
    PC_ACTION_4DOF_REJECT_COMPLETION_PENDING = 7,       /**< 上一次动作的结束事件尚未发送，无法接受新请求 */
    PC_ACTION_4DOF_REJECT_ACTION_ACTIVE = 8,            /**< 预设动作状态机正忙，无法接受新请求 */
    PC_ACTION_4DOF_REJECT_PENDING_FULL = 9,             /**< 待处理请求队列已满，无法接受新请求 */
    PC_ACTION_4DOF_REJECT_PATH_POINT_UNREACHABLE = 10,  /**< 路径中间点不可达 */
} PcAction4DOF_RejectReason;

typedef struct {
    bool active;
    bool pending;
    bool use_left;
    bool use_right;
    bool release_committed;
    bool operation_phase_started;
    bool joint_phase_started;
    uint8_t command;
    uint8_t mode;
    uint8_t state;
    uint8_t joint_substate;
    uint8_t path_index;
    uint32_t timeout_ms;
    uint32_t elapsed_ms;
} PcAction4DOF_DebugSnapshot;

/**
 * @brief 初始化 PC 专用 4DOF 动作状态机。
 *
 * 该状态机只处理 0x11/0x12/0x14/0x15/0x21/0x22/0x23/0x24，遥控器调试入口暂不接入。
 */
void pc_action_4dof_init(void);

/** @brief 周期推进 PC 动作状态机，应在 4DOF 控制任务中每周期调用一次。 */
void pc_action_4dof_loop(void);

/** @brief 强制中断当前/待处理 PC 动作，不产生动作完成帧。 */
void pc_action_4dof_abort(void);

/** @brief 查询 PC 动作是否正在执行，用于控制任务选择 PC 动作或 IDLE 目标。 */
bool pc_action_4dof_is_active(void);

/** @brief 读取 PC 动作状态机调试快照，可用于 IDE watch 或临时上位机调试。 */
void pc_action_4dof_get_debug_snapshot(PcAction4DOF_DebugSnapshot *snapshot);

void pc_action_4dof_record_reject(Dof4_ArmId arm_id,
                                  PcAction4DOF_RejectReason reason,
                                  const Dof4_Pose *target_world);

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

/** @brief 启动双臂动态放块动作（0x23）。 */
bool pc_action_4dof_start_dual_place(const Dof4_Pose *left_target_world,
                                     const Dof4_Pose *right_target_world);

/** @brief 启动双臂同时放块到对应背部动作（0x22）。 */
bool pc_action_4dof_start_dual_put_back(void);

/** @brief 启动双臂同时从对应背部取块动作（0x24）。 */
bool pc_action_4dof_start_dual_get_back(void);

/** @brief 查询是否有一条待发送的 PC 动作结束事件。 */
bool pc_action_4dof_completion_pending(void);

/** @brief 0xCC 发送成功后确认并清除结束事件。 */
void pc_action_4dof_completion_acknowledge(void);

#ifdef __cplusplus
}
#endif

#endif /* PC_ACTION_EXECUTOR_4DOF_H */
