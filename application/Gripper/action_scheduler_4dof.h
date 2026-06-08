/**
 * @file    action_scheduler_4dof.h
 * @brief   4DOF 双臂动作调度器 —— 基于状态机的抓取/放置/舞蹈动作管理
 *
 * ## 设计原则
 *
 * - **与 planar 臂完全解耦**：本模块仅操作 g_dof4_arm_left / g_dof4_arm_right，
 *   不涉及 target_x_test[] / planar_arm_control_loop()
 * - **状态机驱动**：每个动作拆分为多个子状态（预就位→抓取/放置→撤退→完成），
 *   按固定周期推进，含超时保护
 * - **目标位置预留**：所有坐标均为占位值（TODO），后续调试后替换
 * - **中间路径点**：以注释方式预留在各动作的目标数组中，需要时取消注释即可
 *
 * ## 集成方式
 *
 * 在 arm_control_task (DOF4_ARM 分支) 的 for(;;) 循环中，按以下顺序调用：
 *
 * ```
 * input_arbiter_update_rc(get_remote_control_point());
 * input_arbiter_resolve_4dof(false);      // RC/PC 手动控制
 * action_4dof_loop();                       // ★ 动作调度（动作激活时覆盖目标）
 * Dof4_dual_arm_control_loop(...);         // IK + 轨迹 + 舵机输出
 * ```
 *
 * ## 扩展新动作的步骤
 *
 * 1. 在 action_scheduler.h 的 action_state_4dof_e 中新增动作枚举值
 * 2. 在 action_scheduler_4dof.c 的 s_action_targets[] 中添加目标数据
 * 3. 在 action_4dof_handle() 中增加 case 分支
 * 4. 若需要外部触发，声明对应的 trigger 函数
 */

#ifndef ACTION_SCHEDULER_4DOF_H
#define ACTION_SCHEDULER_4DOF_H

#include <stdbool.h>
#include <stdint.h>
#include "action_scheduler.h"   /* 使用其中定义的 action_state_4dof_e */
#include "Dof4_Arm.h"           /* Dof4_Pose, Dof4_ArmId */

#ifdef __cplusplus
extern "C" {
#endif

/* ════════════════════════════════════════════════════════════════
 * 动作子状态枚举
 *
 * 每个 action_state_4dof_e 动作在内部被拆分为以下子状态逐步推进。
 * 并非所有动作都会用到全部子状态（例如纯抓取动作不需要 PLACE_* 阶段）。
 * ════════════════════════════════════════════════════════════════ */

typedef enum {
    ACTION_4DOF_SUBSTATE_IDLE            = 0,  /**< 空闲，无动作执行 */

    /* ── 抓取流程子状态 ── */
    ACTION_4DOF_SUBSTATE_APPROACH        = 1,  /**< ① 移动到预就位点（抓取点上方/前方安全位） */
    ACTION_4DOF_SUBSTATE_GRAB            = 2,  /**< ② 移动到抓取点（贴近物块） */
    ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_GRAB      = 3,  /**< ③ 开启吸盘，等待吸附确认（微动开关） */

    /* ── 撤退/转移子状态 ── */
    ACTION_4DOF_SUBSTATE_RETREAT         = 4,  /**< ④ 携带物块撤退到安全高度 */

    /* ── 放置流程子状态 ── */
    ACTION_4DOF_SUBSTATE_PLACE_APPROACH  = 5,  /**< ⑤ 移动到放置预就位点 */
    ACTION_4DOF_SUBSTATE_PLACE           = 6,  /**< ⑥ 移动到放置点（物块就位） */
    ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_PLACE     = 7,  /**< ⑦ 关闭吸盘，释放物块，等待脱离确认 */

    /* ── 完成子状态 ── */
    ACTION_4DOF_SUBSTATE_COMPLETE        = 8,  /**< ⑧ 移动到安全归位点，动作结束 */

    /* ── 特殊子状态 ── */
    ACTION_4DOF_SUBSTATE_WAYPOINT        = 9,  /**< 途经点（用于 DANCE 等多步动作） */
    ACTION_4DOF_SUBSTATE_INTERMEDIATE    = 10, /**< 中间途经点（抓取/放置流程的绕行点） */
} action_4dof_substate_e;


/**
 * @brief 4dof机械臂全局动作状态机状态枚举
 *
 * 【扩展方法】
 *   需要新增动作类型时, 在此枚举中追加新的状态常量.
 *   例如要增加"舞蹈动作":
 *     ACTION_STATE_DANCE = 2,
 *
 *   然后在 action_scheduler.c 中:
 *     1. 实现 dance_state_machine() 处理函数.
 *     2. 在 ACTION_loop() 末尾增加:
 *        case ACTION_STATE_DANCE: dance_state_machine(); break;
 *     3. 如需外部触发, 声明 dance_trigger() 并在 cmd_dispatch_frame 中增加命令字.
 */
typedef enum {
    ACTION_4DOF_IDLE      = 0,                                    /**< 空闲, 等待动作指令 */
    ACTION_BLOCK_GET_FORWARD = 1,                                 /**< 前侧物块同时抓取   */
    ACTION_BLOCK_GET_FORWARD_LEFT_ARM = 2,                        /**< 前侧物块左臂抓取   */
    ACTION_BLOCK_GET_FORWARD_RIGHT_ARM = 3,                       /**< 前侧物块右臂抓取   */
    ACTION_BLOCK_PLACE_BACK = 4,                                  /**< 双臂同时放置物块到对应后背 */
    ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_BACK = 5,                 /**< 左臂放置物块到左背 */
    ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_BACK =  6,              /**< 右臂放置物块到右背 */
    ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F1 = 7,            /**< 右臂放置物块到左1放置点第一层 */
    ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F2 = 8,            /**< 右臂放置物块到左1放置点第二层 */
    ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F1 = 9,          /**< 右臂放置物块到右1放置点第一层 */
    ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F2 = 10,         /**< 右臂放置物块到右1放置点第二层 */
    ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_LEFT_ARM = 11,                  /**< 从左背抓取到左手上 */
    ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_LEFT_ARM = 12,                 /**< 从右背抓取到左手上 */
    ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_RIGHT_ARM = 13,                  /**< 从左背抓取到右手上 */
    ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_RIGHT_ARM = 14,                 /**< 从右背抓取到右手上 */
    ACTION_DANCE = 15,                                            /**< 神秘舞蹈动作 */
    /* ── 预留扩展位 (按需取消注释) ── */

} action_state_4dof_e;


/* ════════════════════════════════════════════════════════════════
 * 全局物块位置状态
 *
 * 用于跟踪物块的当前位置（左/右背、左/右手），
 * 驱动 IDLE 位姿的自适应调整。
 * ════════════════════════════════════════════════════════════════ */

/** @brief 物块位置追踪状态（仅追踪背部储物区，手部状态由微动开关实时反映） */
typedef struct {
    bool left_back;        /**< 左背储物区有物块 */
    bool right_back;       /**< 右背储物区有物块 */
} BlockPlacementState;

extern BlockPlacementState g_block_state;

/** @brief IDLE 位姿参数 — 相对于各自基座 (effective_base) 的偏移量 */
#define IDLE_BASE_X      0.08f   /**< 基座前方 X 偏移，单位 m */
#define IDLE_BASE_Y      0.00f   /**< 基座侧方 Y 偏移，单位 m（0=正前方） */
#define IDLE_BASE_Z      0.27f   /**< 基座上方 Z 偏移，单位 m */
#define IDLE_BASE_PITCH -0.02f   /**< 基准 pitch，单位 rad */

/** @brief 根据物块位置状态获取指定臂的自适应 IDLE 位姿 */
Dof4_Pose action_4dof_get_idle_pose(Dof4_ArmId arm_id);


/* ════════════════════════════════════════════════════════════════
 * 公共 API
 * ════════════════════════════════════════════════════════════════ */

/** @brief 初始化 4DOF 动作调度器（将内部状态机复位到 IDLE） */
void action_4dof_init(void);

/**
 * @brief 4DOF 动作调度主循环（每控制周期调用一次）
 *
 * 若当前有激活的动作，按子状态机推进：
 *   1. 将当前阶段的目标位姿写入 g_dof4_arm_left/right（覆盖手动控制）
 *   2. 控制吸盘/电磁阀（relay_control）
 *   3. 检查到位条件（超时或传感器），推进到下一子状态
 *
 * 若当前无动作（IDLE），本函数立即返回，不影响手动控制。
 */
void action_4dof_loop(void);

/** @brief 触发指定动作（仅 IDLE 时可触发） */
bool action_4dof_trigger(action_state_4dof_e action);

/** @brief 强制中止当前动作，回到 IDLE，关闭所有吸盘 */
void action_4dof_abort(void);

/** @brief 查询动作是否正在执行中 */
bool action_4dof_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* ACTION_SCHEDULER_4DOF_H */
