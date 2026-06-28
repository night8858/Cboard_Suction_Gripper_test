/**
 * @file    action_scheduler_4dof.c
 * @brief   4DOF 双臂动作调度器实现 —— 状态机驱动的抓取/放置/舞蹈动作
 *
 * ## 状态机总览
 *
 * 抓取类动作（ACTION_BLOCK_GET_*）:
 *   IDLE → APPROACH → GRAB → SUCTION_ON → RETREAT → COMPLETE → IDLE
 *
 * 放置类动作（ACTION_BLOCK_PLACE_*）:
 *   后背放置（放置到背部储物区）:
 *     IDLE → PLACE_APPROACH → (INTERMEDIATE) → PLACE(到达→预释放等待→关阀→释放后等待) → RETREAT → COMPLETE → IDLE
 *   外部放置（放置到机体外堆叠点）:
 *     IDLE → PLACE_APPROACH → (INTERMEDIATE) → PLACE(到达→关阀→释放后等待) → RETREAT → COMPLETE → IDLE
 *
 * DANCE:
 *   IDLE → WAYPOINT(0) → WAYPOINT(1) → ... → COMPLETE → IDLE
 *
 * ## 超时策略
 *
 * - 移动子状态（APPROACH/GRAB/RETREAT/PLACE_*）：固定延时，后续可改为末端到位判断
 * - 吸附子状态（SUCTION_CONTROL_GRAB）：固定延时等待吸附，超时后容错推进
 * - 释放子状态（SUCTION_CONTROL_PLACE）：先关闭手臂电磁阀，再等待物块稳定落下/交接后再撤退
 * - 每个子状态有独立超时，超时后记录错误并推进（不卡死）
 *
 * ## 电磁阀策略
 *
 * - 动作触发时手臂电磁阀打开（提供负压吸取物块），动作中常开
 * - 放置动作到达目标点后关闭手臂电磁阀（通大气释放物块），等待物块稳定后再撤退
 * - 动作完成后（回到 IDLE）手臂电磁阀重新打开，防止真空泵憋压
 * - 背部吸盘电磁阀仅在物块交接期间打开，由 g_block_state 跟踪背储物状态
 *
 * ## 目标坐标说明
 *
 * - 所有坐标均为世界坐标系（base_link 原点），单位 米(m) / 弧度(rad)
 * - 左臂基座位于 (0.10760, +0.13342, 0.011815)（URDF）
 * - 右臂基座位于 (0.10758, -0.13265, 0.011815)
 * - 以下占位值均需**实测后替换**（标记 TODO）
 * - 每个动作支持最多 3 个中间途经点（WAYPOINT_0/1/2），以注释形式预留
 */

#include "action_scheduler_4dof.h"
#include "Dof4_Arm.h"
#include "pneumatic_control.h"
#include "pc_action_executor_4dof.h"
#include "Trajectory_Planning.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <math.h>


/// ════════════════════════════════════════════════════════════════
//特殊角度
#define STAY_LEVEL 0.0f
#define STAY_DOWN  -1.5f

#define BLOCK_GET_DOWN_Z -0.22f    //TODO: 实测调整z的下降目标高度，确保能贴近物块但不碰撞

//特殊距离设定
#define BLOCK_HEIGHT               -0.08f
//小狗为-0.03f   测试架为
#define BLOCK_FIRST_LAYER_HEIGHT  -0.21f                                    /// 实测调整第一层堆叠时的放置高度，同时也是第一层的抓取高度，确保能抓取到物块且不碰撞
#define BLOCK_SECOND_LAYER_HEIGHT  (BLOCK_FIRST_LAYER_HEIGHT  + 0.25f)   

//雷达给的是0.3的x
#define BLOCK_X_DESTANCE_FROM_BASE 0.37f                                      /// 物块距臂基座的x水平距离，实测调整确保在臂工作空间内且能抓取到物块,主要由于雷达的目标点决定

#define BLOCK_Y_DESTANCE_FROM_BASE_PICK  0.32f                                      /// 物块距臂基座的y水平距离，实测调整确保在臂工作空间内且能抓取到物块,主要由于雷达的目标点决定 
#define BLOCK_Y_DESTANCE_FROM_BASE_PLACE 0.40f                                       /// 物块距臂基座的y水平距离，实测调整确保在臂工作空间内且能放置到放置区,主要由于雷达的目标点决定 
/* ════════════════════════════════════════════════════════════════
 * 外部引用
 * ════════════════════════════════════════════════════════════════ */


extern Dof4_Arm g_dof4_arm_left;
extern Dof4_Arm g_dof4_arm_right;

/** @brief 全局物块位置追踪状态 */
BlockPlacementState g_block_state;

/* ════════════════════════════════════════════════════════════════
 * 吸盘/电磁阀映射
 *
 * relay_control(id, state):
 *   id=0→左臂吸盘, id=1→右臂吸盘, id=2→左背吸盘, id=3→右背吸盘
 *   state=1→开启(吸取/吸附), state=0→关闭(释放)
 *
 * 当前动作调度不依赖吸附传感器确认，吸附/释放确认按状态机定时推进。
 * ════════════════════════════════════════════════════════════════ */
#define RELAY_LEFT_ARM    0U  /**< 左臂吸盘电磁阀（一号） */
#define RELAY_RIGHT_ARM   1U  /**< 右臂吸盘电磁阀（二号） */
#define RELAY_LEFT_BACK   2U  /**< 左背吸盘电磁阀（三号） */
#define RELAY_RIGHT_BACK  3U  /**< 右背吸盘电磁阀（四号） */
#define SUCTION_ON        1
#define SUCTION_OFF       0

/* ════════════════════════════════════════════════════════════════
 * 时机参数（单位 ms，后续根据实测调整）
 * ════════════════════════════════════════════════════════════════ */

#define ACT4_MOVE_TIMEOUT_MS           2500U   /**< 单段移动最大超时（容错兜底） */
#define ACT4_SUCTION_TIMEOUT_MS        1500U   /**< 吸附等待最大超时（前方抓取用） */
#define ACT4_PLACE_HOLD_MS              1000U   /**< 背部取回动作：手臂在抓取点等待背部吸盘释放的时间 */
#define ACT4_BACK_RELEASE_HOLD_MS       500U   /**< 背部吸盘关闭后等待物块交接稳定时间 */
/* ── 放置动作释放时序宏（通过修改宏值即可调试，无需改动逻辑代码）── */

/** @brief 后背放置：到达目标点后、关闭手臂电磁阀前的预释放等待时间
 *  此阶段背部吸盘已打开，等待其牢固吸附物块后再松开手臂吸盘 */
#define ACT4_PLACE_PRE_RELEASE_BACK_MS  2000U

/** @brief 后背放置：关闭手臂电磁阀后、撤退前的释放后等待时间
 *  确保物块已完全交接给背部吸盘，机械臂移动不会带偏/刮碰物块 */
#define ACT4_PLACE_POST_RELEASE_BACK_MS 2000U

/** @brief 外部放置：关闭手臂电磁阀后、撤退前的释放后等待时间
 *  物块靠重力落向堆叠点，需等待其稳定后再移动机械臂 */
#define ACT4_PLACE_POST_RELEASE_EXT_MS  2000U

#define ACT4_RELEASE_TIMEOUT_MS        1000U   /**< 释放等待最大超时（保留备用） */
#define ACT4_HOLD_MS                    100U   /**< 完成后保持时间（回 IDLE 前） */
#define ACT4_WAYPOINT_HOLD_MS           100U   /**< 途经点停留时间（DANCE 用） */

/* ── 多段轨迹平滑参数 ── */

/** @brief 运动链中间途经点的默认前瞻切换距离，单位 m。
 *  当机械臂末端距离当前目标小于此距离时，提前下发下一段目标，
 *  使 5 次多项式规划器在速度尚高时自然衔接，消除走-停-走顿挫。 */
#define ACT4_DEFAULT_BLEND_DIST_M        0.08f

/** @brief 运动链中间途经点的默认通过速度因子。
 *  0.0 = 零速停止（精确到位），0.5~0.7 = 以最高速度的 50%~70% 通过。
 *  仅对末端位姿模式（POSE）生效；关节模式（JOINT）仅使用 blend_distance。 */
#define ACT4_DEFAULT_VIA_SPEED_FACTOR    0.6f

/** @brief 运动链终点（需精确停止执行吸取/释放）的前瞻切换距离，单位 m。
 *  比中间途经点更紧，确保吸取/释放位置精度。 */
#define ACT4_CHAIN_FINAL_BLEND_DIST_M    0.03f

/** @brief 到位判定：位置容差，单位 m */
#define ACT4_REACH_POS_TOL_M     0.03f
/** @brief 到位判定：pitch 容差，单位 rad */
#define ACT4_REACH_PITCH_TOL_RAD 0.05f
/** @brief 关节轨迹到位判定容差，单位 rad。TODO: 实测后调整。 */
#define ACT4_JOINT_REACH_TOL_RAD 0.05f

/** @brief 关节轨迹 blend 容差（运动链中间途经点用），单位 rad。比 reach 容差大以实现提前切换。 */
#define ACT4_JOINT_BLEND_TOL_RAD 0.15f

/** @brief 动作表总数（枚举最后一个有效动作 + 1）。 */
#define ACTION_4DOF_COUNT ((uint32_t)ACTION_DANCE + 1U)

typedef enum {
    ACTION_EXEC_MODE_POSE = 0,
    ACTION_EXEC_MODE_JOINT,
} Action4DOF_ExecMode;

#define ACT4_JOINT_TODO {0.0f, 0.0f, 0.0f, 0.0f}

/** @brief 左背占用时的默认左臂高位避让位姿，动作表可覆盖。 */
static const Dof4_Pose s_default_left_back_avoid_pose  = {0.24f,  0.13f, 0.16f, -0.30f};
/** @brief 右背占用时的默认右臂高位避让位姿，动作表可覆盖。 */
static const Dof4_Pose s_default_right_back_avoid_pose = {0.24f, -0.13f, 0.16f, -0.30f};
/** @brief 当前生效的左背占用避让位，放置到左背动作完成后更新。 */
static Dof4_Pose s_current_left_back_avoid_pose  = {0.24f,  0.13f, 0.22f, 0.20f};
/** @brief 当前生效的右背占用避让位，放置到右背动作完成后更新。 */
static Dof4_Pose s_current_right_back_avoid_pose = {0.24f, -0.13f, 0.22f, 0.20f};

/* ════════════════════════════════════════════════════════════════
 * 目标位姿数据结构
 *
 * 每个动作在不同子阶段需要不同的目标位姿。
 * 以下结构体封装了单臂在一个动作中的所有阶段目标。
 * ════════════════════════════════════════════════════════════════ */

/** @brief 单臂动作路径点集合 */
typedef struct {
    Dof4_Pose approach;       /**< 预就位点（抓取前/放置前的安全过渡位） */
    Dof4_Pose waypoint_0;     /**< 中间途经点 0（J1 绕行点，零值=不使用） */
    Dof4_Pose target;         /**< 目标点（抓取点 或 放置点） */
    Dof4_Pose retreat;        /**< 撤退点（抓取后/放置后携带物块的安全位） */
    Dof4_Pose complete;       /**< 完成后的归位点（回到 IDLE 时的位置） */
} Action4DOF_ArmTargets;

#define ACT4_POSE_ARM_ZERO \
    { \
        .approach = {0.0f, 0.0f, 0.0f, 0.0f}, \
        .waypoint_0 = {0.0f, 0.0f, 0.0f, 0.0f}, \
        .target = {0.0f, 0.0f, 0.0f, 0.0f}, \
        .retreat = {0.0f, 0.0f, 0.0f, 0.0f}, \
        .complete = {0.0f, 0.0f, 0.0f, 0.0f}, \
    }

/** @brief 关节轨迹点，占位角度单位 rad。舵机速度统一使用 arm->cfg.servo_speed。 */
typedef struct {
    float j1;
    float j2;
    float j3;
    float j4;
} JointWaypoint;

/** @brief 单臂后背动作关节轨迹点集合。 */
typedef struct {
    JointWaypoint approach;    /**< TODO: 后背动作预就位关节角。 */
    JointWaypoint waypoint_1;  /**< TODO: 后背动作途经点 1。 */
    JointWaypoint waypoint_2;  /**< TODO: 后背动作途经点 2。 */
    JointWaypoint target;      /**< TODO: 后背交接目标关节角。 */
    JointWaypoint retreat;     /**< TODO: 后背交接后撤退关节角。 */
    JointWaypoint complete;    /**< TODO: 后背动作完成关节角。 */
} Action4DOF_JointArmTargets;

#define ACT4_JOINT_ARM_TODO \
    { \
        .approach = ACT4_JOINT_TODO, \
        .waypoint_1 = ACT4_JOINT_TODO, \
        .waypoint_2 = ACT4_JOINT_TODO, \
        .target = ACT4_JOINT_TODO, \
        .retreat = ACT4_JOINT_TODO, \
        .complete = ACT4_JOINT_TODO, \
    }

/** @brief 后背动作关节轨迹数据。 */
typedef struct {
    Action4DOF_JointArmTargets left;
    Action4DOF_JointArmTargets right;
    bool use_left;
    bool use_right;
} Action4DOF_JointTargetData;

typedef enum {
    ACTION_4DOF_BACK_AVOID_NONE = 0,
    ACTION_4DOF_BACK_AVOID_SET,
    ACTION_4DOF_BACK_AVOID_CLEAR,
} Action4DOF_BackAvoidEffect;

/** @brief 一个动作的完整目标数据（左右臂各一份） */
typedef struct {
    Action4DOF_ArmTargets left;   /**< 左臂目标 */
    Action4DOF_ArmTargets right;  /**< 右臂目标 */
    bool use_left;                /**< 本动作是否使用左臂 */
    bool use_right;               /**< 本动作是否使用右臂 */
    bool use_waypoint;            /**< 是否在 approach 和 target 之间插入中间途经点 */
    Action4DOF_ExecMode exec_mode;/**< 运动执行模式：末端位姿或关节轨迹 */
    Action4DOF_BackAvoidEffect left_back_effect;
    Dof4_Pose left_back_avoid;
    Action4DOF_BackAvoidEffect right_back_effect;
    Dof4_Pose right_back_avoid;

    /* ── 多段轨迹平滑参数（可逐动作覆盖默认值）── */
    float blend_dist_m;           /**< 运动链中间途经点的前瞻切换距离，0=使用到位判定 */
    float via_speed_factor;       /**< 运动链中间途经点的通过速度因子，0=零速停止 */
} Action4DOF_TargetData;

/* ════════════════════════════════════════════════════════════════
 * 目标位姿数据库
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 重要：以下所有坐标均为占位值（TODO）！                             │
 * │ 请在硬件调试后替换为实际可到达的位姿。                             │
 * │                                                             │
 * │ 坐标格式: {x, y, z, pitch}  单位: m, rad                      │
 * │                                                             │
 * │ 命名规则:                                                     │
 * │   前侧(FORWARD)  = 机器前方 (+X 方向)                          │
 * │   后侧(BACKWARD) = 机器后方 (-X 方向，需 J1 旋转约 180°)        │
 * │   左放置点 = 机身左侧 (+Y) 的储物区                             │
 * │   右放置点 = 机身右侧 (-Y) 的储物区                             │
 * │                                                              │
 * │ pitch 含义: TCP 末端俯仰角。0=水平向前, -1.57=垂直向下            │
 * │                                                             │
 * │ 调试技巧:                                                     │
 * │   1. 先用上位机或临时调试命令将臂移到期望位置                     │
 * │   2. 读取 current_pose 作为目标值填入                          │
 * │   3. 微调后重新编译测试                                        │
 * └──────────────────────────────────────────────────────────────┘
 *
 * s_action_targets 数组索引 = action_state_4dof_e 枚举值
 * ════════════════════════════════════════════════════════════════ */

static const Action4DOF_TargetData s_action_targets[] = {

    /* ────────────────────────────────────────────────────────────
     * [0] ACTION_4DOF_IDLE — 无动作（保留，不使用）
     * ──────────────────────────────────────────────────────────── */
    {{{0}}, {{0}}, false, false},

    /* ────────────────────────────────────────────────────────────
     * [1] ACTION_BLOCK_GET_FORWARD — 前侧物块同时抓取（双臂）
     *
     * 流程: 双臂从归位 → 移动到物块上方(APPROACH) → 下降到物块(GRAB)
     *       → 吸取(SUCTION_ON) → 抬升撤退(RETREAT) → 归位(COMPLETE)
     * ──────────────────────────────────────────────────────────── */
    {
        /*---------------------------调试完成---------------------------*/
        /* 左臂 */
        .left = {
            .approach = {BLOCK_X_DESTANCE_FROM_BASE + 0.1f, BLOCK_Y_DESTANCE_FROM_BASE_PICK - 0.15f, BLOCK_FIRST_LAYER_HEIGHT + 0.3f, -0.6f},   /* 前侧预就位 */
            .target   = {BLOCK_X_DESTANCE_FROM_BASE, BLOCK_Y_DESTANCE_FROM_BASE_PICK, BLOCK_FIRST_LAYER_HEIGHT + 0.02f, STAY_DOWN}, /* 前侧抓取点 */
            .retreat  = {BLOCK_X_DESTANCE_FROM_BASE + 0.1f, BLOCK_Y_DESTANCE_FROM_BASE_PLACE - 0.15f, 0.175f, -0.30f},   /* 抓取后撤退 */
            .complete = {0.06f,  0.00f, 0.30f, STAY_LEVEL},   /* 归位 = startup */
        },
        /* 右臂 */
        .right = {
            .approach = {BLOCK_X_DESTANCE_FROM_BASE + 0.1f, -(BLOCK_Y_DESTANCE_FROM_BASE_PICK - 0.15f), BLOCK_FIRST_LAYER_HEIGHT + 0.3f, -0.60f},   /* 前侧预就位 */
            .target   = {BLOCK_X_DESTANCE_FROM_BASE, -BLOCK_Y_DESTANCE_FROM_BASE_PICK, BLOCK_FIRST_LAYER_HEIGHT, STAY_DOWN}, /* 前侧抓取点 */
            .retreat  = {BLOCK_X_DESTANCE_FROM_BASE + 0.1f, -(BLOCK_Y_DESTANCE_FROM_BASE_PICK - 0.15f), 0.175f, -0.30f},   /* 抓取后撤退 */
            .complete = {0.06f,  0.00f, 0.30f, STAY_LEVEL},   /* 归位 = startup */
        },
        .use_left  = true,
        .use_right = true,
    },

    /* ────────────────────────────────────────────────────────────
     * [2] ACTION_BLOCK_GET_FORWARD_LEFT_ARM — 前侧物块左臂单独抓取
     *
     * 流程: 仅左臂移动，右臂保持原位不动
     * ──────────────────────────────────────────────────────────── */
    {
        /*---------------------------调试完成---------------------------*/
        .left = {
            .approach = {BLOCK_X_DESTANCE_FROM_BASE - 0.1f, BLOCK_Y_DESTANCE_FROM_BASE_PICK - 0.15f, BLOCK_FIRST_LAYER_HEIGHT + 0.4f, -0.60f},   /* 前侧预就位 */
            .target   = {BLOCK_X_DESTANCE_FROM_BASE, BLOCK_Y_DESTANCE_FROM_BASE_PICK, BLOCK_FIRST_LAYER_HEIGHT + 0.02f, STAY_DOWN}, /* 前侧抓取点 */
            .retreat  = {BLOCK_X_DESTANCE_FROM_BASE - 0.1f, BLOCK_Y_DESTANCE_FROM_BASE_PLACE - 0.15f, BLOCK_FIRST_LAYER_HEIGHT + 0.4f, -0.30f},   /* 抓取后撤退 */
            .complete = {0.06f,  0.00f, 0.30f, STAY_LEVEL},   /* 归位 = startup */
        },
        .right     = ACT4_POSE_ARM_ZERO,   /* 右臂不使用，保持原位 */
        .use_left  = true,
        .use_right = false,
    },

    /* ────────────────────────────────────────────────────────────
     * [3] ACTION_BLOCK_GET_FORWARD_RIGHT_ARM — 前侧物块右臂单独抓取
     *
     * 流程: 仅右臂移动，左臂保持原位不动
     * ──────────────────────────────────────────────────────────── */
    {
        /*---------------------------调试完成---------------------------*/
        .left      = ACT4_POSE_ARM_ZERO,   /* 左臂不使用 */
        .right = {
            .approach = {BLOCK_X_DESTANCE_FROM_BASE + 0.1f, -(BLOCK_Y_DESTANCE_FROM_BASE_PICK - 0.15f), BLOCK_FIRST_LAYER_HEIGHT + 0.4f, -0.60f},   /* 前侧预就位 */
            .target   = {BLOCK_X_DESTANCE_FROM_BASE, -BLOCK_Y_DESTANCE_FROM_BASE_PICK, BLOCK_FIRST_LAYER_HEIGHT , STAY_DOWN}, /* 前侧抓取点 */
            .retreat  = {BLOCK_X_DESTANCE_FROM_BASE + 0.1f, -(BLOCK_Y_DESTANCE_FROM_BASE_PICK - 0.15f), BLOCK_FIRST_LAYER_HEIGHT + 0.4f, -0.30f},   /* 抓取后撤退 */
            .complete = {0.06f,  0.00f, 0.30f, STAY_LEVEL},   /* 归位 = startup */
        },
        .use_left  = false,
        .use_right = true,
    },

    /* ────────────────────────────────────────────────────────────
     * [4] ACTION_BLOCK_PLACE_BACK — 双臂同时放置物块到对应后背
     *
     * 运动: 使用 s_back_joint_targets[] 中的关节轨迹，不使用 Dof4_Pose/IK。
     * 流程: 双臂同时携物块 → 放置预就位 → 途经点绕行 → 放置点
     *       → 预释放等待(背部吸附) → 关阀 → 释放后等待 → 撤退 → 归位
     * 左臂→左背，右臂→右背，同时执行。
     * ──────────────────────────────────────────────────────────── */
    {
        //ACTION_BLOCK_PLACE_BACK
        .left = ACT4_POSE_ARM_ZERO,
        .right = ACT4_POSE_ARM_ZERO,
        .use_left  = true,
        .use_right = true,
        .use_waypoint = true,
        .exec_mode = ACTION_EXEC_MODE_JOINT,
        .left_back_effect = ACTION_4DOF_BACK_AVOID_SET,
        .left_back_avoid  = {0.24f, 0.18f, 0.22f, 0.20f},
        .right_back_effect = ACTION_4DOF_BACK_AVOID_SET,
        .right_back_avoid  = {0.24f, -0.18f, 0.22f, 0.20f},
    },

    /* ────────────────────────────────────────────────────────────
     * [5] ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_BACK
     *     左臂放置物块到左背（机身左侧后方的储物区）
     *
     * 运动: 使用 s_back_joint_targets[] 中的关节轨迹，不使用 Dof4_Pose/IK。
     * 流程: 左臂携物块 → 放置预就位(PLACE_APPROACH) → 放置点(PLACE)
     *       → 预释放等待(背吸吸附) → 关阀 → 释放后等待 → 撤退(RETREAT) → 归位(COMPLETE)
     * 右臂保持原位。
     * ──────────────────────────────────────────────────────────── */
    {
        //ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_BACK
        .left = ACT4_POSE_ARM_ZERO,
        .right = ACT4_POSE_ARM_ZERO,
        .use_left  = true,
        .use_right = false,
        .use_waypoint = true,
        .exec_mode = ACTION_EXEC_MODE_JOINT,
        .left_back_effect = ACTION_4DOF_BACK_AVOID_SET,
        .left_back_avoid  = {0.28f, 0.18f, 0.22f, 0.20f}
    },

    /* ────────────────────────────────────────────────────────────
     * [6] ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_BACK
     *     右臂放置物块到右背（机身右侧后方的储物区）
     *
     * 运动: 使用 s_back_joint_targets[] 中的关节轨迹，不使用 Dof4_Pose/IK。
     * 流程: 右臂携物块 → 放置预就位(PLACE_APPROACH) → 途经点绕行 → 放置点(PLACE)
     *       → 预释放等待(背吸吸附) → 关阀 → 释放后等待 → 撤退(RETREAT) → 归位(COMPLETE)
     * 左臂保持原位。
     * ──────────────────────────────────────────────────────────── */
    {
        //ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_BACK
        .left = ACT4_POSE_ARM_ZERO,
        .right = ACT4_POSE_ARM_ZERO,
        .use_left  = false,
        .use_right = true,
        .use_waypoint = true,
        .exec_mode = ACTION_EXEC_MODE_JOINT,
        .right_back_effect = ACTION_4DOF_BACK_AVOID_SET,
        .right_back_avoid  = {0.28f, -0.18f, 0.22f, 0.20f}
    },

    /* ────────────────────────────────────────────────────────────
     * [7] ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F1
     *      左臂放置物块到左1放置点第一层
     *
     * "左1放置点" 是机身上一个固定的堆叠放置位。
     * F1 = 第1层（底部），F2 = 第2层（堆叠在上方）。
     *
     * 注意: pitch 可能需要更接近垂直向下（-1.57 rad）以便精确放置。
     * ──────────────────────────────────────────────────────────── */
    {
        //ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F1
        .left = {
            .approach = {0.35f, 0.2f, 0.30f, -0.30f},   /* 后侧预就位 */
            .target   = {0.425f, 0.40f, BLOCK_FIRST_LAYER_HEIGHT + 0.02 , STAY_DOWN}, /* 后侧抓取点 ⚠️ J1限位 */
            .retreat  = {0.25f, 0.16f, 0.25f, -0.50f},   /* 抓取后撤退 */
            .complete = {0.02f,  0.00f, 0.20f, -0.02f},
        },
        .right     = ACT4_POSE_ARM_ZERO,
        .use_left  = true,
        .use_right = false,
        .left_back_effect = ACTION_4DOF_BACK_AVOID_NONE,  /* 外部放置，不修改后背避让状态 */
    },

    /* ────────────────────────────────────────────────────────────
     * [8] ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F2
     *      左臂放置物块到左1放置点第二层（堆叠）
     *
     * F2 的 Z 坐标应比 F1 高约一个物块厚度（~0.04m）。
     * ──────────────────────────────────────────────────────────── */
    {
        //ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F2
        .left = {
            .approach = {0.212f,  0.35f, 0.30f, -0.30f},   /* TODO: 左1放置点F2预就位 */
            .target   = {0.425f,  0.40f, BLOCK_SECOND_LAYER_HEIGHT+ 0.02f, STAY_DOWN},   /* TODO: 左1放置点F2（比F1高~0.04m） */
            .retreat  = {0.212f,  0.308f, 0.15f, -0.30f},   /* TODO */
            .complete = {0.02f,  0.00f, 0.20f, -0.02f},
        },
        .right     = ACT4_POSE_ARM_ZERO,
        .use_left  = true,
        .use_right = false,
        .left_back_effect = ACTION_4DOF_BACK_AVOID_NONE,  /* 外部放置，不修改后背避让状态 */
    },

    /* ────────────────────────────────────────────────────────────
     * [9] ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F1
     *      右臂放置物块到右1放置点第一层
     * ──────────────────────────────────────────────────────────── */
    {
        //ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F1
        .left      = ACT4_POSE_ARM_ZERO,
        .right = {
            .approach = {0.35f, -0.35f, 0.34f, -1.00f},   /* TODO */
            .target   = {0.425f, -0.45f, BLOCK_FIRST_LAYER_HEIGHT + 0.03f, STAY_DOWN},   /* TODO: 比F1高~0.04m */
            .retreat  = {0.25f, -0.30f, 0.25f, -0.50f},   /* TODO */
            .complete = {0.02f,  -0.00f, 0.20f, -0.02f},
        },
        .use_left  = false,
        .use_right = true ,
        .right_back_effect = ACTION_4DOF_BACK_AVOID_NONE,  /* 外部放置，不修改后背避让状态 */
    },

    /* ────────────────────────────────────────────────────────────
     * [10] ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F2
     *      右臂放置物块到右1放置点第二层（堆叠）
     * ──────────────────────────────────────────────────────────── */
    {
        //ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F2
        .left      = ACT4_POSE_ARM_ZERO,
        .right = {
            .approach = {0.35f, -0.35f, 0.34f, -1.00f},   /* TODO */
            .target   = {0.425f, -0.45f, BLOCK_SECOND_LAYER_HEIGHT+ 0.03f, STAY_DOWN},   /* TODO: 比F1高~0.04m */
            .retreat  = {0.25f, -0.30f, 0.25f, -0.50f},   /* TODO */
            .complete = {0.02f,  0.00f, 0.20f, -0.02f},
        },
        .use_left  = false,
        .use_right = true,
        .right_back_effect = ACTION_4DOF_BACK_AVOID_NONE,  /* 外部放置，不修改后背避让状态 */
    },

    /* ────────────────────────────────────────────────────────────
     * [11] ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_LEFT_ARM
     *      左臂从左背抓取物块到手
     *
     * 运动: 使用 s_back_joint_targets[] 中的关节轨迹，不使用 Dof4_Pose/IK。
     * 流程: 左臂 → 左背预就位 → 途经点绕行 → 抓取点 → 吸取 → 撤退 → 归位
     * 抓取后左背储物区清空。
     * ──────────────────────────────────────────────────────────── */
    {
        .left = ACT4_POSE_ARM_ZERO,
        .right = ACT4_POSE_ARM_ZERO,
        .use_left  = true,
        .use_right = false,
        .use_waypoint = true,
        .exec_mode = ACTION_EXEC_MODE_JOINT,
        .left_back_effect = ACTION_4DOF_BACK_AVOID_CLEAR,
    },

    /* ────────────────────────────────────────────────────────────
     * [12] ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_RIGHT_ARM
     *      右臂从右背抓取物块到手
     *
     * 运动: 使用 s_back_joint_targets[] 中的关节轨迹，不使用 Dof4_Pose/IK。
     * 流程: 右臂 → 右背预就位 → 途经点绕行 → 抓取点 → 吸取 → 撤退 → 归位
     * 抓取后右背储物区清空。
     * ──────────────────────────────────────────────────────────── */
    {
        .left = ACT4_POSE_ARM_ZERO,
        .right = ACT4_POSE_ARM_ZERO,
        .use_left  = false,
        .use_right = true,
        .use_waypoint = true,
        .exec_mode = ACTION_EXEC_MODE_JOINT,
        .right_back_effect = ACTION_4DOF_BACK_AVOID_CLEAR,
    },

    /* ────────────────────────────────────────────────────────────
     * [13] ACTION_DANCE — 神秘舞蹈动作
     *
     * 多途经点序列动作。DANCE 不使用固定的 approach/target/retreat 字段，
     * 而是通过 WAYPOINT 子状态 + s_dance_waypoints[] 数组依次访问各途经点。
     *
     * 途经点数量由 DANCE_WAYPOINT_COUNT 定义（见下方）。
     * ──────────────────────────────────────────────────────────── */
    {
        .left      = ACT4_POSE_ARM_ZERO,   /* DANCE 使用 s_dance_waypoints，不使用此结构 */
        .right     = ACT4_POSE_ARM_ZERO,
        .use_left  = true,
        .use_right = true,
    },
};

/* 后背动作关节轨迹表。所有角度均为 TODO 占位值，调试时逐点替换。 */
static const Action4DOF_JointTargetData s_back_joint_targets[ACTION_4DOF_COUNT] = {
    //同时放置到背部的动作使用以下轨迹，单臂放置到背部的动作使用对应臂的轨迹，其他动作不使用关节轨迹。
    [ACTION_BLOCK_PLACE_BACK] = {
        .left = {
            .approach =   {-0.042f, 1.5f, -1.7f, -1.463f},
            .waypoint_1 = {1.57f, 1.5f, -1.7f, -1.463f},
            .waypoint_2 = {3.1f, 1.5f, -1.19f, -1.68f},
            .target =     {3.04f, 1.57f, -1.88f, -1.247f},
            .retreat =    {3.1f, 1.5f, -1.19f, -1.68f},
            .complete =   {-0.042f, 1.5f, -1.7f, -1.463f},
        },
        .right = {
            .approach =   {0.042f, 1.5f, -1.7f, -1.463f},
            .waypoint_1 = {-1.57f, 1.5f, -1.7f, -1.463f},
            .waypoint_2 = {-3.1f, 1.5f, -1.19f, -1.68f},
            .target =     {-3.04f, 1.57f, -1.88f, -1.247f},
            .retreat =    {-3.1f, 1.5f, -1.19f, -1.68f},
            .complete =   {0.042f, 1.5f, -1.7f, -1.463f},
        },
        .use_left = true,
        .use_right = true,
    },
    //单臂放置到背部的动作使用以下轨迹，左->左背，右->右背，另一臂不使用轨迹保持原位。
    [ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_BACK] = {
        .left = {
            .approach =   {-0.042f, 1.5f, -1.7f, -1.463f},
            .waypoint_1 = {1.57f, 1.5f, -1.7f, -1.463f},
            .waypoint_2 = {2.8f, 1.5f, -1.19f, -1.68f},
            .target =     {2.8f, 1.57f, -1.88f, -1.247f},//在此之后还需要一个中间点，不然容易打到
            .retreat =    {2.1f, 1.5f, -1.19f, -1.68f},
            .complete =   {-0.042f, 1.5f, -1.7f, -1.463f},
        },
        .use_left = true,
        .use_right = false,
    },

    [ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_BACK] = {
        .right = {
        .approach =   {0.042f,1.5f,-1.7f,-1.463f},
        .waypoint_1 = {-1.57f,1.5f,-1.7f,-1.463f}, 
        .waypoint_2 = {-2.8f,1.5f,-1.0f,-1.68f}, 
        .target =     {-2.8f,1.57f,-1.88f,-1.247f}, 
        .retreat =    {-2.0f,1.5f,-1.80f,-1.68f}, 
        .complete =   {0.042f,1.5f,-1.7f,-1.463f}, 
    },
        .use_left = false,
        .use_right = true,
    },
    //从背部抓取到手的动作使用以下轨迹，左背->左手，右背->右手，另一臂不使用轨迹保持原位。
    [ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_LEFT_ARM] = {
        .left = {
            .approach =   {3.1f, 1.5f, -1.19f, -1.68f},
            .waypoint_1 = {3.1f, 1.5f, -1.19f, -1.68f},
            .waypoint_2 = {1.57f, 1.5f, -1.7f, -1.463f},
            .target =     {3.04f, 1.57f, -1.88f, -1.247f},
            .retreat =    {-0.042f, 1.5f, -1.7f, -1.463f},
            .complete =   {-0.042f, 1.5f, -1.7f, -1.463f},
        },
        .use_left = true,
        .use_right = false,
    },
    [ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_RIGHT_ARM] = {
        .right = {
        .approach =   {-1.57f, 1.5f, -1.19f, -1.68f},
        .waypoint_1 = {-2.0f,1.5f,-1.0f,-1.463f}, 
        .waypoint_2 = {-2.8f,1.5f,-1.0f,-1.68f}, 
        .target =     {-2.8f,1.57f,-1.88f,-1.247f}, 
        .retreat =    {-1.57f,1.5f,-1.80f,-1.68f}, 
            .complete =   {0.042f, 1.5f, -1.7f, -1.463f},
        },
        .use_left = false,
        .use_right = true,
    },
};

/* ════════════════════════════════════════════════════════════════
 * DANCE 动作途经点序列
 *
 * DANCE 动作通过 WAYPOINT 子状态依次访问以下位姿。
 * 每个途经点停留 ACT4_WAYPOINT_HOLD_MS 后自动切换到下一个。
 * 左右臂可分别设置不同途经点，实现不对称舞蹈动作。
 *
 * 增加途经点: 直接在数组中追加新元素即可，状态机会自动推进。
 * ════════════════════════════════════════════════════════════════ */

/** @brief DANCE 单步途经点（左右臂各一个位姿） */
typedef struct {
    Dof4_Pose left;    /**< 左臂目标位姿 */
    Dof4_Pose right;   /**< 右臂目标位姿 */
} DanceWaypoint;

#define DANCE_WAYPOINT_COUNT 4U  /**< DANCE 途经点总数 */

static const DanceWaypoint s_dance_waypoints[DANCE_WAYPOINT_COUNT] = {

    /* 途经点 0 — 双臂向两侧展开 */
    {
        .left  = { 0.20f,  0.35f, 0.20f,  0.00f},   /* TODO: 左臂展开 */
        .right = { 0.20f, -0.35f, 0.20f,  0.00f},   /* TODO: 右臂展开 */
    },

    /* 途经点 1 — 双臂上举 */
    {
        .left  = { 0.05f,  0.15f, 0.45f,  1.00f},   /* TODO: 左臂上举 */
        .right = { 0.05f, -0.15f, 0.45f,  1.00f},   /* TODO: 右臂上举 */
    },

    /* 途经点 2 — 双臂交叉 */
    {
        .left  = { 0.15f, -0.15f, 0.30f,  0.00f},   /* TODO: 左臂交叉到右侧 */
        .right = { 0.15f,  0.15f, 0.30f,  0.00f},   /* TODO: 右臂交叉到左侧 */
    },

    /* 途经点 3 — 双臂归位姿态 */
    {
        .left  = { 0.02f,  0.00f, 0.20f, -0.02f},   /* TODO: 归位 */
        .right = { 0.02f,  0.00f, 0.20f, -0.02f},   /* TODO: 归位 */
    },
    /* 添加更多途经点只需在此数组末尾追加即可 */};

/* ════════════════════════════════════════════════════════════════
 * 内部状态机上下文
 * ════════════════════════════════════════════════════════════════ */

typedef struct {
    action_state_4dof_e     action;         /**< 当前执行的动作 */
    action_4dof_substate_e  substate;       /**< 当前子状态 */
    uint32_t                enter_tick;     /**< 进入当前子状态的系统 tick */
    uint32_t                timeout_ms;     /**< 当前子状态的超时时间 */
    uint8_t                 waypoint_idx;   /**< 当前途经点索引（DANCE 用） */
    volatile bool           active;         /**< 是否有动作正在执行（跨任务读取） */
} Action4DOF_Ctx;

static Action4DOF_Ctx s_ctx;

/* ════════════════════════════════════════════════════════════════
 * 内部辅助函数
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 切换到新子状态，记录进入时刻和超时时间
 * @param new_state  目标子状态
 * @param timeout_ms 该子状态的超时时间 (ms)，0=永不超时
 */
static void action_4dof_set_substate(action_4dof_substate_e new_state, uint32_t timeout_ms)
{
    s_ctx.substate   = new_state;
    s_ctx.enter_tick = HAL_GetTick();
    s_ctx.timeout_ms = timeout_ms;
}

/**
 * @brief 检查当前子状态是否超时
 * @retval true   已超时
 * @retval false  未超时 或 无超时设置 (timeout_ms==0)
 */
static bool action_4dof_is_timed_out(void)
{
    if (s_ctx.timeout_ms == 0U) {
        return false;
    }
    return ((HAL_GetTick() - s_ctx.enter_tick) >= s_ctx.timeout_ms);
}

/**
 * @brief 设置单臂目标位姿（带工作空间安全检查）
 * @param arm    机械臂实例指针
 * @param pose   目标位姿
 * @param label  调试标签（预留，当前未使用）
 */
static void action_4dof_set_arm_target(Dof4_Arm *arm, const Dof4_Pose *pose,
                                       const char *label)
{
    if (arm == NULL || pose == NULL) {
        return;
    }
    (void)label;
    (void)Dof4_arm_set_target(arm, pose->x, pose->y, pose->z, pose->pitch);
}

/**
 * @brief 判断一个 4DOF 动作是否属于放置类动作。
 *
 * 放置类动作从 PLACE_APPROACH 子状态开始，避让臂使用放置/背部高位避让点；
 * 抓取类动作从 APPROACH 子状态开始，避让臂使用抓取避让点。
 *
 * @param action 需要判断的动作枚举值。
 * @retval true  动作为 ACTION_BLOCK_PLACE_* 范围内的放置动作。
 * @retval false 动作为抓取、DANCE、IDLE 或非法范围动作。
 */
static bool action_4dof_is_place_action(action_state_4dof_e action)
{
    return (action >= ACTION_BLOCK_PLACE_BACK &&
            action <= ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F2);
}

static bool action_4dof_is_get_action(action_state_4dof_e action)
{
    return ((action >= ACTION_BLOCK_GET_FORWARD &&
             action <= ACTION_BLOCK_GET_FORWARD_RIGHT_ARM) ||
            (action >= ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_LEFT_ARM &&
             action <= ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_RIGHT_ARM));
}

static bool action_4dof_is_back_get_action(action_state_4dof_e action)
{
    return (action >= ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_LEFT_ARM &&
            action <= ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_RIGHT_ARM);
}

/**
 * @brief 判断一个放置类动作是否为后背放置（放到背部储物区）。
 *
 * 后背放置（索引 4-6）需要将物块交接给背部吸盘，时序为：
 *   到达 → 预释放等待(背部吸附) → 关手臂阀 → 释放后等待(交接稳定) → 撤退
 *
 * 外部放置（索引 7-10）物块靠重力落在机体外堆叠点，时序为：
 *   到达 → 关手臂阀 → 释放后等待(物块落下稳定) → 撤退
 *
 * @param action 需要判断的动作枚举值。
 * @retval true  动作为 ACTION_BLOCK_PLACE_BACK / _TO_LEFT_BACK 范围内的后背放置动作。
 * @retval false 动作为外部放置、抓取、DANCE、IDLE 或非法动作。
 */
static bool action_4dof_is_back_place_action(action_state_4dof_e action)
{
    return (action >= ACTION_BLOCK_PLACE_BACK &&
            action <= ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_BACK);
}

static void action_4dof_control_arm_suction(const Action4DOF_TargetData *td, uint8_t state)
{
    if (td == NULL) {
        return;
    }

    if (td->use_left) {
        relay_control(RELAY_LEFT_ARM, state);
    }
    if (td->use_right) {
        relay_control(RELAY_RIGHT_ARM, state);
    }
}

static void action_4dof_open_target_back_suction(const Action4DOF_TargetData *td)
{
    if (td == NULL) {
        return;
    }

    if (td->left_back_effect == ACTION_4DOF_BACK_AVOID_SET) {
        relay_control(RELAY_LEFT_BACK, SUCTION_ON);
    }
    if (td->right_back_effect == ACTION_4DOF_BACK_AVOID_SET) {
        relay_control(RELAY_RIGHT_BACK, SUCTION_ON);
    }
}

static void action_4dof_close_source_back_suction(const Action4DOF_TargetData *td)
{
    if (td == NULL) {
        return;
    }

    if (td->left_back_effect == ACTION_4DOF_BACK_AVOID_CLEAR) {
        relay_control(RELAY_LEFT_BACK, SUCTION_OFF);
    }
    if (td->right_back_effect == ACTION_4DOF_BACK_AVOID_CLEAR) {
        relay_control(RELAY_RIGHT_BACK, SUCTION_OFF);
    }
}

static void action_4dof_set_arm_target_smooth(Dof4_Arm *arm,
                                              const Action4DOF_TargetData *td,
                                              action_4dof_substate_e substate,
                                              bool is_place,
                                              char arm_side,
                                              const Dof4_Pose *work_pose,
                                              const char *label);

/**
 * @brief 给当前阶段同时下发工作臂目标和非工作臂避让目标。
 *
 * 该函数是单臂动作防干涉的核心入口：
 * - 工作臂使用 left_work_pose/right_work_pose 中对应侧目标；
 * - 非工作臂根据当前动作自动选择抓取避让位或放置避让位；
 * - 双臂动作中两臂都是工作臂，不会额外套用避让位；
 * - DANCE 使用单独 waypoint 流程，不调用本函数。
 *
 * 内部根据运动链状态自动选择 via-point（平滑通过）或 stop-point（精确停止）模式。
 *
 * @param td 当前动作目标数据，包含工作臂选择标志。
 * @param action 当前动作枚举，用于选择避让位类型。
 * @param left_work_pose 左臂作为工作臂时本阶段应到达的位姿。
 * @param right_work_pose 右臂作为工作臂时本阶段应到达的位姿。
 * @param left_label 左臂调试标签，当前仅保留给调试输出扩展。
 * @param right_label 右臂调试标签，当前仅保留给调试输出扩展。
 */
static void action_4dof_set_stage_targets(const Action4DOF_TargetData *td,
                                          action_state_4dof_e action,
                                          const Dof4_Pose *left_work_pose,
                                          const Dof4_Pose *right_work_pose,
                                          const char *left_label,
                                          const char *right_label)
{
    if (td == NULL) {
        return;
    }

    const bool is_place = action_4dof_is_place_action(action);

    if (td->use_left) {
        action_4dof_set_arm_target_smooth(&g_dof4_arm_left, td, s_ctx.substate,
                                          is_place, 'L', left_work_pose, left_label);
    }

    if (td->use_right) {
        action_4dof_set_arm_target_smooth(&g_dof4_arm_right, td, s_ctx.substate,
                                          is_place, 'R', right_work_pose, right_label);
    }
}

/**
 * @brief 判断当前子状态是否为运动链的终点（需精确停止执行吸取/释放）。
 *
 * 运动链终点定义为：该子状态之后紧跟吸盘操作子状态或动作完成。
 * 对于终点，使用零终端速度 + 原有到位判定；对于中间途经点，使用
 * 非零通过速度 + 前瞻切换距离实现平滑衔接。
 *
 * @param substate 当前子状态。
 * @param is_place 是否为放置类动作（影响 INTERMEDIATE 后的目标判断）。
 * @retval true  当前子状态是运动链终点，应精确停止。
 * @retval false 当前子状态是运动链中间途经点，可平滑通过。
 */
static bool action_4dof_is_motion_chain_final(action_4dof_substate_e substate,
                                              bool is_place)
{
    switch (substate) {
    case ACTION_4DOF_SUBSTATE_GRAB:    /* GRAB 后进入 SUCTION_CONTROL_GRAB */
    case ACTION_4DOF_SUBSTATE_PLACE:   /* PLACE 后进入 SUCTION_CONTROL_PLACE */
    case ACTION_4DOF_SUBSTATE_COMPLETE:/* COMPLETE 后动作结束回到 IDLE */
        return true;

    case ACTION_4DOF_SUBSTATE_APPROACH:
    case ACTION_4DOF_SUBSTATE_PLACE_APPROACH:
    case ACTION_4DOF_SUBSTATE_RETREAT:
        return false;  /* 这些子状态后紧跟另一个运动子状态 */

    case ACTION_4DOF_SUBSTATE_INTERMEDIATE:
        /* INTERMEDIATE 后进入 GRAB（抓取）或 PLACE（放置），它们本身就是终点 */
        return false;

    default:
        return true;   /* 未知子状态保守按终点处理 */
    }
}

/**
 * @brief 获取运动链中当前子状态的“下一站”目标位姿，用于计算通过速度。
 *
 * 通过速度方向 = 下一站 - 当前站 的单位方向向量。
 *
 * @param td 当前动作目标数据。
 * @param substate 当前子状态。
 * @param is_place 是否为放置类动作。
 * @param arm_side 'L'=左臂, 'R'=右臂。
 * @param next_pose 输出下一站目标位姿。
 * @retval true  成功获取下一站目标。
 * @retval false 无法获取（如当前已是终点或数据无效）。
 */
static bool action_4dof_get_next_target_pose(const Action4DOF_TargetData *td,
                                             action_4dof_substate_e substate,
                                             bool is_place,
                                             char arm_side,
                                             Dof4_Pose *next_pose)
{
    if (td == NULL || next_pose == NULL) {
        return false;
    }

    const Action4DOF_ArmTargets *arm = (arm_side == 'L') ? &td->left : &td->right;

    switch (substate) {
    case ACTION_4DOF_SUBSTATE_APPROACH:
    case ACTION_4DOF_SUBSTATE_PLACE_APPROACH:
        /* 下一站 = waypoint_0（如果有）或 target */
        if (td->use_waypoint) {
            *next_pose = arm->waypoint_0;
        } else {
            *next_pose = arm->target;
        }
        return true;

    case ACTION_4DOF_SUBSTATE_INTERMEDIATE:
        /* 下一站 = target（GRAB 或 PLACE） */
        *next_pose = arm->target;
        return true;

    case ACTION_4DOF_SUBSTATE_RETREAT:
        /* 下一站 = complete（但 complete 是动态 idle，用 retreat 本身方向延续） */
        *next_pose = arm->complete;
        return true;

    default:
        return false;
    }
}

/**
 * @brief 设置单臂目标位姿，根据是否为运动链终点选择 via-point 或 stop-point 模式。
 *
 * @param arm 机械臂实例。
 * @param td 当前动作目标数据。
 * @param substate 当前子状态。
 * @param is_place 是否为放置类动作。
 * @param arm_side 'L'=左臂, 'R'=右臂。
 * @param work_pose 本阶段目标位姿。
 * @param label 调试标签。
 */
static void action_4dof_set_arm_target_smooth(Dof4_Arm *arm,
                                              const Action4DOF_TargetData *td,
                                              action_4dof_substate_e substate,
                                              bool is_place,
                                              char arm_side,
                                              const Dof4_Pose *work_pose,
                                              const char *label)
{
    if (arm == NULL || td == NULL || work_pose == NULL) {
        return;
    }
    (void)label;

    const bool is_final = action_4dof_is_motion_chain_final(substate, is_place);
    const float speed_factor = (td->via_speed_factor > 0.0f)
                               ? td->via_speed_factor
                               : ACT4_DEFAULT_VIA_SPEED_FACTOR;

    if (!is_final && speed_factor > 0.0f) {
        /* 运动链中间途经点：使用非零终端速度平滑通过 */
        Dof4_Pose next_pose;
        float via_vel[4] = {0.0f, 0.0f, 0.0f, 0.0f};

        if (action_4dof_get_next_target_pose(td, substate, is_place, arm_side, &next_pose)) {
            Dof4_compute_via_velocity(work_pose, &next_pose, speed_factor,
                                      arm->cfg.cart_vel_mps,
                                      arm->cfg.pitch_vel_rps,
                                      via_vel);
        }

        (void)Dof4_arm_set_target_via(arm, work_pose->x, work_pose->y,
                                      work_pose->z, work_pose->pitch, via_vel);
    } else {
        /* 运动链终点 或 零速因子：精确停止 */
        (void)Dof4_arm_set_target(arm, work_pose->x, work_pose->y,
                                  work_pose->z, work_pose->pitch);
    }
}

/**
 * @brief 获取当前子状态的前瞻切换距离。
 *
 * 运动链中间途经点使用较大的 blend_distance 实现提前切换；
 * 运动链终点使用较紧的 chain_final 距离确保操作精度。
 *
 * @param td 当前动作目标数据。
 * @param substate 当前子状态。
 * @param is_place 是否为放置类动作。
 * @retval float 前瞻切换距离，单位 m。
 */
static float action_4dof_get_blend_dist(const Action4DOF_TargetData *td,
                                        action_4dof_substate_e substate,
                                        bool is_place)
{
    const bool is_final = action_4dof_is_motion_chain_final(substate, is_place);

    if (is_final) {
        return ACT4_CHAIN_FINAL_BLEND_DIST_M;
    }

    return (td->blend_dist_m > 0.0f) ? td->blend_dist_m : ACT4_DEFAULT_BLEND_DIST_M;
}

/**
 * @brief 检查单臂末端是否在目标位姿的 blend 距离内。
 * @param arm 机械臂实例。
 * @param target 目标位姿。
 * @param blend_dist_m blend 距离，单位 m。
 * @retval true  当前末端距目标 <= blend_dist_m。
 * @retval false 未进入 blend 范围或参数无效。
 */
static bool action_4dof_arm_within_blend(const Dof4_Arm *arm,
                                         const Dof4_Pose *target,
                                         float blend_dist_m)
{
    if (arm == NULL || target == NULL) {
        return true;  /* 无效参数视为已到位，避免卡死 */
    }
    const float dx = arm->current_pose.x - target->x;
    const float dy = arm->current_pose.y - target->y;
    const float dz = arm->current_pose.z - target->z;
    const float dist = sqrtf(dx * dx + dy * dy + dz * dz);
    return (dist <= blend_dist_m);
}

/**
 * @brief COMPLETE 阶段将本动作涉及的机械臂送回自适应 idle 位。
 *
 * 单臂动作中，工作臂和避让臂都会回到 `action_4dof_get_idle_pose()` 返回的位置。
 * 如果某侧背部已经有物块，该侧 idle 会自动变成对应的高位避让点。
 *
 * @param td 当前动作目标数据，包含工作臂和避让臂判定所需标志。
 */
static void action_4dof_set_complete_targets(const Action4DOF_TargetData *td)
{
    if (td == NULL) {
        return;
    }

    if (td->use_left || td->left_back_effect != ACTION_4DOF_BACK_AVOID_NONE) {
        Dof4_Pose idle_left = action_4dof_get_idle_pose(DOF4_ARM_LEFT);
        action_4dof_set_arm_target(&g_dof4_arm_left, &idle_left, "L_cplt");
    }
    if (td->use_right || td->right_back_effect != ACTION_4DOF_BACK_AVOID_NONE) {
        Dof4_Pose idle_right = action_4dof_get_idle_pose(DOF4_ARM_RIGHT);
        action_4dof_set_arm_target(&g_dof4_arm_right, &idle_right, "R_cplt");
    }
}

/**
 * @brief 检查指定吸盘的微动开关是否确认吸附到物块。
 * @param arm_side 吸盘侧编号，0=左臂吸盘，1=右臂吸盘。
 * @retval true  微动开关已触发，认为物块已吸附。
 * @retval false 微动开关未触发、编号非法，或未检测到吸附。
 */
static void action_4dof_apply_back_avoid_effects(const Action4DOF_TargetData *td)
{
    if (td == NULL) {
        return;
    }

    if (td->left_back_effect == ACTION_4DOF_BACK_AVOID_SET) {
        g_block_state.left_back = true;
        s_current_left_back_avoid_pose = td->left_back_avoid;
    } else if (td->left_back_effect == ACTION_4DOF_BACK_AVOID_CLEAR) {
        g_block_state.left_back = false;
    }

    if (td->right_back_effect == ACTION_4DOF_BACK_AVOID_SET) {
        g_block_state.right_back = true;
        s_current_right_back_avoid_pose = td->right_back_avoid;
    } else if (td->right_back_effect == ACTION_4DOF_BACK_AVOID_CLEAR) {
        g_block_state.right_back = false;
    }
}

/**
 * @brief 检查单臂 FK 位姿是否已到达目标（位置 + pitch 均在容差内）
 * @param arm    机械臂实例
 * @param target 目标位姿
 * @retval true  已到达
 */
static bool action_4dof_arm_reached_pose(const Dof4_Arm *arm, const Dof4_Pose *target)
{
    if (arm == NULL || target == NULL) {
        return true;
    }
    const float dx = arm->current_pose.x - target->x;
    const float dy = arm->current_pose.y - target->y;
    const float dz = arm->current_pose.z - target->z;
    const float pos_err = sqrtf(dx * dx + dy * dy + dz * dz);
    const float pitch_err = fabsf(arm->current_pose.pitch - target->pitch);
    return (pos_err <= ACT4_REACH_POS_TOL_M) && (pitch_err <= ACT4_REACH_PITCH_TOL_RAD);
}

static void action_4dof_set_joint_arm_target(Dof4_Arm *arm, const JointWaypoint *waypoint)
{
    if (arm == NULL || waypoint == NULL) {
        return;
    }

    Dof4_JointState joints = {{
        waypoint->j1,
        waypoint->j2,
        waypoint->j3,
        waypoint->j4,
    }};
    (void)Dof4_arm_set_joint_target(arm, &joints);
}

static void action_4dof_set_joint_stage_targets(const Action4DOF_JointTargetData *jd,
                                                const JointWaypoint *left_waypoint,
                                                const JointWaypoint *right_waypoint)
{
    if (jd == NULL) {
        return;
    }
    if (jd->use_left) {
        action_4dof_set_joint_arm_target(&g_dof4_arm_left, left_waypoint);
    }
    if (jd->use_right) {
        action_4dof_set_joint_arm_target(&g_dof4_arm_right, right_waypoint);
    }
}

static bool action_4dof_joint_reached(const Dof4_Arm *arm, const JointWaypoint *waypoint)
{
    if (arm == NULL || waypoint == NULL) {
        return true;
    }

    const float target[DOF4_JOINT_COUNT] = {
        waypoint->j1,
        waypoint->j2,
        waypoint->j3,
        waypoint->j4,
    };
    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        const float err = fabsf(Dof4_normalize_angle(arm->joint_actual.q[i] - target[i]));
        if (err > ACT4_JOINT_REACH_TOL_RAD) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 检查单臂关节角是否在目标关节的 blend 容差内（用于运动链中间途经点提前切换）。
 * @param arm 机械臂实例。
 * @param waypoint 目标关节途经点。
 * @retval true  所有关节角距目标 <= ACT4_JOINT_BLEND_TOL_RAD。
 * @retval false 未进入 blend 范围。
 */
static bool action_4dof_joint_within_blend(const Dof4_Arm *arm,
                                           const JointWaypoint *waypoint)
{
    if (arm == NULL || waypoint == NULL) {
        return true;
    }

    const float target[DOF4_JOINT_COUNT] = {
        waypoint->j1,
        waypoint->j2,
        waypoint->j3,
        waypoint->j4,
    };
    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        const float err = fabsf(Dof4_normalize_angle(arm->joint_actual.q[i] - target[i]));
        if (err > ACT4_JOINT_BLEND_TOL_RAD) {
            return false;
        }
    }
    return true;
}

static bool action_4dof_joint_stage_reached(const Action4DOF_JointTargetData *jd,
                                            const JointWaypoint *left_waypoint,
                                            const JointWaypoint *right_waypoint)
{
    if (jd == NULL) {
        return true;
    }

    bool left_ok = !jd->use_left ||
                   action_4dof_joint_reached(&g_dof4_arm_left, left_waypoint);
    bool right_ok = !jd->use_right ||
                    action_4dof_joint_reached(&g_dof4_arm_right, right_waypoint);
    return left_ok && right_ok;
}

/**
 * @brief 检查双臂关节角是否在目标关节的 blend 容差内。
 * @param jd 关节目标数据。
 * @param left_waypoint 左臂目标。
 * @param right_waypoint 右臂目标。
 * @retval true  双臂均在 blend 容差内。
 */
static bool action_4dof_joint_stage_within_blend(const Action4DOF_JointTargetData *jd,
                                                  const JointWaypoint *left_waypoint,
                                                  const JointWaypoint *right_waypoint)
{
    if (jd == NULL) {
        return true;
    }

    bool left_ok = !jd->use_left ||
                   action_4dof_joint_within_blend(&g_dof4_arm_left, left_waypoint);
    bool right_ok = !jd->use_right ||
                    action_4dof_joint_within_blend(&g_dof4_arm_right, right_waypoint);
    return left_ok && right_ok;
}

static void action_4dof_finish_current_action(const Action4DOF_TargetData *td)
{
    action_4dof_control_arm_suction(td, SUCTION_ON);
    s_ctx.active = false;
    s_ctx.action = ACTION_4DOF_IDLE;
    s_ctx.substate = ACTION_4DOF_SUBSTATE_IDLE;
    s_ctx.timeout_ms = 0U;
    s_ctx.waypoint_idx = 0U;
}

static void action_4dof_handle_joint(const Action4DOF_TargetData *td,
                                     const Action4DOF_JointTargetData *jd)
{
    if (td == NULL || jd == NULL || (!jd->use_left && !jd->use_right)) {
        action_4dof_abort();
        return;
    }

    switch (s_ctx.substate) {
    case ACTION_4DOF_SUBSTATE_APPROACH:
    case ACTION_4DOF_SUBSTATE_PLACE_APPROACH:
        action_4dof_set_joint_stage_targets(jd,
                                            &jd->left.approach,
                                            &jd->right.approach);
        if (action_4dof_joint_stage_within_blend(jd,
                                                 &jd->left.approach,
                                                 &jd->right.approach) ||
            action_4dof_is_timed_out()) {
            s_ctx.waypoint_idx = 0U;
            action_4dof_set_substate(ACTION_4DOF_SUBSTATE_INTERMEDIATE,
                                      ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    case ACTION_4DOF_SUBSTATE_INTERMEDIATE:
    {
        const JointWaypoint *left_wp = (s_ctx.waypoint_idx == 0U)
                                       ? &jd->left.waypoint_1
                                       : &jd->left.waypoint_2;
        const JointWaypoint *right_wp = (s_ctx.waypoint_idx == 0U)
                                        ? &jd->right.waypoint_1
                                        : &jd->right.waypoint_2;
        action_4dof_set_joint_stage_targets(jd, left_wp, right_wp);
        if (action_4dof_joint_stage_within_blend(jd, left_wp, right_wp) ||
            action_4dof_is_timed_out()) {
            if (s_ctx.waypoint_idx == 0U) {
                s_ctx.waypoint_idx = 1U;
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_INTERMEDIATE,
                                          ACT4_MOVE_TIMEOUT_MS);
            } else if (action_4dof_is_place_action(s_ctx.action)) {
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_PLACE,
                                          ACT4_MOVE_TIMEOUT_MS);
            } else {
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_GRAB,
                                          ACT4_MOVE_TIMEOUT_MS);
            }
        }
        break;
    }

    case ACTION_4DOF_SUBSTATE_GRAB:
        action_4dof_set_joint_stage_targets(jd, &jd->left.target, &jd->right.target);
        if (action_4dof_joint_stage_reached(jd, &jd->left.target, &jd->right.target) ||
            action_4dof_is_timed_out()) {
            uint32_t hold_ms = action_4dof_is_back_get_action(s_ctx.action) ?
                               ACT4_PLACE_HOLD_MS : ACT4_SUCTION_TIMEOUT_MS;
            action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_GRAB,
                                      hold_ms);
        }
        break;

    case ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_GRAB:
        action_4dof_set_joint_stage_targets(jd, &jd->left.target, &jd->right.target);
        if (action_4dof_is_timed_out()) {
            if (action_4dof_is_back_get_action(s_ctx.action) &&
                s_ctx.timeout_ms != ACT4_BACK_RELEASE_HOLD_MS) {
                action_4dof_close_source_back_suction(td);
                action_4dof_apply_back_avoid_effects(td);
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_GRAB,
                                          ACT4_BACK_RELEASE_HOLD_MS);
            } else {
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_RETREAT,
                                          ACT4_MOVE_TIMEOUT_MS);
            }
        }
        break;

    case ACTION_4DOF_SUBSTATE_PLACE:
        action_4dof_set_joint_stage_targets(jd, &jd->left.target, &jd->right.target);
        if (action_4dof_joint_stage_reached(jd, &jd->left.target, &jd->right.target)) {
            if (s_ctx.timeout_ms != ACT4_PLACE_PRE_RELEASE_BACK_MS) {
                action_4dof_open_target_back_suction(td);
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_PLACE,
                                          ACT4_PLACE_PRE_RELEASE_BACK_MS);
            } else if (action_4dof_is_timed_out()) {
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_PLACE, 0U);
            }
        } else if (action_4dof_is_timed_out()) {
            action_4dof_set_substate(ACTION_4DOF_SUBSTATE_RETREAT,
                                      ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    case ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_PLACE:
        action_4dof_set_joint_stage_targets(jd, &jd->left.target, &jd->right.target);
        if (s_ctx.timeout_ms == 0U) {
            action_4dof_control_arm_suction(td, SUCTION_OFF);
            action_4dof_apply_back_avoid_effects(td);
            action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_PLACE,
                                      ACT4_PLACE_POST_RELEASE_BACK_MS);
        } else if (action_4dof_is_timed_out()) {
            action_4dof_set_substate(ACTION_4DOF_SUBSTATE_RETREAT,
                                      ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    case ACTION_4DOF_SUBSTATE_RETREAT:
        action_4dof_set_joint_stage_targets(jd, &jd->left.retreat, &jd->right.retreat);
        if (action_4dof_joint_stage_within_blend(jd, &jd->left.retreat, &jd->right.retreat) ||
            action_4dof_is_timed_out()) {
            action_4dof_set_substate(ACTION_4DOF_SUBSTATE_COMPLETE,
                                      ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    case ACTION_4DOF_SUBSTATE_COMPLETE:
        /* 两阶段：先达关节 complete 途经点，再切 POSE 模式回自适应 idle */
        if (s_ctx.timeout_ms == ACT4_MOVE_TIMEOUT_MS) {
            /* 阶段①：移动到关节 complete 途经点 */
            action_4dof_set_joint_stage_targets(jd, &jd->left.complete, &jd->right.complete);
            if (action_4dof_joint_stage_reached(jd, &jd->left.complete, &jd->right.complete) ||
                action_4dof_is_timed_out()) {
                /* 关节到位 → 切换到 POSE 模式，驱动机械臂回到自适应 idle 位姿 */
                action_4dof_set_complete_targets(td);
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_COMPLETE, ACT4_HOLD_MS);
            }
        } else {
            /* 阶段②：保持在 POSE idle 目标，等待短暂稳定后结束 */
            action_4dof_set_complete_targets(td);
            if (action_4dof_is_timed_out()) {
                action_4dof_finish_current_action(td);
            }
        }
        break;

    case ACTION_4DOF_SUBSTATE_IDLE:
    case ACTION_4DOF_SUBSTATE_WAYPOINT:
    default:
        break;
    }
}

/* ════════════════════════════════════════════════════════════════
 * 动作处理核心 — 状态机推进
 *
 * 根据 s_ctx.action 和 s_ctx.substate 执行对应的目标设定和条件判断。
 * 所有超时均为容错设计——超时后推进而非卡死。
 * ════════════════════════════════════════════════════════════════ */

static void action_4dof_handle(void)
{
    /* 边界检查 */
    if ((uint32_t)s_ctx.action >= ACTION_4DOF_COUNT) {
        action_4dof_abort();
        return;
    }

    const Action4DOF_TargetData *td = &s_action_targets[s_ctx.action];
    if (td->exec_mode == ACTION_EXEC_MODE_JOINT) {
        action_4dof_handle_joint(td, &s_back_joint_targets[s_ctx.action]);
        return;
    }

    /* 根据子状态执行对应阶段 */
    switch (s_ctx.substate) {

    /* ═══════════════════════════════════════════════════════════
     * 子状态: APPROACH — 移动到预就位点
     *
     * 运动链中间途经点：使用 blend_distance 提前切换，via_vel 平滑通过。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_APPROACH:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.approach, &td->right.approach,
                                      "L_ap", "R_ap");

        {
            const float blend = action_4dof_get_blend_dist(td, s_ctx.substate,
                                   action_4dof_is_place_action(s_ctx.action));
            bool left_ok  = !td->use_left  ||
                            action_4dof_arm_within_blend(&g_dof4_arm_left,  &td->left.approach, blend);
            bool right_ok = !td->use_right ||
                            action_4dof_arm_within_blend(&g_dof4_arm_right, &td->right.approach, blend);
            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                if (td->use_waypoint) {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_INTERMEDIATE, ACT4_MOVE_TIMEOUT_MS);
                }
                else {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_GRAB, ACT4_MOVE_TIMEOUT_MS);
                }
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: INTERMEDIATE — 中间途经点（J1 绕行安全位）
     *
     * 运动链中间途经点：使用 blend_distance 提前切换，via_vel 平滑通过。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_INTERMEDIATE:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.waypoint_0, &td->right.waypoint_0,
                                      "L_wp0", "R_wp0");

        {
            const float blend = action_4dof_get_blend_dist(td, s_ctx.substate,
                                   action_4dof_is_place_action(s_ctx.action));
            bool left_ok  = !td->use_left  ||
                            action_4dof_arm_within_blend(&g_dof4_arm_left,  &td->left.waypoint_0, blend);
            bool right_ok = !td->use_right ||
                            action_4dof_arm_within_blend(&g_dof4_arm_right, &td->right.waypoint_0, blend);
            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                if (action_4dof_is_place_action(s_ctx.action)) {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_PLACE, ACT4_MOVE_TIMEOUT_MS);
                }
                else {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_GRAB, ACT4_MOVE_TIMEOUT_MS);
                }
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: GRAB — 移动到抓取点
     *
     * 从预就位下降到物块位置。到位后（或超时容错）进入吸附保持/背部交接等待。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_GRAB:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.target, &td->right.target,
                                      "L_grab", "R_grab");

        {
            bool left_ok  = !td->use_left  || action_4dof_arm_reached_pose(&g_dof4_arm_left,  &td->left.target);
            bool right_ok = !td->use_right || action_4dof_arm_reached_pose(&g_dof4_arm_right, &td->right.target);
            if ((left_ok && right_ok) || action_4dof_is_timed_out())
            {
                uint32_t hold_ms = action_4dof_is_back_get_action(s_ctx.action) ?
                                   ACT4_PLACE_HOLD_MS : ACT4_SUCTION_TIMEOUT_MS;
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_GRAB, hold_ms);
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: SUCTION_CONTROL_GRAB — 控制吸盘吸附
     *
     * 手臂吸盘已在动作触发时开启；此处仅定时等待。
     * 背部取回动作先关闭对应背部吸盘，再等待背部释放稳定后撤退。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_GRAB:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.target, &td->right.target,
                                      "L_suck", "R_suck");

        if (action_4dof_is_timed_out()) {
            if (action_4dof_is_back_get_action(s_ctx.action) &&
                s_ctx.timeout_ms != ACT4_BACK_RELEASE_HOLD_MS) {
                action_4dof_close_source_back_suction(td);
                action_4dof_apply_back_avoid_effects(td);
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_GRAB,
                                          ACT4_BACK_RELEASE_HOLD_MS);
            } else {
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_RETREAT, ACT4_MOVE_TIMEOUT_MS);
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: RETREAT — 携带物块撤退到安全高度
     *
     * 运动链中间途经点：使用 blend_distance 提前切换到 COMPLETE。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_RETREAT:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.retreat, &td->right.retreat,
                                      "L_ret", "R_ret");

        {
            const float blend = action_4dof_get_blend_dist(td, s_ctx.substate,
                                   action_4dof_is_place_action(s_ctx.action));
            bool left_ok  = !td->use_left  ||
                            action_4dof_arm_within_blend(&g_dof4_arm_left,  &td->left.retreat, blend);
            bool right_ok = !td->use_right ||
                            action_4dof_arm_within_blend(&g_dof4_arm_right, &td->right.retreat, blend);
            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_COMPLETE, ACT4_HOLD_MS);
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: PLACE_APPROACH — 移动到放置预就位点
     *
     * 运动链中间途经点：使用 blend_distance 提前切换。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_PLACE_APPROACH:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.approach, &td->right.approach,
                                      "L_pap", "R_pap");

        {
            const float blend = action_4dof_get_blend_dist(td, s_ctx.substate,
                                   action_4dof_is_place_action(s_ctx.action));
            bool left_ok  = !td->use_left  ||
                            action_4dof_arm_within_blend(&g_dof4_arm_left,  &td->left.approach, blend);
            bool right_ok = !td->use_right ||
                            action_4dof_arm_within_blend(&g_dof4_arm_right, &td->right.approach, blend);
            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                if (td->use_waypoint) {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_INTERMEDIATE, ACT4_MOVE_TIMEOUT_MS);
                } else {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_PLACE, ACT4_MOVE_TIMEOUT_MS);
                }
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: PLACE — 移动到放置点，区分后背/外部放置时序
     *
     * ┌─────────────────────────────────────────────────────────┐
     * │ 后背放置（放到背部储物区，索引 4-7）:                       │
     * │   阶段① 移动到位（timeout = ACT4_MOVE_TIMEOUT_MS）        │
     * │   阶段② 到达 → 开背部吸盘 → 预释放等待                     │
     * │          （timeout = ACT4_PLACE_PRE_RELEASE_BACK_MS）     │
     * │          → 超时后进入 SUCTION_CONTROL_PLACE 关阀+释放后等待 │
     * │                                                         │
     * │ 外部放置（放到机体外堆叠点，索引 8-11）:                    │
     * │   阶段① 移动到位（timeout = ACT4_MOVE_TIMEOUT_MS）        │
     * │   阶段② 到达 → 直接进入 SUCTION_CONTROL_PLACE 关阀+释放后等待│
     * │          （无需预释放，无背部吸盘参与）                      │
     * └─────────────────────────────────────────────────────────┘
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_PLACE:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.target, &td->right.target,
                                      "L_place", "R_place");

        {
            bool left_ok  = !td->use_left  || action_4dof_arm_reached_pose(&g_dof4_arm_left,  &td->left.target);
            bool right_ok = !td->use_right || action_4dof_arm_reached_pose(&g_dof4_arm_right, &td->right.target);
            bool is_back_place = action_4dof_is_back_place_action(s_ctx.action);

            if (left_ok && right_ok) {
                if (is_back_place) {
                    /* ── 后背放置：到达后先开背部吸盘，再等待预释放时间 ── */
                    if (s_ctx.timeout_ms != ACT4_PLACE_PRE_RELEASE_BACK_MS) {
                        /* 首次到达 → 打开背部吸盘，启动预释放等待 */
                        action_4dof_open_target_back_suction(td);
                        action_4dof_set_substate(ACTION_4DOF_SUBSTATE_PLACE,
                                                  ACT4_PLACE_PRE_RELEASE_BACK_MS);
                    } else if (action_4dof_is_timed_out()) {
                        /* 预释放等待到期 → 进入关阀+释放后等待阶段 */
                        action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_PLACE, 0U);
                    }
                } else {
                    /* ── 外部放置：到达后直接关阀+释放后等待，无预释放阶段 ── */
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_PLACE, 0U);
                }
            } else if (action_4dof_is_timed_out()) {
                /* 移动超时容错：未到达但已超时，直接撤退避免卡死 */
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_RETREAT, ACT4_MOVE_TIMEOUT_MS);
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: SUCTION_CONTROL_PLACE — 关闭手臂电磁阀 + 释放后等待
     *
     * 两阶段设计（通过 timeout_ms 区分，复用同一子状态枚举）:
     *   阶段① 首次进入（timeout_ms == 0）:
     *           关闭手臂电磁阀（通大气，释放物块）
     *           应用背部状态变更（SET/CLEAR g_block_state）
     *           根据动作类型设置释放后等待时间：
     *             - 后背放置 → ACT4_PLACE_POST_RELEASE_BACK_MS（物块交接稳定）
     *             - 外部放置 → ACT4_PLACE_POST_RELEASE_EXT_MS（物块落下稳定）
     *   阶段② 等待中（timeout_ms > 0）:
     *           保持目标位姿不动，等待物块稳定
     *           超时后 → RETREAT（撤退到安全位）
     *
     * 注意：此阶段不控制背部吸盘（背部吸盘在 PLACE 阶段已打开，
     *       在 COMPLETE/IDLE 阶段由 g_block_state 维持状态）。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_PLACE:
        /* 保持目标位姿不动，避免机械臂晃动影响物块释放 */
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.target, &td->right.target,
                                      "L_rel", "R_rel");

        if (s_ctx.timeout_ms == 0U) {
            /* ── 阶段①：首次进入，执行关阀操作并启动释放后等待 ── */
            action_4dof_control_arm_suction(td, SUCTION_OFF);
            action_4dof_apply_back_avoid_effects(td);

            /* 根据放置类型选择不同的释放后等待时间 */
            uint32_t post_wait_ms = action_4dof_is_back_place_action(s_ctx.action)
                                    ? ACT4_PLACE_POST_RELEASE_BACK_MS
                                    : ACT4_PLACE_POST_RELEASE_EXT_MS;
            action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_CONTROL_PLACE,
                                      post_wait_ms);
        } else if (action_4dof_is_timed_out()) {
            /* ── 阶段②：释放后等待到期 → 撤退到安全位 ── */
            action_4dof_set_substate(ACTION_4DOF_SUBSTATE_RETREAT, ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: COMPLETE — 动作完成，归位后回到 IDLE
     *
     * 移动到自适应 IDLE 归位点，短暂保持后：
     *   1. 显式打开本动作涉及的手臂电磁阀（防真空泵憋压）
     *   2. 清空调度器上下文，回到 IDLE 空闲状态
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_COMPLETE:
        /* 使用全局物块状态驱动的自适应 IDLE 位姿 */
        action_4dof_set_complete_targets(td);

        if (action_4dof_is_timed_out()) {
            action_4dof_finish_current_action(td);
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: WAYPOINT — DANCE 途经点推进
     *
     * 依次访问 s_dance_waypoints[]，每个途经点停留固定时间。
     * 全部访问完毕后进入 COMPLETE。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_WAYPOINT:
        if (s_ctx.waypoint_idx < DANCE_WAYPOINT_COUNT) {
            const DanceWaypoint *wp = &s_dance_waypoints[s_ctx.waypoint_idx];
            if (td->use_left)  action_4dof_set_arm_target(&g_dof4_arm_left,  &wp->left,  "L_dance");
            if (td->use_right) action_4dof_set_arm_target(&g_dof4_arm_right, &wp->right, "R_dance");

            if (action_4dof_is_timed_out()) {
                s_ctx.waypoint_idx++;
                if (s_ctx.waypoint_idx < DANCE_WAYPOINT_COUNT) {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_WAYPOINT, ACT4_WAYPOINT_HOLD_MS);
                }
                else {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_COMPLETE, ACT4_HOLD_MS);
                }
            }
        } else {
            /* 防御: waypoint_idx 越界 → 完成 */
            action_4dof_set_substate(ACTION_4DOF_SUBSTATE_COMPLETE, ACT4_HOLD_MS);
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: IDLE — 无动作
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_IDLE:
    default:
        break;
    }
}

/* ════════════════════════════════════════════════════════════════
 * 自适应 IDLE 位姿计算
 *
 * 根据全局 g_block_state 返回当前物块分布下的安全归位点。
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 获取指定臂的自适应 IDLE 位姿（世界坐标系，基于 effective_base 动态计算）。
 *
 * IDLE 位姿 = effective_base + (IDLE_BASE_X, IDLE_BASE_Y, IDLE_BASE_Z)。
 * effective_base = cfg.base + cfg.base_offset，因此 base_offset 变化时
 * IDLE 归位点自动跟随基座移动。
 * 若对应侧背部已有物块，则不使用普通 idle，而返回该侧的放置/背部高位避让点，
 * 避免机械臂回位时扫到 0.25 m 立方体物块。
 *
 * @param arm_id 机械臂 ID，DOF4_ARM_LEFT 或 DOF4_ARM_RIGHT。
 * @retval Dof4_Pose 自适应归位位姿，世界坐标，单位 m/rad。
 */
Dof4_Pose action_4dof_get_idle_pose(Dof4_ArmId arm_id)
{
    if (arm_id == DOF4_ARM_LEFT && g_block_state.left_back) {
        return s_current_left_back_avoid_pose;
    }
    if (arm_id == DOF4_ARM_RIGHT && g_block_state.right_back) {
        return s_current_right_back_avoid_pose;
    }

    const Dof4_Arm *arm = (arm_id == DOF4_ARM_LEFT) ? &g_dof4_arm_left : &g_dof4_arm_right;
    const float base_x = arm->cfg.base[0] + arm->cfg.base_offset[0];
    const float base_y = arm->cfg.base[1] + arm->cfg.base_offset[1];
    const float base_z = arm->cfg.base[2] + arm->cfg.base_offset[2];

    Dof4_Pose pose;
    pose.x     = base_x + IDLE_BASE_X;
    pose.y     = base_y + IDLE_BASE_Y;
    pose.z     = base_z + IDLE_BASE_Z;
    pose.pitch = IDLE_BASE_PITCH;

    return pose;
}


/* ════════════════════════════════════════════════════════════════
 * 公共 API 实现
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 初始化 4DOF 动作调度器。
 *
 * 将内部动作上下文清零并回到 ACTION_4DOF_IDLE，同时清空全局物块位置状态
 * `g_block_state`。通常在 4DOF 双臂初始化完成后调用一次。
 *
 * @param none
 * @return none
 */
void action_4dof_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.action   = ACTION_4DOF_IDLE;
    s_ctx.substate = ACTION_4DOF_SUBSTATE_IDLE;
    s_ctx.active   = false;

    /* 复位全局物块位置状态 */
    memset(&g_block_state, 0, sizeof(g_block_state));
    s_current_left_back_avoid_pose = s_default_left_back_avoid_pose;
    s_current_right_back_avoid_pose = s_default_right_back_avoid_pose;

    /* 初始化后 IDLE 状态：双臂电磁阀常开，防止真空泵憋压 */
    relay_control(RELAY_LEFT_ARM,  SUCTION_ON);
    relay_control(RELAY_RIGHT_ARM, SUCTION_ON);
    relay_control(RELAY_LEFT_BACK,  SUCTION_ON);
    relay_control(RELAY_RIGHT_BACK, SUCTION_ON);
}

/**
 * @brief 触发一个 4DOF 预设动作。
 *
 * 仅当当前没有动作运行时才会接受新动作。动作被接受后，本函数只设置状态机
 * 起始子状态，真正的目标下发、吸盘控制和超时推进由后续周期性调用
 * `action_4dof_loop()` 完成。
 *
 * @param action 需要触发的动作枚举，必须大于 ACTION_4DOF_IDLE 且小于动作表长度。
 * @retval true  动作被接受，状态机已进入对应起始子状态。
 * @retval false 当前已有动作运行，或 action 非法。
 */
static bool action_4dof_trigger_internal(action_state_4dof_e action)
{
    /* 检查 action 是否合法 */
    if (action <= ACTION_4DOF_IDLE || (uint32_t)action >= ACTION_4DOF_COUNT) {
        return false;
    }

    /*
     * PC 接收任务和预设动作触发可能在不同 RTOS 任务中同时发生。
     * 在短临界区内同时检查两个状态机并占用 4DOF 动作执行权，确保只接受一方。
     */
    taskENTER_CRITICAL();
    if (s_ctx.active || pc_action_4dof_is_active()) {
        taskEXIT_CRITICAL();
        return false;
    }
    s_ctx.action       = action;
    s_ctx.active       = true;
    s_ctx.waypoint_idx = 0U;
    taskEXIT_CRITICAL();

    const Action4DOF_TargetData *td = &s_action_targets[action];
    if (action_4dof_is_get_action(action) || action_4dof_is_place_action(action)) {
        action_4dof_control_arm_suction(td, SUCTION_ON);
    }

    /* 根据动作类型选择起始子状态 */
    if (action_4dof_is_place_action(action)) {
        /* 放置类动作: 从 PLACE_APPROACH 开始 */
        action_4dof_set_substate(ACTION_4DOF_SUBSTATE_PLACE_APPROACH, ACT4_MOVE_TIMEOUT_MS);
    } else if (action == ACTION_DANCE) {
        /* DANCE: 从 WAYPOINT 开始 */
        action_4dof_set_substate(ACTION_4DOF_SUBSTATE_WAYPOINT, ACT4_WAYPOINT_HOLD_MS);
    } else {
        /* 抓取类动作: 从 APPROACH 开始 */
        action_4dof_set_substate(ACTION_4DOF_SUBSTATE_APPROACH, ACT4_MOVE_TIMEOUT_MS);
    }

    return true;
}

bool action_4dof_trigger(action_state_4dof_e action)
{
    return action_4dof_trigger_internal(action);
}




/**
 * @brief 强制中止当前 4DOF 动作。
 *
 * 立即关闭左右吸盘，清空动作上下文并回到 IDLE。该函数不会主动把机械臂移动到
 * idle 位姿，只停止调度器继续覆盖目标；后续目标由手动控制或下一次动作决定。
 *
 * @param none
 * @return none
 */
void action_4dof_abort(void)
{
    /* 关闭所有吸盘 */
    relay_control(RELAY_LEFT_ARM,  SUCTION_OFF);
    relay_control(RELAY_RIGHT_ARM, SUCTION_OFF);
    relay_control(RELAY_LEFT_BACK,  SUCTION_OFF);
    relay_control(RELAY_RIGHT_BACK, SUCTION_OFF);

    /* 复位状态机 */
    s_ctx.action       = ACTION_4DOF_IDLE;
    s_ctx.substate     = ACTION_4DOF_SUBSTATE_IDLE;
    s_ctx.active       = false;
    s_ctx.timeout_ms   = 0U;
    s_ctx.waypoint_idx = 0U;
}

/**
 * @brief 查询调度器是否正在执行动作。
 * @retval true  当前有动作激活，`action_4dof_loop()` 会持续覆盖目标位姿。
 * @retval false 当前空闲，`action_4dof_loop()` 不会影响手动控制目标。
 */
bool action_4dof_is_active(void)
{
    return s_ctx.active;
}

/**
 * @brief 4DOF 动作调度器周期函数。
 *
 * 应在控制任务循环中周期性调用。若动作激活，本函数根据当前子状态：
 * 1. 下发工作臂目标位姿；
 * 2. 对单臂动作下发非工作臂避让位姿；
 * 3. 控制吸盘开关；
 * 4. 根据超时或传感器状态推进状态机。
 *
 * 若当前空闲，本函数立即返回，不覆盖手动/上位机控制目标。
 *
 * @param none
 * @return none
 */
void action_4dof_loop(void)
{
    if (!s_ctx.active) {
        return;   /* 无动作，不干预手动控制 */
    }

    /* 推进状态机 */
    action_4dof_handle();
}

void action_4dof_set_back_occupied(Dof4_ArmId arm_id, bool occupied)
{
    if (arm_id == DOF4_ARM_LEFT) {
        g_block_state.left_back = occupied;
        if (occupied) {
            s_current_left_back_avoid_pose = s_default_left_back_avoid_pose;
        }
    } else if (arm_id == DOF4_ARM_RIGHT) {
        g_block_state.right_back = occupied;
        if (occupied) {
            s_current_right_back_avoid_pose = s_default_right_back_avoid_pose;
        }
    }
}
