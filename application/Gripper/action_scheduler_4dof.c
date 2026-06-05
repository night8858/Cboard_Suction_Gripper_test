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
 *   IDLE → PLACE_APPROACH → PLACE → SUCTION_OFF → RETREAT → COMPLETE → IDLE
 *
 * DANCE:
 *   IDLE → WAYPOINT(0) → WAYPOINT(1) → ... → COMPLETE → IDLE
 *
 * ## 超时策略
 *
 * - 移动子状态（APPROACH/GRAB/RETREAT/PLACE_*）：固定延时，后续可改为末端到位判断
 * - 吸附子状态（SUCTION_ON）：等待微动开关触发，超时后容错推进
 * - 释放子状态（SUCTION_OFF）：固定延时 + 微动开关确认脱离
 * - 每个子状态有独立超时，超时后记录错误并推进（不卡死）
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
#include "block_inspect.h"
#include "stm32f4xx_hal.h"

#include <string.h>
#include <math.h>


/// ════════════════════════════════════════════════════════════════
//特殊角度
#define STAY_LEVEL 0.0f
#define STAY_DOWN  -1.45f

#define BLOCK_GET_DOWN_Z -0.22f    //TODO: 实测调整z的下降目标高度，确保能贴近物块但不碰撞

/* ════════════════════════════════════════════════════════════════
 * 外部引用
 * ════════════════════════════════════════════════════════════════ */

extern Dof4_Arm g_dof4_arm_left;
extern Dof4_Arm g_dof4_arm_right;
extern SwitchInput g_switch_input;  /* 微动开关状态: state[0]=左臂, state[1]=右臂 */

/** @brief 全局物块位置追踪状态 */
BlockPlacementState g_block_state;

/* ════════════════════════════════════════════════════════════════
 * 吸盘/电磁阀映射
 *
 * relay_control(id, state):  id=0→左臂吸盘, id=1→右臂吸盘
 *                           state=1→开启(吸取), state=0→关闭(释放)
 *
 * 微动开关: g_switch_input.state[0]=左臂吸附状态, [1]=右臂吸附状态
 *           0=未吸附, 1=已吸附
 * ════════════════════════════════════════════════════════════════ */
#define RELAY_LEFT_ARM    0U
#define RELAY_RIGHT_ARM   1U
#define SUCTION_ON        1
#define SUCTION_OFF       0

/* ════════════════════════════════════════════════════════════════
 * 时机参数（单位 ms，后续根据实测调整）
 * ════════════════════════════════════════════════════════════════ */

#define ACT4_MOVE_TIMEOUT_MS       3000U   /**< 单段移动最大超时（容错兜底） */
#define ACT4_SUCTION_TIMEOUT_MS    600U   /**< 吸附等待最大超时 */
#define ACT4_RELEASE_TIMEOUT_MS    600U   /**< 释放等待最大超时 */
#define ACT4_HOLD_MS                100U   /**< 完成后保持时间（回 IDLE 前） */
#define ACT4_WAYPOINT_HOLD_MS       400U   /**< 途经点停留时间（DANCE 用） */

/** @brief 到位判定：位置容差，单位 m */
#define ACT4_REACH_POS_TOL_M     0.03f
/** @brief 到位判定：pitch 容差，单位 rad */
#define ACT4_REACH_PITCH_TOL_RAD 0.05f

/** @brief 左背占用时的默认左臂高位避让位姿，动作表可覆盖。 */
static const Dof4_Pose s_default_left_back_avoid_pose  = {0.24f,  0.13f, 0.16f, -0.30f};
/** @brief 右背占用时的默认右臂高位避让位姿，动作表可覆盖。 */
static const Dof4_Pose s_default_right_back_avoid_pose = {0.24f, -0.13f, 0.16f, -0.30f};
/** @brief 当前生效的左背占用避让位，放置到左背动作完成后更新。 */
static Dof4_Pose s_current_left_back_avoid_pose  = {0.24f,  0.13f, 0.16f, -0.30f};
/** @brief 当前生效的右背占用避让位，放置到右背动作完成后更新。 */
static Dof4_Pose s_current_right_back_avoid_pose = {0.24f, -0.13f, 0.16f, -0.30f};

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
    Action4DOF_BackAvoidEffect left_back_effect;
    Dof4_Pose left_back_avoid;
    Action4DOF_BackAvoidEffect right_back_effect;
    Dof4_Pose right_back_avoid;
} Action4DOF_TargetData;

/* ════════════════════════════════════════════════════════════════
 * 目标位姿数据库
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 重要：以下所有坐标均为占位值（TODO）！                         │
 * │ 请在硬件调试后替换为实际可到达的位姿。                         │
 * │                                                              │
 * │ 坐标格式: {x, y, z, pitch}  单位: m, rad                     │
 * │                                                              │
 * │ 命名规则:                                                     │
 * │   前侧(FORWARD)  = 机器前方 (+X 方向)                          │
 * │   后侧(BACKWARD) = 机器后方 (-X 方向，需 J1 旋转约 180°)       │
 * │   左放置点 = 机身左侧 (+Y) 的储物区                            │
 * │   右放置点 = 机身右侧 (-Y) 的储物区                            │
 * │                                                              │
 * │ pitch 含义: TCP 末端俯仰角。0=水平向前, -1.57=垂直向下         │
 * │                                                              │
 * │ 调试技巧:                                                     │
 * │   1. 先用 RC 手动将臂移到期望位置                              │
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
        /* 左臂 */
        .left = {
            .approach = {0.355f, 0.361f, 0.125f, -0.30f},   /* 前侧预就位 */
            .target   = {0.425f, 0.425f, -0.25f, STAY_DOWN}, /* 前侧抓取点 */
            .retreat  = {0.298f, 0.308f, 0.075f, -0.10f},   /* 抓取后撤退 */
            .complete = {0.06f,  0.00f, 0.30f, STAY_LEVEL},   /* 归位 = startup */
            /* 中间途经点（如需经过某特定位置，取消注释）:
            // .waypoint_0 = {0.35f, 0.15f, 0.12f, -0.15f},  // 途经点0 */
        },
        /* 右臂 */
        .right = {
            .approach = {0.355f, -0.361f, 0.175f, -0.30f},   /* 前侧预就位 */
            .target   = {0.425f, -0.425f, -0.25f, STAY_DOWN}, /* 前侧抓取点 */
            .retreat  = {0.298f, -0.308f, 0.075f, -0.10f},   /* 抓取后撤退 */
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
        .left = {
            .approach = {0.355f, 0.361f, 0.125f, -0.30f},   /* 前侧预就位 */
            .target   = {0.425f, 0.425f, -0.25f, STAY_DOWN}, /* 前侧抓取点 */
            .retreat  = {0.298f, 0.308f, 0.075f, -0.10f},   /* 抓取后撤退 */
            .complete = {0.06f,  0.00f, 0.30f, STAY_LEVEL},   /* 归位 = startup */
        },
        .right     = {{{0}}},   /* 右臂不使用，保持原位 */
        .use_left  = true,
        .use_right = false,
    },

    /* ────────────────────────────────────────────────────────────
     * [3] ACTION_BLOCK_GET_FORWARD_RIGHT_ARM — 前侧物块右臂单独抓取
     *
     * 流程: 仅右臂移动，左臂保持原位不动
     * ──────────────────────────────────────────────────────────── */
    {
        .left      = {{{0}}},   /* 左臂不使用 */
        .right = {
            .approach = {0.355f, -0.361f, 0.175f, -0.30f},   /* 前侧预就位 */
            .target   = {0.425f, -0.425f, -0.25f, STAY_DOWN}, /* 前侧抓取点 */
            .retreat  = {0.298f, -0.308f, 0.075f, -0.10f},   /* 抓取后撤退 */
            .complete = {0.06f,  0.00f, 0.30f, STAY_LEVEL},   /* 归位 = startup */
        },
        .use_left  = false,
        .use_right = true,
    },

    /* ────────────────────────────────────────────────────────────
     * [4] ACTION_BLOCK_GET_BACKWARD — 后侧物块同时抓取（双臂）
     *
     * 注意: 后侧取物需 J1 旋转 ~151°。左臂 J1 上限仅 65°（+10° 扩展后），
     *       右臂 J1 下限仅 -65°。IK 极可能返回 UNREACHABLE。
     *       若发生，需减小 |Y| 值（约从 0.425→0.28 以下）使目标落入 J1 范围。
     * ──────────────────────────────────────────────────────────── */
    {
        .left = {
            .approach = {-0.308f, 0.361f, 0.175f, -0.30f},   /* 后侧预就位 */
            .target   = {-0.425f, 0.425f, 0.025f, STAY_DOWN}, /* 后侧抓取点 ⚠️ J1限位 */
            .retreat  = {-0.212f, 0.308f, 0.225f, -0.10f},   /* 抓取后撤退 */
            .complete = { 0.02f,  0.00f, 0.20f, -0.02f},
        },
        .right = {
            .approach = {-0.308f, -0.361f, 0.175f, -0.30f},   /* 后侧预就位 */
            .target   = {-0.425f, -0.425f, 0.025f, STAY_DOWN}, /* 后侧抓取点 ⚠️ J1限位 */
            .retreat  = {-0.212f, -0.308f, 0.225f, -0.10f},   /* 抓取后撤退 */
            .complete = { 0.02f,  0.00f, 0.20f, -0.02f},
        },
        .use_left  = true,
        .use_right = true,
    },

    /* ────────────────────────────────────────────────────────────
     * [5] ACTION_BLOCK_GET_BACKWARD_LEFT_ARM — 后侧物块左臂单独抓取
     * ──────────────────────────────────────────────────────────── */
    {
        .left = {
            .approach = {-0.308f, 0.361f, 0.175f, -0.30f},   /* 后侧预就位 */
            .target   = {-0.425f, 0.425f, 0.025f, STAY_DOWN}, /* 后侧抓取点 ⚠️ */
            .retreat  = {-0.212f, 0.308f, 0.225f, -0.10f},   /* 抓取后撤退 */
            .complete = { 0.02f,  0.00f, 0.20f, -0.02f},
        },
        .right     = {{{0}}},
        .use_left  = true,
        .use_right = false,
    },

    /* ────────────────────────────────────────────────────────────
     * [6] ACTION_BLOCK_GET_BACKWARD_RIGHT_ARM — 后侧物块右臂单独抓取
     * ──────────────────────────────────────────────────────────── */
    {
        .left      = {{{0}}},
        .right = {
            .approach = {-0.308f, -0.361f, 0.175f, -0.30f},   /* 后侧预就位 */
            .target   = {-0.425f, -0.425f, 0.025f, STAY_DOWN}, /* 后侧抓取点 ⚠️ */
            .retreat  = {-0.212f, -0.308f, 0.225f, -0.10f},   /* 抓取后撤退 */
            .complete = { 0.02f,  0.00f, 0.20f, -0.02f},
        },
        .use_left  = false,
        .use_right = true,
    },

    /* ────────────────────────────────────────────────────────────
     * [7] ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_BACK
     *     左臂放置物块到左背（机身左侧后方的储物区）
     *
     * 流程: 左臂携物块 → 放置预就位(PLACE_APPROACH) → 放置点(PLACE)
     *       → 释放(SUCTION_OFF) → 撤退(RETREAT) → 归位(COMPLETE)
     * 右臂保持原位。
     *
     * 注意: 放置位置应在左臂工作空间内（X>0 前侧 或 X<0 后侧），
     *       且 Y 偏左（+Y）。
     * ──────────────────────────────────────────────────────────── */
    {
        //ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_BACK
        .left = {
            .approach   = {0.24f, 0.13f, 0.13f, 0.0f},   /* 左背放置预就位 */
            .waypoint_0 = {0.12f, 0.26f, 0.13f, 0.0f}, /* J1=-90° 半伸展绕行点 */
            .target     = {-0.22f, 0.185f, 0.30f, -1.4f},  /* 左背放置点 */
            .retreat    = {0.11f, 0.29f, 0.155f, -0.10f},   /* 放置后撤退 */
            .complete   = {0.02f,  0.00f, 0.20f, -0.02f},
        },
        .right     = {{{0}}},
        .use_left  = true,
        .use_right = false,
        .use_waypoint = true,
        .left_back_effect = ACTION_4DOF_BACK_AVOID_SET,
        .left_back_avoid  = {0.08f, 0.0f, 0.36f, -0.10f},
    },

    /* ────────────────────────────────────────────────────────────
     * [8] ACTION_BLOCK_PLACE_LEFT_ARM_TO_RIGHT_BACK
     *     左臂放置物块到右背（机身右侧后方的储物区）
     *
     * 注意: 左臂跨中线到右侧放置，需确认 J1 限位和碰撞安全。
     * ──────────────────────────────────────────────────────────── */
    {
        //ACTION_BLOCK_PLACE_LEFT_ARM_TO_RIGHT_BACK
        .left = {
            .approach = {-0.062f, -0.074f, 0.18f, -0.30f},   /* 跨中线预就位（左臂→右背） */
            .target   = {-0.11f, -0.1325f, 0.03f, STAY_DOWN}, /* 右背放置点 */
            .retreat  = {-0.023f, -0.026f, 0.23f, -0.10f},   /* 放置后撤退 */
            .complete = { 0.02f,  0.00f, 0.20f, -0.02f},
        },
        .right     = {{{0}}},
        .use_left  = true,
        .use_right = false,
        .right_back_effect = ACTION_4DOF_BACK_AVOID_SET,
        .right_back_avoid  = {0.24f, -0.45f, 0.36f, -0.10f},
    },

    /* ────────────────────────────────────────────────────────────
     * [9] ACTION_BLOCK_PLACE_RIGHT_ARM_TO_LEFT_BACK
     *     右臂放置物块到左背（机身左侧后方的储物区）
     *
     * 注意: 右臂跨中线到左侧放置，需确认 J1 限位和碰撞安全。
     * ──────────────────────────────────────────────────────────── */
    {
        .left      = {{{0}}},
        .right = {
            .approach = {-0.062f, 0.074f, 0.18f, -0.30f},   /* 跨中线预就位（右臂→左背） */
            .target   = {-0.11f, 0.1325f, 0.03f, STAY_DOWN}, /* 左背放置点 */
            .retreat  = {-0.023f, 0.026f, 0.23f, -0.10f},   /* 放置后撤退 */
            .complete = { 0.02f,  0.00f, 0.20f, -0.02f},
        },
        .use_left  = false,
        .use_right = true,
        .left_back_effect = ACTION_4DOF_BACK_AVOID_SET,
        .left_back_avoid  = {0.24f, 0.45f, 0.36f, -0.10f},
    },

    /* ────────────────────────────────────────────────────────────
     * [10] ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_BACK
     *      右臂放置物块到右背（机身右侧后方的储物区）
     * ──────────────────────────────────────────────────────────── */
    {
        .left      = {{{0}}},
        .right = {
            .approach = {-0.062f, -0.133f, 0.18f, -0.30f},   /* 右背放置预就位 */
            .target   = {-0.11f, -0.1325f, 0.03f, STAY_DOWN}, /* 右背放置点 */
            .retreat  = {-0.023f, -0.133f, 0.23f, -0.10f},   /* 放置后撤退 */
            .complete = { 0.02f,  0.00f, 0.20f, -0.02f},
        },
        .use_left  = false,
        .use_right = true,
        .right_back_effect = ACTION_4DOF_BACK_AVOID_SET,
        .right_back_avoid  = {0.24f, -0.45f, 0.36f, -0.10f},
    },

    /* ────────────────────────────────────────────────────────────
     * [11] ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F1
     *      左臂放置物块到左1放置点第一层
     *
     * "左1放置点" 是机身上一个固定的堆叠放置位。
     * F1 = 第1层（底部），F2 = 第2层（堆叠在上方）。
     *
     * 注意: pitch 可能需要更接近垂直向下（-1.57 rad）以便精确放置。
     * ──────────────────────────────────────────────────────────── */
    {
        .left = {
            .approach = {0.05f,  0.35f, 0.30f, -1.00f},   /* TODO: 左1放置点F1预就位 */
            .target   = {0.05f,  0.45f, 0.08f, -1.50f},   /* TODO: 左1放置点F1 */
            .retreat  = {0.05f,  0.30f, 0.25f, -0.50f},   /* TODO: 放置后撤退 */
            .complete = {0.02f,  0.00f, 0.20f, -0.02f},
        },
        .right     = {{{0}}},
        .use_left  = true,
        .use_right = false,
        .left_back_effect = ACTION_4DOF_BACK_AVOID_CLEAR,
    },

    /* ────────────────────────────────────────────────────────────
     * [12] ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F2
     *      左臂放置物块到左1放置点第二层（堆叠）
     *
     * F2 的 Z 坐标应比 F1 高约一个物块厚度（~0.04m）。
     * ──────────────────────────────────────────────────────────── */
    {
        .left = {
            .approach = {0.05f,  0.35f, 0.34f, -1.00f},   /* TODO: 左1放置点F2预就位 */
            .target   = {0.05f,  0.45f, 0.12f, -1.50f},   /* TODO: 左1放置点F2（比F1高~0.04m） */
            .retreat  = {0.05f,  0.30f, 0.25f, -0.50f},   /* TODO */
            .complete = {0.02f,  0.00f, 0.20f, -0.02f},
        },
        .right     = {{{0}}},
        .use_left  = true,
        .use_right = false,
        .left_back_effect = ACTION_4DOF_BACK_AVOID_CLEAR,
    },

    /* ────────────────────────────────────────────────────────────
     * [13] ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F1
     *      右臂放置物块到右1放置点第一层
     * ──────────────────────────────────────────────────────────── */
    {
        .left      = {{{0}}},
        .right = {
            .approach = {0.05f, -0.3f, 0.30f, -1.00f},   /* TODO */
            .target   = {0.05f, -0.4f, -0.22f, STAY_DOWN},   /* TODO */
            .retreat  = {0.05f, -0.30f, 0.25f, -0.50f},   /* TODO */
            .complete = {0.02f,  0.00f, 0.20f, -0.02f},
        },
        .use_left  = false,
        .use_right = true,
        .right_back_effect = ACTION_4DOF_BACK_AVOID_CLEAR,
    },

    /* ────────────────────────────────────────────────────────────
     * [14] ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F2
     *      右臂放置物块到右1放置点第二层（堆叠）
     * ──────────────────────────────────────────────────────────── */
    {
        .left      = {{{0}}},
        .right = {
            .approach = {0.05f, -0.35f, 0.34f, -1.00f},   /* TODO */
            .target   = {0.05f, -0.45f, 0.03f, STAY_DOWN},   /* TODO: 比F1高~0.04m */
            .retreat  = {0.05f, -0.30f, 0.25f, -0.50f},   /* TODO */
            .complete = {0.02f,  0.00f, 0.20f, -0.02f},
        },
        .use_left  = false,
        .use_right = true,
        .right_back_effect = ACTION_4DOF_BACK_AVOID_CLEAR,
    },

    /* ────────────────────────────────────────────────────────────
     * [15] ACTION_DANCE — 神秘舞蹈动作
     *
     * 多途经点序列动作。DANCE 不使用固定的 approach/target/retreat 字段，
     * 而是通过 WAYPOINT 子状态 + s_dance_waypoints[] 数组依次访问各途经点。
     *
     * 途经点数量由 DANCE_WAYPOINT_COUNT 定义（见下方）。
     * ──────────────────────────────────────────────────────────── */
    {
        .left      = {{{0}}},   /* DANCE 使用 s_dance_waypoints，不使用此结构 */
        .right     = {{{0}}},
        .use_left  = true,
        .use_right = true,
    },
};

/* 动作总数（用于边界检查） */
#define ACTION_4DOF_COUNT \
    (sizeof(s_action_targets) / sizeof(s_action_targets[0]))

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
    /* 添加更多途经点只需在此数组末尾追加即可 */
};

/* ════════════════════════════════════════════════════════════════
 * 内部状态机上下文
 * ════════════════════════════════════════════════════════════════ */

typedef struct {
    action_state_4dof_e     action;         /**< 当前执行的动作 */
    action_4dof_substate_e  substate;       /**< 当前子状态 */
    uint32_t                enter_tick;     /**< 进入当前子状态的系统 tick */
    uint32_t                timeout_ms;     /**< 当前子状态的超时时间 */
    uint8_t                 waypoint_idx;   /**< 当前途经点索引（DANCE 用） */
    bool                    active;         /**< 是否有动作正在执行 */
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
    return (action >= ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_BACK &&
            action <= ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F2);
}

/**
 * @brief 给当前阶段同时下发工作臂目标和非工作臂避让目标。
 *
 * 该函数是单臂动作防干涉的核心入口：
 * - 工作臂使用 left_work_pose/right_work_pose 中对应侧目标；
 * - 非工作臂根据当前动作自动选择抓取避让位或放置避让位；
 * - 双臂动作中两臂都是工作臂，不会额外套用避让位；
 * - DANCE 使用单独 waypoint 流程，不调用本函数。
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
    (void)action;

    if (td->use_left) {
        action_4dof_set_arm_target(&g_dof4_arm_left, left_work_pose, left_label);
    }

    if (td->use_right) {
        action_4dof_set_arm_target(&g_dof4_arm_right, right_work_pose, right_label);
    }
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

static bool action_4dof_is_block_grabbed(uint8_t arm_side)
{
    if (arm_side > 1U) return false;
    return (g_switch_input.state[arm_side] != 0U);
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

    /* 根据子状态执行对应阶段 */
    switch (s_ctx.substate) {

    /* ═══════════════════════════════════════════════════════════
     * 子状态: APPROACH — 移动到预就位点
     *
     * 设定预就位目标，到位后（或超时容错）进入 GRAB。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_APPROACH:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.approach, &td->right.approach,
                                      "L_ap", "R_ap");

        {
            bool left_ok  = !td->use_left  || action_4dof_arm_reached_pose(&g_dof4_arm_left,  &td->left.approach);
            bool right_ok = !td->use_right || action_4dof_arm_reached_pose(&g_dof4_arm_right, &td->right.approach);
            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                if (td->use_waypoint) {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_INTERMEDIATE, ACT4_MOVE_TIMEOUT_MS);
                } else {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_GRAB, ACT4_MOVE_TIMEOUT_MS);
                }
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: INTERMEDIATE — 中间途经点（J1 绕行安全位）
     *
     * 使用 waypoint_0 作为目标，到位后进入 GRAB（抓取流）或 PLACE（放置流）。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_INTERMEDIATE:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.waypoint_0, &td->right.waypoint_0,
                                      "L_wp0", "R_wp0");

        {
            bool left_ok  = !td->use_left  || action_4dof_arm_reached_pose(&g_dof4_arm_left,  &td->left.waypoint_0);
            bool right_ok = !td->use_right || action_4dof_arm_reached_pose(&g_dof4_arm_right, &td->right.waypoint_0);
            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                if (action_4dof_is_place_action(s_ctx.action)) {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_PLACE, ACT4_MOVE_TIMEOUT_MS);
                } else {
                    action_4dof_set_substate(ACTION_4DOF_SUBSTATE_GRAB, ACT4_MOVE_TIMEOUT_MS);
                }
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: GRAB — 移动到抓取点
     *
     * 从预就位下降到物块位置。到位后（或超时容错）开启吸盘。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_GRAB:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.target, &td->right.target,
                                      "L_grab", "R_grab");

        {
            bool left_ok  = !td->use_left  || action_4dof_arm_reached_pose(&g_dof4_arm_left,  &td->left.target);
            bool right_ok = !td->use_right || action_4dof_arm_reached_pose(&g_dof4_arm_right, &td->right.target);
            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_ON, ACT4_SUCTION_TIMEOUT_MS);
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: SUCTION_ON — 开启吸盘，等待吸附确认
     *
     * 开启对应臂的电磁阀，等待微动开关触发。
     * 左臂→relay 0, 右臂→relay 1。
     * 超时后容错推进（即使未检测到吸附）。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_SUCTION_ON:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.target, &td->right.target,
                                      "L_suck", "R_suck");
        if (td->use_left)  relay_control(RELAY_LEFT_ARM,  SUCTION_ON);
        if (td->use_right) relay_control(RELAY_RIGHT_ARM, SUCTION_ON);

        {
            bool left_ok  = !td->use_left  || action_4dof_is_block_grabbed(0);
            bool right_ok = !td->use_right || action_4dof_is_block_grabbed(1);

            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                /* 吸附成功（或超时容错） */
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_RETREAT, ACT4_MOVE_TIMEOUT_MS);
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: RETREAT — 携带物块撤退到安全高度
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_RETREAT:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.retreat, &td->right.retreat,
                                      "L_ret", "R_ret");

        {
            bool left_ok  = !td->use_left  || action_4dof_arm_reached_pose(&g_dof4_arm_left,  &td->left.retreat);
            bool right_ok = !td->use_right || action_4dof_arm_reached_pose(&g_dof4_arm_right, &td->right.retreat);
            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_COMPLETE, ACT4_HOLD_MS);
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: PLACE_APPROACH — 移动到放置预就位点
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_PLACE_APPROACH:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.approach, &td->right.approach,
                                      "L_pap", "R_pap");

        {
            bool left_ok  = !td->use_left  || action_4dof_arm_reached_pose(&g_dof4_arm_left,  &td->left.approach);
            bool right_ok = !td->use_right || action_4dof_arm_reached_pose(&g_dof4_arm_right, &td->right.approach);
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
     * 子状态: PLACE — 移动到放置点
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_PLACE:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.target, &td->right.target,
                                      "L_place", "R_place");

        {
            bool left_ok  = !td->use_left  || action_4dof_arm_reached_pose(&g_dof4_arm_left,  &td->left.target);
            bool right_ok = !td->use_right || action_4dof_arm_reached_pose(&g_dof4_arm_right, &td->right.target);
            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_SUCTION_OFF, ACT4_RELEASE_TIMEOUT_MS);
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: SUCTION_OFF — 关闭吸盘，释放物块
     *
     * 关闭电磁阀，等待微动开关确认脱离。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_SUCTION_OFF:
        action_4dof_set_stage_targets(td, s_ctx.action,
                                      &td->left.target, &td->right.target,
                                      "L_rel", "R_rel");
        if (td->use_left)  relay_control(RELAY_LEFT_ARM,  SUCTION_OFF);
        if (td->use_right) relay_control(RELAY_RIGHT_ARM, SUCTION_OFF);

        {
            bool left_ok  = !td->use_left  || !action_4dof_is_block_grabbed(0);
            bool right_ok = !td->use_right || !action_4dof_is_block_grabbed(1);

            if ((left_ok && right_ok) || action_4dof_is_timed_out()) {
                /* 释放成功 → 更新背上有块状态 */
                action_4dof_apply_back_avoid_effects(td);
                action_4dof_set_substate(ACTION_4DOF_SUBSTATE_RETREAT, ACT4_MOVE_TIMEOUT_MS);
            }
        }
        break;

    /* ═══════════════════════════════════════════════════════════
     * 子状态: COMPLETE — 动作完成，归位
     *
     * 移动到 complete 归位点，短暂保持后回到 IDLE。
     * ═══════════════════════════════════════════════════════════ */
    case ACTION_4DOF_SUBSTATE_COMPLETE:
        /* 使用全局物块状态驱动的自适应 IDLE 位姿 */
        action_4dof_set_complete_targets(td);

        if (action_4dof_is_timed_out()) {
            /* 动作完全结束，回到 IDLE */
            s_ctx.active = false;
            s_ctx.action = ACTION_4DOF_IDLE;
            s_ctx.substate = ACTION_4DOF_SUBSTATE_IDLE;
            s_ctx.timeout_ms = 0U;
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
                } else {
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
bool action_4dof_trigger(action_state_4dof_e action)
{
    /* 检查当前是否有动作正在执行 */
    if (s_ctx.active) {
        return false;
    }

    /* 检查 action 是否合法 */
    if (action <= ACTION_4DOF_IDLE || (uint32_t)action >= ACTION_4DOF_COUNT) {
        return false;
    }

    /* 初始化上下文 */
    s_ctx.action       = action;
    s_ctx.active       = true;
    s_ctx.waypoint_idx = 0U;

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
