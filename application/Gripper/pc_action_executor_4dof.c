/**
 * @file    pc_action_executor_4dof.c
 * @brief   PC 下发动作专用 4DOF 状态机
 *
 * 本模块与 action_scheduler_4dof 的 RC/预设动作状态机相互独立。两者只共享
 * 机械臂底层目标接口、吸盘接口和背部物块占用状态，并通过临界区保证同一时刻
 * 只有一个状态机取得执行权。
 *
 * 动态取放路径：
 *   相对安全入口 -> 目标正上方 -> 垂直下降到目标
 *   -> 吸取/释放 -> 垂直抬回目标正上方 -> 相对安全出口
 *   -> 自适应 IDLE
 *
 * 背部固定路径使用关节角序列。路径数组和有效长度分离，后续实机调试需要增加
 * 中间点时，只需在对应数组中追加关节角并调整 count。
 *
 * 超时原则：
 *   - 运动阶段未到位：立即终止后续吸取/释放操作，尝试回自适应 IDLE。
 *   - 释放前终止：保持手臂吸盘开启，不修改背部占用状态，避免掉块。
 *   - 释放后终止：保留已经执行的阀门和背部占用结果。
 *   - 成功和超时终止都会产生一次 0xCC 结束事件；0xCC 不区分结果。
 */

#include "pc_action_executor_4dof.h"

#include "FreeRTOS.h"
#include "action_scheduler_4dof.h"
#include "pneumatic_control.h"
#include "stm32f4xx_hal.h"
#include "task.h"

#include <math.h>
#include <string.h>

/* ======================== 吸盘继电器映射 ======================== */
#define PC_ACT4_RELAY_LEFT_ARM   0U   /**< 左手臂吸盘继电器索引 */
#define PC_ACT4_RELAY_RIGHT_ARM  1U   /**< 右手臂吸盘继电器索引 */
#define PC_ACT4_RELAY_LEFT_BACK  2U   /**< 左背部吸盘继电器索引 */
#define PC_ACT4_RELAY_RIGHT_BACK 3U   /**< 右背部吸盘继电器索引 */
#define PC_ACT4_SUCTION_ON       1U   /**< 吸盘开启 */
#define PC_ACT4_SUCTION_OFF      0U   /**< 吸盘关闭 */

/* ======================== 运动到位公差 ======================== */
/** @brief 动态目标正上方点统一比 PC 目标高 0.20 m（定义于头文件）。 */
/* 到位和超时参数沿用原 4DOF 调度器的量级，便于实机统一调试。 */
#define PC_ACT4_MOVE_TIMEOUT_MS              2500U   /**< 单段路径运动超时 (ms) */
#define PC_ACT4_POSE_POS_TOL_M                0.03f  /**< 位姿位置到位公差 (m) */
#define PC_ACT4_POSE_PITCH_TOL_RAD            0.05f  /**< 位姿俯仰角到位公差 (rad) */
#define PC_ACT4_JOINT_TOL_RAD                  0.05f /**< 关节角到位公差 (rad) */

/* ======================== 操作保持时间 ======================== */
#define PC_ACT4_PICK_HOLD_MS                  1500U  /**< 吸取保持时间 (ms) */
#define PC_ACT4_EXTERNAL_RELEASE_HOLD_MS      2000U  /**< 外部放置释放保持时间 (ms) */
#define PC_ACT4_BACK_PRE_RELEASE_HOLD_MS      2000U  /**< 背部放置-预吸附保持时间 (ms) */
#define PC_ACT4_BACK_POST_RELEASE_HOLD_MS     2000U  /**< 背部放置-后释放保持时间 (ms) */
#define PC_ACT4_BACK_GET_ARM_HOLD_MS          1000U  /**< 背部取块-手臂吸附保持时间 (ms) */
#define PC_ACT4_BACK_RELEASE_HOLD_MS           500U  /**< 背部取块-背部释放保持时间 (ms) */
#define PC_ACT4_IDLE_HOLD_MS                   100U  /**< 到达 IDLE 后的保持时间 (ms) */

/* ======================== 路径数组容量 ======================== */
#define PC_ACT4_ARM_COUNT              2U     /**< 机械臂数量（左/右） */
/* 数组容量大于当前有效点数，实机调试时可直接追加中间点并调整 count。 */
#define PC_ACT4_MAX_PRE_POSE_POINTS    5U     /**< 位姿路径-接近段最大点数 */
#define PC_ACT4_MAX_POST_POSE_POINTS   4U     /**< 位姿路径-撤离段最大点数 */
#define PC_ACT4_MAX_PRE_JOINT_POINTS   6U     /**< 关节路径-接近段最大点数 */
#define PC_ACT4_MAX_POST_JOINT_POINTS  4U     /**< 关节路径-撤离段最大点数 */

/**
 * @brief PC 动作命令枚举。
 *
 * 每个值对应一种由 PC 下发的动作类型，与 0x11/0x12/0x14/0x15/0x21/0x22
 * 协议命令一一映射。
 */
typedef enum {
    PC_ACT4_COMMAND_NONE = 0,       /**< 无命令 / 空闲 */
    PC_ACT4_COMMAND_PICK,           /**< 0x11: 单臂动态取块 */
    PC_ACT4_COMMAND_PLACE,          /**< 0x12: 单臂动态放块 */
    PC_ACT4_COMMAND_PUT_BACK,       /**< 0x14: 单臂放块到背部 */
    PC_ACT4_COMMAND_GET_BACK,       /**< 0x15: 单臂从背部取块 */
    PC_ACT4_COMMAND_DUAL_PICK,      /**< 0x21: 双臂动态取块 */
    PC_ACT4_COMMAND_DUAL_PUT_BACK,  /**< 0x22: 双臂放块到背部 */
} PcAction4DOF_Command;

/**
 * @brief 路径模式枚举。
 *
 * POSE 模式使用工作空间位姿（x, y, z, pitch），适用于动态取放路径；
 * JOINT 模式使用关节角序列，适用于背部固定路径。
 */
typedef enum {
    PC_ACT4_MODE_POSE = 0,   /**< 位姿模式（动态路径） */
    PC_ACT4_MODE_JOINT,      /**< 关节角模式（固定路径） */
} PcAction4DOF_Mode;

/**
 * @brief PC 状态机状态枚举。
 *
 * 状态转换流程：
 *   IDLE -> MOVE_PRE_PATH (接近) -> OPERATION_HOLD (操作保持)
 *   -> [OPERATION_SECOND_HOLD (二次保持，仅背部操作)] -> MOVE_POST_PATH (撤离)
 *   -> RETURN_IDLE -> IDLE_HOLD -> IDLE
 *
 * 任何运动阶段超时都迁入 ABORT_RETURN_IDLE 尝试紧急归位。
 */
typedef enum {
    PC_ACT4_STATE_IDLE = 0,             /**< 空闲态 */
    PC_ACT4_STATE_MOVE_PRE_PATH,        /**< 接近路径运动 */
    PC_ACT4_STATE_OPERATION_HOLD,       /**< 操作保持（吸取/释放/背部交接） */
    PC_ACT4_STATE_OPERATION_SECOND_HOLD,/**< 二次保持（仅背部操作时的第二阶段保持） */
    PC_ACT4_STATE_MOVE_POST_PATH,       /**< 撤离路径运动 */
    PC_ACT4_STATE_RETURN_IDLE,          /**< 回归 IDLE 位姿 */
    PC_ACT4_STATE_ABORT_RETURN_IDLE,    /**< 超时中止后强制归位 */
    PC_ACT4_STATE_IDLE_HOLD,            /**< 到达 IDLE 后的短暂保持 */
} PcAction4DOF_State;

/**
 * @brief 关节角路径点，包含 4 个关节的目标角度。
 */
typedef struct {
    float q[DOF4_JOINT_COUNT];  /**< 关节角数组 [J1, J2, J3, J4]，单位 rad */
} PcAction4DOF_JointPoint;

/**
 * @brief 位姿模式路径，包含接近段和撤离段两个方向的路点序列。
 *
 * pre:  从安全位置运动到操作目标的路径点序列。
 * post: 操作完成后从目标撤离回到安全位置的路径点序列。
 */
typedef struct {
    Dof4_Pose pre[PC_ACT4_MAX_PRE_POSE_POINTS];    /**< 接近段路点 */
    Dof4_Pose post[PC_ACT4_MAX_POST_POSE_POINTS];   /**< 撤离段路点 */
    uint8_t pre_count;   /**< 接近段有效路点数（<= PC_ACT4_MAX_PRE_POSE_POINTS） */
    uint8_t post_count;  /**< 撤离段有效路点数（<= PC_ACT4_MAX_POST_POSE_POINTS） */
} PcAction4DOF_PosePath;

/**
 * @brief 关节角模式路径，包含接近段和撤离段两个方向的关节角序列。
 *
 * 用于背部存放/取回等固定路径，路径点以关节角形式直接指定。
 */
typedef struct {
    PcAction4DOF_JointPoint pre[PC_ACT4_MAX_PRE_JOINT_POINTS];   /**< 接近段关节角序列 */
    PcAction4DOF_JointPoint post[PC_ACT4_MAX_POST_JOINT_POINTS]; /**< 撤离段关节角序列 */
    uint8_t pre_count;   /**< 接近段有效点数（<= PC_ACT4_MAX_PRE_JOINT_POINTS） */
    uint8_t post_count;  /**< 撤离段有效点数（<= PC_ACT4_MAX_POST_JOINT_POINTS） */
} PcAction4DOF_JointPath;

/**
 * @brief 动态动作模板。
 *
 * entry/exit_offset 是当前已调模板中 approach/retreat 相对 target 的偏移。
 * PC 只提供最终 xyz；安全入口和出口通过 target + offset 生成。
 * 每个手臂（左/右）各有一个独立模板，以适应双侧不同的工作空间约束。
 */
typedef struct {
    Dof4_Pose entry_offset;  /**< 入口偏移量（相对 target 的 dx, dy, dz, dpitch） */
    Dof4_Pose exit_offset;   /**< 出口偏移量（相对 target 的 dx, dy, dz, dpitch） */
    float target_pitch;      /**< 操作目标点的俯仰角，单位 rad */
} PcAction4DOF_DynamicTemplate;

/**
 * @brief PC 动作状态机运行时上下文。
 *
 * 所有字段在 claim 时一次写入，运行期间由 pc_action_4dof_loop 维护。
 * 临界区保护保证与 RC 状态机互斥访问。
 */
typedef struct {
    PcAction4DOF_Command command;       /**< 当前执行的动作命令类型 */
    PcAction4DOF_Mode mode;             /**< 路径模式：位姿或关节角 */
    PcAction4DOF_State state;           /**< 状态机当前状态 */
    uint32_t enter_tick;                /**< 进入当前状态的时刻 (HAL_GetTick) */
    uint32_t timeout_ms;                /**< 当前状态的超时时间 (ms)，0 表示不限 */
    uint8_t path_index;                 /**< 当前路径点索引（正在运动的目标点） */
    volatile bool active;               /**< 状态机是否活跃（正在执行动作） */
    bool use_left;                      /**< 本次动作是否使用左臂 */
    bool use_right;                     /**< 本次动作是否使用右臂 */
    bool release_committed;             /**< 是否已经执行了物块释放操作 */
    bool operation_first_step_done;     /**< OPERATION_HOLD 首次周期是否已完成 */
    union {
        PcAction4DOF_PosePath pose[PC_ACT4_ARM_COUNT];   /**< 位姿路径（双臂各一份） */
        PcAction4DOF_JointPath joint[PC_ACT4_ARM_COUNT]; /**< 关节角路径（双臂各一份） */
    } path;                             /**< 路径数据，根据 mode 选择访问 pose 或 joint */
} PcAction4DOF_Context;

/* ======================== 外部全局变量引用 ======================== */
extern Dof4_Arm g_dof4_arm_left;   /**< 左机械臂实例（定义于 Dof4_Arm.c） */
extern Dof4_Arm g_dof4_arm_right;  /**< 右机械臂实例（定义于 Dof4_Arm.c） */

/* ======================== 模块内部静态变量 ======================== */
static PcAction4DOF_Context s_pc_ctx;            /**< PC 状态机运行时上下文 */
static volatile bool s_pc_completion_pending;    /**< 是否有待发送的 0xCC 结束事件 */

/* ======================== 单臂动态动作模板 ======================== */
/**
 * @brief 单臂取块动作模板。
 *
 * 各数值由已调动作表换算为相对 PC 下发 target 的偏移量。
 * [0]=左臂, [1]=右臂，偏移方向因左右工作空间对称性而不同。
 */
static const PcAction4DOF_DynamicTemplate s_pick_templates[PC_ACT4_ARM_COUNT] = {
    /* 左臂 */
    {
        .entry_offset = {-0.10f, -0.15f, 0.38f, -0.60f},
        .exit_offset  = {-0.10f, -0.07f, 0.38f, -0.30f},
        .target_pitch = -1.50f,
    },
    /* 右臂 */
    {
        .entry_offset = {0.10f, 0.15f, 0.40f, -0.60f},
        .exit_offset  = {0.10f, 0.07f, 0.40f, -0.30f},
        .target_pitch = -1.50f,
    },
};

/* ======================== 双臂动态动作模板 ======================== */
/**
 * @brief 双臂取块动作模板。
 *
 * 双臂同时取块时双臂间距较大，entry_offset 需额外考虑双臂干涉规避。
 */
static const PcAction4DOF_DynamicTemplate s_dual_pick_templates[PC_ACT4_ARM_COUNT] = {
    /* 左臂 */
    {
        .entry_offset = {0.10f, -0.15f, 0.28f, -0.60f},
        .exit_offset  = {0.10f, -0.07f, 0.365f, -0.30f},
        .target_pitch = -1.50f,
    },
    /* 右臂 */
    {
        .entry_offset = {0.10f, 0.15f, 0.30f, -0.60f},
        .exit_offset  = {0.10f, 0.07f, 0.385f, -0.30f},
        .target_pitch = -1.50f,
    },
};

/* ======================== 单臂放置动作模板 ======================== */
/**
 * @brief 单臂放块动作模板。
 *
 * 放置时需要更大的垂直空间以避开已堆叠物块，因此 entry_offset.z 偏大。
 */
static const PcAction4DOF_DynamicTemplate s_place_templates[PC_ACT4_ARM_COUNT] = {
    /* 左臂 */
    {
        .entry_offset = {-0.075f, -0.20f, 0.49f, -0.30f},
        .exit_offset  = {-0.175f, -0.24f, 0.44f, -0.50f},
        .target_pitch = -1.50f,
    },
    /* 右臂 */
    {
        .entry_offset = {-0.075f, 0.10f, 0.52f, -1.00f},
        .exit_offset  = {-0.175f, 0.15f, 0.43f, -0.50f},
        .target_pitch = -1.50f,
    },
};

#define PC_JOINT_POINT(j1_, j2_, j3_, j4_) \
    { .q = {(j1_), (j2_), (j3_), (j4_)} }

/* ======================== 背部固定路径 - 关节角序列 ======================== */
/**
 * @brief 单臂放块到背部路径（0x14）。
 *
 * 以下关节角逐点迁移自原 action_scheduler_4dof 动作表，并按当前左右臂
 * J1 坐标方向换算。pre 段从当前位姿运动至背部吸盘位置；post 段撤离回退。
 */
static const PcAction4DOF_JointPath s_put_back_paths[PC_ACT4_ARM_COUNT] = {
    /* 左臂：J1 向负方向旋转到左后方。 */
    {
        .pre = {
            PC_JOINT_POINT(-0.042f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(-1.570f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(-2.800f, 1.50f, -1.19f, -1.680f),
            PC_JOINT_POINT(-2.800f, 1.57f, -1.88f, -1.247f),
        },
        .post = {
            PC_JOINT_POINT(-2.100f, 1.50f, -1.19f, -1.680f),
            PC_JOINT_POINT(-0.042f, 1.50f, -1.70f, -1.463f),
        },
        .pre_count = 4U,
        .post_count = 2U,
    },
    /* 右臂：J1 向正方向旋转到右后方。 */
    {
        .pre = {
            PC_JOINT_POINT(0.042f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(1.570f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(2.800f, 1.50f, -1.00f, -1.680f),
            PC_JOINT_POINT(2.800f, 1.57f, -1.88f, -1.247f),
        },
        .post = {
            PC_JOINT_POINT(2.000f, 1.50f, -1.80f, -1.680f),
            PC_JOINT_POINT(0.042f, 1.50f, -1.70f, -1.463f),
        },
        .pre_count = 4U,
        .post_count = 2U,
    },
};

/**
 * @brief 双臂同时放块到背部路径（0x22）。
 *
 * 与单臂路径相比，双臂模式下展开角度更大（J1=±3.10），为对侧手臂留出空间。
 */
static const PcAction4DOF_JointPath s_dual_put_back_paths[PC_ACT4_ARM_COUNT] = {
    /* 左臂 */
    {
        .pre = {
            PC_JOINT_POINT(-0.042f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(-1.570f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(-3.100f, 1.50f, -1.19f, -1.680f),
            PC_JOINT_POINT(-3.040f, 1.57f, -1.88f, -1.247f),
        },
        .post = {
            PC_JOINT_POINT(-3.100f, 1.50f, -1.19f, -1.680f),
            PC_JOINT_POINT(-0.042f, 1.50f, -1.70f, -1.463f),
        },
        .pre_count = 4U,
        .post_count = 2U,
    },
    /* 右臂 */
    {
        .pre = {
            PC_JOINT_POINT(0.042f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(1.570f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(3.100f, 1.50f, -1.19f, -1.680f),
            PC_JOINT_POINT(3.040f, 1.57f, -1.88f, -1.247f),
        },
        .post = {
            PC_JOINT_POINT(3.100f, 1.50f, -1.19f, -1.680f),
            PC_JOINT_POINT(0.042f, 1.50f, -1.70f, -1.463f),
        },
        .pre_count = 4U,
        .post_count = 2U,
    },
};

/**
 * @brief 单臂从背部取块路径（0x15）。
 *
 * 与 put_back 反向：先从当前位姿运动到背部吸盘位置吸取物块，
 * 然后撤离回到安全位姿。pre 段最后一个点对准背部，post 段撤回。
 * 注意左臂 pre[0] 和 pre[1] 重复（两点相同以实现停留）。
 */
static const PcAction4DOF_JointPath s_get_back_paths[PC_ACT4_ARM_COUNT] = {
    /* 左臂 */
    {
        .pre = {
            PC_JOINT_POINT(-3.100f, 1.50f, -1.19f, -1.680f),
            PC_JOINT_POINT(-3.100f, 1.50f, -1.19f, -1.680f),
            PC_JOINT_POINT(-1.570f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(-3.040f, 1.57f, -1.88f, -1.247f),
        },
        .post = {
            PC_JOINT_POINT(-0.042f, 1.50f, -1.70f, -1.463f),
            PC_JOINT_POINT(-0.042f, 1.50f, -1.70f, -1.463f),
        },
        .pre_count = 4U,
        .post_count = 2U,
    },
    /* 右臂 */
    {
        .pre = {
            PC_JOINT_POINT(1.570f, 1.50f, -1.19f, -1.680f),
            PC_JOINT_POINT(2.000f, 1.50f, -1.00f, -1.463f),
            PC_JOINT_POINT(2.800f, 1.50f, -1.00f, -1.680f),
            PC_JOINT_POINT(2.800f, 1.57f, -1.88f, -1.247f),
        },
        .post = {
            PC_JOINT_POINT(1.570f, 1.50f, -1.80f, -1.680f),
            PC_JOINT_POINT(0.042f, 1.50f, -1.70f, -1.463f),
        },
        .pre_count = 4U,
        .post_count = 2U,
    },
};

/* ======================== 辅助函数 ======================== */

/**
 * @brief 根据索引获取机械臂实例指针。
 * @param arm_index 0=左臂, 1=右臂
 * @return Dof4_Arm 指针
 */
static Dof4_Arm *pc_action_4dof_arm(uint8_t arm_index)
{
    return (arm_index == 0U) ? &g_dof4_arm_left : &g_dof4_arm_right;
}

/**
 * @brief 检查指定手臂是否被当前动作使用。
 * @param ctx       PC 动作上下文
 * @param arm_index 手臂索引 (0=左, 1=右)
 * @return true 表示该手臂参与当前动作
 */
static bool pc_action_4dof_arm_used(const PcAction4DOF_Context *ctx,
                                    uint8_t arm_index)
{
    return (arm_index == 0U) ? ctx->use_left : ctx->use_right;
}

/**
 * @brief 校验固定关节路径，避免动作被接受后因限位裁剪而永远无法到位。
 */
static bool pc_action_4dof_joint_path_valid(
    uint8_t arm_index,
    const PcAction4DOF_JointPath *path)
{
    if (arm_index >= PC_ACT4_ARM_COUNT || path == NULL ||
        path->pre_count == 0U ||
        path->pre_count > PC_ACT4_MAX_PRE_JOINT_POINTS ||
        path->post_count > PC_ACT4_MAX_POST_JOINT_POINTS) {
        return false;
    }

    const Dof4_Arm *arm = pc_action_4dof_arm(arm_index);
    for (uint8_t segment = 0U; segment < 2U; ++segment) {
        const PcAction4DOF_JointPoint *points =
            (segment == 0U) ? path->pre : path->post;
        const uint8_t count =
            (segment == 0U) ? path->pre_count : path->post_count;

        for (uint8_t point_index = 0U; point_index < count; ++point_index) {
            for (uint8_t joint_index = 0U;
                 joint_index < DOF4_JOINT_COUNT;
                 ++joint_index) {
                const float angle = points[point_index].q[joint_index];
                int16_t servo_pos;
                if (!isfinite(angle) ||
                    angle < arm->cfg.joint_min[joint_index] ||
                    angle > arm->cfg.joint_max[joint_index] ||
                    Dof4_angle_to_servo(arm, joint_index, angle, &servo_pos) !=
                        DOF4_STATUS_OK) {
                    return false;
                }
            }
        }
    }
    return true;
}

/**
 * @brief 检查位姿数据是否有限（非 NaN、非无穷）。
 * @param pose 待检查的位姿指针
 * @return true 表示所有分量均有限
 */
static bool pc_action_4dof_pose_finite(const Dof4_Pose *pose)
{
    return pose != NULL &&
           isfinite(pose->x) &&
           isfinite(pose->y) &&
           isfinite(pose->z) &&
           isfinite(pose->pitch);
}

/**
 * @brief 检查位姿是否可达（运动学逆解有解且各关节不超限）。
 *
 * 通过逆运动学计算并验证所有关节角在机械臂限位范围内。
 *
 * @param arm  机械臂实例（包含运动学参数和关节限位）
 * @param pose 目标位姿
 * @return true 表示可达
 */
static bool pc_action_4dof_pose_reachable(Dof4_Arm *arm,
                                          const Dof4_Pose *pose)
{
    Dof4_JointState joints;

    if (arm == NULL || !pc_action_4dof_pose_finite(pose) ||
        Dof4_arm_inverse_kinematics(arm, pose, -1.0f, &joints) != DOF4_STATUS_OK) {
        return false;
    }

    for (uint8_t i = 0U; i < DOF4_JOINT_COUNT; ++i) {
        if (!isfinite(joints.q[i]) ||
            joints.q[i] < arm->cfg.joint_min[i] ||
            joints.q[i] > arm->cfg.joint_max[i]) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 检查机械臂末端是否已到达目标位姿（在公差范围内）。
 *
 * 同时检查位置误差（三维欧氏距离）和俯仰角误差，两者均需在允许公差内。
 *
 * @param arm    机械臂实例（读取 current_pose）
 * @param target 目标位姿
 * @return true 表示已到位
 */
static bool pc_action_4dof_pose_reached(const Dof4_Arm *arm,
                                        const Dof4_Pose *target)
{
    if (arm == NULL || target == NULL) {
        return false;
    }

    const float dx = arm->current_pose.x - target->x;
    const float dy = arm->current_pose.y - target->y;
    const float dz = arm->current_pose.z - target->z;
    const float position_error = sqrtf(dx * dx + dy * dy + dz * dz);
    const float pitch_error = fabsf(arm->current_pose.pitch - target->pitch);

    return position_error <= PC_ACT4_POSE_POS_TOL_M &&
           pitch_error <= PC_ACT4_POSE_PITCH_TOL_RAD;
}

/**
 * @brief 检查机械臂各关节是否已到达目标关节角（在公差范围内）。
 *
 * 使用 Dof4_normalize_angle 将角度差归一化到 [-π, π] 后取绝对值比较。
 *
 * @param arm    机械臂实例（读取 joint_actual）
 * @param target 目标关节角点
 * @return true 表示所有关节均到位
 */
static bool pc_action_4dof_joint_reached(const Dof4_Arm *arm,
                                         const PcAction4DOF_JointPoint *target)
{
    if (arm == NULL || target == NULL) {
        return false;
    }

    for (uint8_t i = 0U; i < DOF4_JOINT_COUNT; ++i) {
        const float error =
            fabsf(Dof4_normalize_angle(arm->joint_actual.q[i] - target->q[i]));
        if (error > PC_ACT4_JOINT_TOL_RAD) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 设置状态机的新状态并记录进入时刻和超时时间。
 *
 * @param state      新状态
 * @param timeout_ms 超时时间 (ms)，0 表示不超时
 */
static void pc_action_4dof_set_state(PcAction4DOF_State state,
                                      uint32_t timeout_ms)
{
    s_pc_ctx.state = state;
    s_pc_ctx.enter_tick = HAL_GetTick();
    s_pc_ctx.timeout_ms = timeout_ms;
}

/**
 * @brief 检查当前状态是否已超时。
 * @return true 表示已超过设置的 timeout_ms
 */
static bool pc_action_4dof_timed_out(void)
{
    return s_pc_ctx.timeout_ms != 0U &&
           (HAL_GetTick() - s_pc_ctx.enter_tick) >= s_pc_ctx.timeout_ms;
}

/**
 * @brief 设置工作臂的吸盘状态（开启/关闭）。
 *
 * 根据 use_left/use_right 决定控制哪一侧的手臂吸盘继电器。
 *
 * @param state PC_ACT4_SUCTION_ON 或 PC_ACT4_SUCTION_OFF
 */
static void pc_action_4dof_set_arm_suction(uint8_t state)
{
    if (s_pc_ctx.use_left) {
        relay_control(PC_ACT4_RELAY_LEFT_ARM, state);
    }
    if (s_pc_ctx.use_right) {
        relay_control(PC_ACT4_RELAY_RIGHT_ARM, state);
    }
}

/**
 * @brief 设置目标背部吸盘状态（开启/关闭）。
 *
 * 用于背部存放/取回操作时控制背部吸盘继电器。
 *
 * @param state PC_ACT4_SUCTION_ON 或 PC_ACT4_SUCTION_OFF
 */
static void pc_action_4dof_set_target_back_suction(uint8_t state)
{
    if (s_pc_ctx.use_left) {
        relay_control(PC_ACT4_RELAY_LEFT_BACK, state);
    }
    if (s_pc_ctx.use_right) {
        relay_control(PC_ACT4_RELAY_RIGHT_BACK, state);
    }
}

/**
 * @brief 更新背部物块占用状态。
 *
 * @param occupied true=标记为占用（放块完成），false=标记为空闲（取块完成）
 */
static void pc_action_4dof_apply_back_state(bool occupied)
{
    if (s_pc_ctx.use_left) {
        action_4dof_set_back_occupied(DOF4_ARM_LEFT, occupied);
    }
    if (s_pc_ctx.use_right) {
        action_4dof_set_back_occupied(DOF4_ARM_RIGHT, occupied);
    }
}

/**
 * @brief 获取当前路径段的有效点数。
 *
 * 根据 mode（位姿/关节）和 pre_path（接近/撤离）选择正确的 count。
 * 单臂动作时 use_left 决定使用哪条手臂的路径数据。
 *
 * @param pre_path true=接近段, false=撤离段
 * @return 有效路径点数
 */
static uint8_t pc_action_4dof_current_path_count(bool pre_path)
{
    const uint8_t arm_index = s_pc_ctx.use_left ? 0U : 1U;
    if (s_pc_ctx.mode == PC_ACT4_MODE_POSE) {
        return pre_path ? s_pc_ctx.path.pose[arm_index].pre_count
                        : s_pc_ctx.path.pose[arm_index].post_count;
    }
    return pre_path ? s_pc_ctx.path.joint[arm_index].pre_count
                    : s_pc_ctx.path.joint[arm_index].post_count;
}

/**
 * @brief 下发当前路径点并执行双臂同步到位判断。
 *
 * 对每条使用中的手臂，设置当前路径索引对应的目标，然后检查是否到位。
 * 对双臂命令，只有左右臂都到达同一索引的路径点才返回 true，因此任何一侧
 * 较慢都会形成同步屏障，不会出现一只手先下降或先释放的情况。
 *
 * @param pre_path true=接近段, false=撤离段
 * @return true 表示所有使用中的手臂均已到达当前路径点
 */
static bool pc_action_4dof_drive_current_path_point(bool pre_path)
{
    bool all_reached = true;

    for (uint8_t arm_index = 0U; arm_index < PC_ACT4_ARM_COUNT; ++arm_index) {
        if (!pc_action_4dof_arm_used(&s_pc_ctx, arm_index)) {
            continue;  /* 跳过未参与本次动作的手臂 */
        }

        Dof4_Arm *arm = pc_action_4dof_arm(arm_index);
        if (s_pc_ctx.mode == PC_ACT4_MODE_POSE) {
            /* 位姿模式：设置 (x, y, z, pitch) 目标 */
            const Dof4_Pose *target = pre_path
                ? &s_pc_ctx.path.pose[arm_index].pre[s_pc_ctx.path_index]
                : &s_pc_ctx.path.pose[arm_index].post[s_pc_ctx.path_index];
            (void)Dof4_arm_set_target(arm, target->x, target->y,
                                      target->z, target->pitch);
            all_reached = pc_action_4dof_pose_reached(arm, target) && all_reached;
        } else {
            /* 关节角模式：设置各关节目标角度 */
            const PcAction4DOF_JointPoint *target = pre_path
                ? &s_pc_ctx.path.joint[arm_index].pre[s_pc_ctx.path_index]
                : &s_pc_ctx.path.joint[arm_index].post[s_pc_ctx.path_index];
            Dof4_JointState joints;
            memcpy(joints.q, target->q, sizeof(joints.q));
            (void)Dof4_arm_set_joint_target(arm, &joints);
            all_reached = pc_action_4dof_joint_reached(arm, target) && all_reached;
        }
    }
    return all_reached;
}

/**
 * @brief 控制所有使用中的手臂回归 IDLE 位姿，并检查是否已到达。
 *
 * 每周期调用 action_4dof_get_idle_pose 获取空闲位姿并下发，
 * 所有手臂都到位后才返回 true。
 *
 * @return true 表示所有工作臂均已到达 IDLE 位姿
 */
static bool pc_action_4dof_idle_reached(void)
{
    bool all_reached = true;

    for (uint8_t arm_index = 0U; arm_index < PC_ACT4_ARM_COUNT; ++arm_index) {
        if (!pc_action_4dof_arm_used(&s_pc_ctx, arm_index)) {
            continue;
        }

        const Dof4_ArmId arm_id =
            (arm_index == 0U) ? DOF4_ARM_LEFT : DOF4_ARM_RIGHT;
        Dof4_Arm *arm = pc_action_4dof_arm(arm_index);
        const Dof4_Pose idle = action_4dof_get_idle_pose(arm_id);
        (void)Dof4_arm_set_target(arm, idle.x, idle.y, idle.z, idle.pitch);
        all_reached = pc_action_4dof_pose_reached(arm, &idle) && all_reached;
    }
    return all_reached;
}

/**
 * @brief 正常完成动作，释放执行权并标记结束事件待发送。
 *
 * 在临界区内清除活跃标志、重置上下文，设置 completion_pending 以通知
 * 上层发送 0xCC 结束事件。
 */
static void pc_action_4dof_finish(void)
{
    taskENTER_CRITICAL();
    s_pc_ctx.active = false;
    s_pc_ctx.command = PC_ACT4_COMMAND_NONE;
    s_pc_ctx.state = PC_ACT4_STATE_IDLE;
    s_pc_ctx.timeout_ms = 0U;
    s_pc_ctx.path_index = 0U;
    s_pc_completion_pending = true;
    taskEXIT_CRITICAL();
}

/**
 * @brief 异常中止时启动强制归位流程。
 *
 * 此处不统一改阀门：
 * - release_committed=false 时，手臂吸盘仍保持开启，继续保载；
 * - release_committed=true 时，保留已经完成的释放和背部占用结果。
 *
 * 状态机进入 ABORT_RETURN_IDLE 后尝试一个超时周期归位。
 */
static void pc_action_4dof_begin_abort_return(void)
{
    pc_action_4dof_set_state(PC_ACT4_STATE_ABORT_RETURN_IDLE,
                             PC_ACT4_MOVE_TIMEOUT_MS);
}

/**
 * @brief 构造动态取放路径（位姿模式）。
 *
 * 根据 PC 下发的目标世界坐标和模板偏移量，自动生成三段式接近路径：
 *   1. 相对安全入口（target + entry_offset）
 *   2. 目标正上方（target.x, target.y, target.z + 垂直净空）
 *   3. 目标点（target）
 * 和两段式撤离路径：
 *   1. 目标正上方
 *   2. 相对安全出口（target + exit_offset）
 *
 * 生成后对所有路径点进行可达性校验。
 *
 * @param candidate     待填充的上下文（输出参数）
 * @param arm_index     手臂索引 (0=左, 1=右)
 * @param target_world  PC 下发的目标世界坐标
 * @param template_data 动态动作模板（包含偏移量和俯仰角）
 * @return true 表示路径构造成功且所有点可达
 */
static bool pc_action_4dof_build_dynamic_arm_path(
    PcAction4DOF_Context *candidate,
    uint8_t arm_index,
    const Dof4_Pose *target_world,
    const PcAction4DOF_DynamicTemplate *template_data)
{
    if (candidate == NULL || arm_index >= PC_ACT4_ARM_COUNT ||
        !pc_action_4dof_pose_finite(target_world) || template_data == NULL) {
        return false;
    }

    PcAction4DOF_PosePath *path = &candidate->path.pose[arm_index];
    const Dof4_Pose target = {
        .x = target_world->x,
        .y = target_world->y,
        .z = target_world->z,
        .pitch = template_data->target_pitch,
    };
    const Dof4_Pose overhead = {
        .x = target.x,
        .y = target.y,
        .z = target.z + PC_ACTION_4DOF_VERTICAL_CLEARANCE_M,
        .pitch = template_data->target_pitch,
    };

    /* 接近段：安全入口 -> 目标正上方 -> 目标 */
    path->pre[0] = (Dof4_Pose){
        target.x + template_data->entry_offset.x,
        target.y + template_data->entry_offset.y,
        target.z + template_data->entry_offset.z,
        template_data->entry_offset.pitch,
    };
    path->pre[1] = overhead;
    path->pre[2] = target;
    path->pre_count = 3U;

    /* 撤离段：目标正上方 -> 安全出口 */
    path->post[0] = overhead;
    path->post[1] = (Dof4_Pose){
        target.x + template_data->exit_offset.x,
        target.y + template_data->exit_offset.y,
        target.z + template_data->exit_offset.z,
        template_data->exit_offset.pitch,
    };
    path->post_count = 2U;

    /* 校验所有路径点的可达性 */
    Dof4_Arm *arm = pc_action_4dof_arm(arm_index);
    for (uint8_t i = 0U; i < path->pre_count; ++i) {
        if (!pc_action_4dof_pose_reachable(arm, &path->pre[i])) {
            return false;
        }
    }
    for (uint8_t i = 0U; i < path->post_count; ++i) {
        if (!pc_action_4dof_pose_reachable(arm, &path->post[i])) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 原子取得 PC/RC 共用执行权。
 *
 * 路径在临界区外构造和校验，最终只在短临界区内检查两个状态机并复制上下文。
 * 因此 PC 接收任务与 RC 控制任务即使同时触发，也只会有一方成功。
 *
 * 执行条件：
 * - PC 状态机不在活跃状态
 * - 无待发送的结束事件
 * - RC 状态机不在活跃状态
 *
 * @param candidate 已构造好的候选上下文
 * @return true 表示成功取得执行权，动作已启动
 */
static bool pc_action_4dof_claim(const PcAction4DOF_Context *candidate)
{
    bool accepted = false;

    if (candidate == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    if (!s_pc_ctx.active &&
        !s_pc_completion_pending &&
        !action_4dof_is_active()) {
        s_pc_ctx = *candidate;
        s_pc_ctx.active = true;
        accepted = true;
    }
    taskEXIT_CRITICAL();

    if (accepted) {
        /* 所有取放动作启动时先打开工作臂吸盘，确保接近过程中保持负压。 */
        pc_action_4dof_set_arm_suction(PC_ACT4_SUCTION_ON);
    }
    return accepted;
}

/**
 * @brief 初始化候选动作上下文，设置通用字段默认值。
 *
 * 将上下文结构体清零后填入命令、模式、手臂选择等基础信息，
 * 起始状态统一为 MOVE_PRE_PATH，超时使用默认的 PC_ACT4_MOVE_TIMEOUT_MS。
 *
 * @param candidate 待初始化的候选上下文（输出参数）
 * @param command   动作命令类型
 * @param mode      路径模式（位姿/关节角）
 * @param use_left  是否使用左臂
 * @param use_right 是否使用右臂
 */
static void pc_action_4dof_prepare_candidate(PcAction4DOF_Context *candidate,
                                              PcAction4DOF_Command command,
                                              PcAction4DOF_Mode mode,
                                              bool use_left,
                                              bool use_right)
{
    memset(candidate, 0, sizeof(*candidate));
    candidate->command = command;
    candidate->mode = mode;
    candidate->state = PC_ACT4_STATE_MOVE_PRE_PATH;
    candidate->enter_tick = HAL_GetTick();
    candidate->timeout_ms = PC_ACT4_MOVE_TIMEOUT_MS;
    candidate->use_left = use_left;
    candidate->use_right = use_right;
}

/* ======================== 公有 API 实现 ======================== */

void pc_action_4dof_init(void)
{
    taskENTER_CRITICAL();
    memset(&s_pc_ctx, 0, sizeof(s_pc_ctx));
    s_pc_ctx.command = PC_ACT4_COMMAND_NONE;
    s_pc_ctx.state = PC_ACT4_STATE_IDLE;
    s_pc_completion_pending = false;
    taskEXIT_CRITICAL();
}

bool pc_action_4dof_start_pick(Dof4_ArmId arm_id,
                               const Dof4_Pose *target_world)
{
    /* 校验手臂 ID 合法性 */
    if (arm_id != DOF4_ARM_LEFT && arm_id != DOF4_ARM_RIGHT) {
        return false;
    }

    PcAction4DOF_Context candidate;
    const uint8_t arm_index = (arm_id == DOF4_ARM_LEFT) ? 0U : 1U;
    pc_action_4dof_prepare_candidate(&candidate, PC_ACT4_COMMAND_PICK,
                                     PC_ACT4_MODE_POSE,
                                     arm_index == 0U, arm_index == 1U);
    if (!pc_action_4dof_build_dynamic_arm_path(
            &candidate, arm_index, target_world, &s_pick_templates[arm_index])) {
        return false;
    }
    return pc_action_4dof_claim(&candidate);
}

bool pc_action_4dof_start_place(Dof4_ArmId arm_id,
                                const Dof4_Pose *target_world)
{
    /* 校验手臂 ID 合法性 */
    if (arm_id != DOF4_ARM_LEFT && arm_id != DOF4_ARM_RIGHT) {
        return false;
    }

    PcAction4DOF_Context candidate;
    const uint8_t arm_index = (arm_id == DOF4_ARM_LEFT) ? 0U : 1U;
    pc_action_4dof_prepare_candidate(&candidate, PC_ACT4_COMMAND_PLACE,
                                     PC_ACT4_MODE_POSE,
                                     arm_index == 0U, arm_index == 1U);
    if (!pc_action_4dof_build_dynamic_arm_path(
            &candidate, arm_index, target_world, &s_place_templates[arm_index])) {
        return false;
    }
    return pc_action_4dof_claim(&candidate);
}

bool pc_action_4dof_start_put_back(Dof4_ArmId arm_id)
{
    /* 校验手臂 ID 合法性 */
    if (arm_id != DOF4_ARM_LEFT && arm_id != DOF4_ARM_RIGHT) {
        return false;
    }

    PcAction4DOF_Context candidate;
    const uint8_t arm_index = (arm_id == DOF4_ARM_LEFT) ? 0U : 1U;
    pc_action_4dof_prepare_candidate(&candidate, PC_ACT4_COMMAND_PUT_BACK,
                                     PC_ACT4_MODE_JOINT,
                                     arm_index == 0U, arm_index == 1U);
    candidate.path.joint[arm_index] = s_put_back_paths[arm_index];
    if (!pc_action_4dof_joint_path_valid(
            arm_index, &candidate.path.joint[arm_index])) {
        return false;
    }
    return pc_action_4dof_claim(&candidate);
}

bool pc_action_4dof_start_get_back(Dof4_ArmId arm_id)
{
    /* 校验手臂 ID 合法性 */
    if (arm_id != DOF4_ARM_LEFT && arm_id != DOF4_ARM_RIGHT) {
        return false;
    }

    PcAction4DOF_Context candidate;
    const uint8_t arm_index = (arm_id == DOF4_ARM_LEFT) ? 0U : 1U;
    pc_action_4dof_prepare_candidate(&candidate, PC_ACT4_COMMAND_GET_BACK,
                                     PC_ACT4_MODE_JOINT,
                                     arm_index == 0U, arm_index == 1U);
    candidate.path.joint[arm_index] = s_get_back_paths[arm_index];
    if (!pc_action_4dof_joint_path_valid(
            arm_index, &candidate.path.joint[arm_index])) {
        return false;
    }
    return pc_action_4dof_claim(&candidate);
}

bool pc_action_4dof_start_dual_pick(const Dof4_Pose *left_target_world,
                                    const Dof4_Pose *right_target_world)
{
    PcAction4DOF_Context candidate;
    pc_action_4dof_prepare_candidate(&candidate, PC_ACT4_COMMAND_DUAL_PICK,
                                     PC_ACT4_MODE_POSE, true, true);
    /* 分别为左右臂构造动态路径，任一失败则整体拒绝 */
    if (!pc_action_4dof_build_dynamic_arm_path(
            &candidate, 0U, left_target_world, &s_dual_pick_templates[0]) ||
        !pc_action_4dof_build_dynamic_arm_path(
            &candidate, 1U, right_target_world, &s_dual_pick_templates[1])) {
        return false;
    }
    return pc_action_4dof_claim(&candidate);
}

bool pc_action_4dof_start_dual_put_back(void)
{
    PcAction4DOF_Context candidate;
    pc_action_4dof_prepare_candidate(&candidate,
                                     PC_ACT4_COMMAND_DUAL_PUT_BACK,
                                     PC_ACT4_MODE_JOINT, true, true);
    candidate.path.joint[0] = s_dual_put_back_paths[0];
    candidate.path.joint[1] = s_dual_put_back_paths[1];
    if (!pc_action_4dof_joint_path_valid(0U, &candidate.path.joint[0]) ||
        !pc_action_4dof_joint_path_valid(1U, &candidate.path.joint[1])) {
        return false;
    }
    return pc_action_4dof_claim(&candidate);
}

bool pc_action_4dof_is_active(void)
{
    return s_pc_ctx.active;
}

bool pc_action_4dof_completion_pending(void)
{
    return s_pc_completion_pending;
}

void pc_action_4dof_completion_acknowledge(void)
{
    taskENTER_CRITICAL();
    s_pc_completion_pending = false;
    taskEXIT_CRITICAL();
}

/**
 * @brief 周期推进 PC 动作状态机。
 *
 * 状态转换总图（正常流程）：
 * @code
 *   IDLE
 *     │
 *     ▼  (pc_action_4dof_claim 成功)
 *   MOVE_PRE_PATH ───超时───┐
 *     │ 逐点运动               │
 *     ▼ 全部到位               │
 *   OPERATION_HOLD            │
 *     │ 首次：执行阀门操作      │
 *     │ 等待保持时间           │
 *     │ (背部命令需二次保持)    │
 *     ▼ 保持结束               │
 *   [OPERATION_SECOND_HOLD]   │
 *     │ (仅背部命令)           │
 *     ▼                        │
 *   MOVE_POST_PATH ───超时───┤
 *     │ 逐点运动               │
 *     ▼ 全部到位               │
 *   RETURN_IDLE      ───超时───┤
 *     │ 回归空闲位姿            │
 *     ▼ 到位                   │
 *   IDLE_HOLD                 │
 *     │ 短暂保持                │
 *     ▼ 结束                   │
 *   IDLE (active=false) ◄─────┘
 * @endcode
 *
 * 任何运动阶段超时转而进入 ABORT_RETURN_IDLE 尝试紧急归位。
 * 成功和超时终止都会产生一次 0xCC 结束事件（通过 completion_pending 标记）。
 */
void pc_action_4dof_loop(void)
{
    /* 状态机未激活时直接返回 */
    if (!s_pc_ctx.active) {
        return;
    }

    switch (s_pc_ctx.state) {
    /* ================================================================
     * 接近段路径运动
     *
     * 沿 pre 路径逐点运动，每到达一个路点后推进 path_index。
     * 最后一个路点到达后进入 OPERATION_HOLD 执行阀门操作。
     * 接近阶段持续确保手臂吸盘开启，即使启动和控制任务并发也不会漏开。
     * ================================================================ */
    case PC_ACT4_STATE_MOVE_PRE_PATH:
        pc_action_4dof_set_arm_suction(PC_ACT4_SUCTION_ON);
        if (pc_action_4dof_drive_current_path_point(true)) {
            ++s_pc_ctx.path_index;
            if (s_pc_ctx.path_index >= pc_action_4dof_current_path_count(true)) {
                /* 所有接近段路点运动完毕，切换到操作保持阶段 */
                s_pc_ctx.path_index = 0U;
                s_pc_ctx.operation_first_step_done = false;
                pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_HOLD, 0U);
            } else {
                /* 继续下一路点，重置超时 */
                pc_action_4dof_set_state(PC_ACT4_STATE_MOVE_PRE_PATH,
                                         PC_ACT4_MOVE_TIMEOUT_MS);
            }
        } else if (pc_action_4dof_timed_out()) {
            pc_action_4dof_begin_abort_return();
        }
        break;

    /* ================================================================
     * 操作保持
     *
     * 该状态分为两个阶段：
     *   第一阶段（首次进入）：根据命令类型执行对应的阀门操作并设置等待时间。
     *   第二阶段（超时后）：根据命令类型决定后续流程。
     *
     * 保持期间机械臂保持末端目标不动（路径最后一点即为操作目标），
     * 等待吸附/释放稳定。
     *
     * 命令分支说明：
     *   PICK/DUAL_PICK      → 保持当前姿态，等待吸取稳定
     *   PLACE               → 关闭手臂吸盘（释放），等待释放完成
     *   PUT_BACK/DUAL_PUT_BACK → 先开背部吸盘，等待背部吸附稳定后
     *                            再关手臂吸盘 + 提交背部占用
     *   GET_BACK            → 先等手臂吸稳，再关背部吸盘 + 清除背部占用
     * ================================================================ */
    case PC_ACT4_STATE_OPERATION_HOLD:
        if (!s_pc_ctx.operation_first_step_done) {
            /* ---- 第一阶段：执行阀门操作 ---- */
            s_pc_ctx.operation_first_step_done = true;

            if (s_pc_ctx.command == PC_ACT4_COMMAND_PICK ||
                s_pc_ctx.command == PC_ACT4_COMMAND_DUAL_PICK) {
                /* 取块：吸盘已开启（启动时已打开），只需等待稳定吸附 */
                pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_HOLD,
                                         PC_ACT4_PICK_HOLD_MS);
            } else if (s_pc_ctx.command == PC_ACT4_COMMAND_PLACE) {
                /* 放块：关闭手臂吸盘释放物块 */
                pc_action_4dof_set_arm_suction(PC_ACT4_SUCTION_OFF);
                s_pc_ctx.release_committed = true;
                pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_HOLD,
                                         PC_ACT4_EXTERNAL_RELEASE_HOLD_MS);
            } else if (s_pc_ctx.command == PC_ACT4_COMMAND_PUT_BACK ||
                       s_pc_ctx.command == PC_ACT4_COMMAND_DUAL_PUT_BACK) {
                /* 放块到背部：先开启背部吸盘准备接收 */
                pc_action_4dof_set_target_back_suction(PC_ACT4_SUCTION_ON);
                pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_HOLD,
                                         PC_ACT4_BACK_PRE_RELEASE_HOLD_MS);
            } else if (s_pc_ctx.command == PC_ACT4_COMMAND_GET_BACK) {
                /* 从背部取块：等待手臂吸盘完全吸附物块 */
                pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_HOLD,
                                         PC_ACT4_BACK_GET_ARM_HOLD_MS);
            } else {
                /* 未知命令 → 异常中止 */
                pc_action_4dof_begin_abort_return();
            }
        } else if (pc_action_4dof_timed_out()) {
            /* ---- 第二阶段：保持时间到，执行后续动作 ---- */
            if (s_pc_ctx.command == PC_ACT4_COMMAND_PUT_BACK ||
                s_pc_ctx.command == PC_ACT4_COMMAND_DUAL_PUT_BACK) {
                /*
                 * 背部放置：背部已吸附稳定，手臂释放物块并提交背部占用。
                 * 按顺序：关手臂吸盘 → 标记背部占用 → 标记 release_committed
                 */
                pc_action_4dof_set_arm_suction(PC_ACT4_SUCTION_OFF);
                pc_action_4dof_apply_back_state(true);
                s_pc_ctx.release_committed = true;
                pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_SECOND_HOLD,
                                         PC_ACT4_BACK_POST_RELEASE_HOLD_MS);
            } else if (s_pc_ctx.command == PC_ACT4_COMMAND_GET_BACK) {
                /*
                 * 背部取块：手臂已吸稳，关闭来源背部吸盘并清除占用标记。
                 * 按顺序：关背部吸盘 → 清除背部占用 → 标记 release_committed
                 * 先吸后放的顺序确保交接瞬间不掉块。
                 */
                pc_action_4dof_set_target_back_suction(PC_ACT4_SUCTION_OFF);
                pc_action_4dof_apply_back_state(false);
                s_pc_ctx.release_committed = true;
                pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_SECOND_HOLD,
                                         PC_ACT4_BACK_RELEASE_HOLD_MS);
            } else {
                /* 取块/放块：无需二次保持，直接进入撤离段 */
                s_pc_ctx.path_index = 0U;
                pc_action_4dof_set_state(PC_ACT4_STATE_MOVE_POST_PATH,
                                         PC_ACT4_MOVE_TIMEOUT_MS);
            }
        }
        break;

    /* ================================================================
     * 二次保持（仅背部操作使用）
     *
     * PUT_BACK/DUAL_PUT_BACK：等待手臂释放后残留物块稳定。
     * GET_BACK：等待背部吸盘关闭后完全脱离。
     * ================================================================ */
    case PC_ACT4_STATE_OPERATION_SECOND_HOLD:
        if (pc_action_4dof_timed_out()) {
            s_pc_ctx.path_index = 0U;
            pc_action_4dof_set_state(PC_ACT4_STATE_MOVE_POST_PATH,
                                     PC_ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    /* ================================================================
     * 撤离段路径运动
     *
     * 沿 post 路径逐点运动，逻辑与 MOVE_PRE_PATH 相同但方向相反。
     * 全部路点到达后进入 RETURN_IDLE 回归空闲位姿。
     * ================================================================ */
    case PC_ACT4_STATE_MOVE_POST_PATH:
        if (pc_action_4dof_drive_current_path_point(false)) {
            ++s_pc_ctx.path_index;
            if (s_pc_ctx.path_index >= pc_action_4dof_current_path_count(false)) {
                /* 所有撤离段路点运动完毕，回归空闲位姿 */
                s_pc_ctx.path_index = 0U;
                pc_action_4dof_set_state(PC_ACT4_STATE_RETURN_IDLE,
                                         PC_ACT4_MOVE_TIMEOUT_MS);
            } else {
                /* 继续下一路点 */
                pc_action_4dof_set_state(PC_ACT4_STATE_MOVE_POST_PATH,
                                         PC_ACT4_MOVE_TIMEOUT_MS);
            }
        } else if (pc_action_4dof_timed_out()) {
            pc_action_4dof_begin_abort_return();
        }
        break;

    /* ================================================================
     * 回归 IDLE 空闲位姿
     *
     * 将各工作臂引导至系统预设的空闲位姿，到位后短暂保持即可完成动作。
     * ================================================================ */
    case PC_ACT4_STATE_RETURN_IDLE:
        if (pc_action_4dof_idle_reached()) {
            pc_action_4dof_set_state(PC_ACT4_STATE_IDLE_HOLD,
                                     PC_ACT4_IDLE_HOLD_MS);
        } else if (pc_action_4dof_timed_out()) {
            pc_action_4dof_begin_abort_return();
        }
        break;

    /* ================================================================
     * 异常中止 - 强制归位
     *
     * 当任何运动阶段超时时进入此状态。只尝试一个超时周期归位到 IDLE，
     * 无论是否最终到位（超时或到位），都必须释放执行权。
     * ================================================================ */
    case PC_ACT4_STATE_ABORT_RETURN_IDLE:
        if (pc_action_4dof_idle_reached() || pc_action_4dof_timed_out()) {
            pc_action_4dof_set_state(PC_ACT4_STATE_IDLE_HOLD,
                                     PC_ACT4_IDLE_HOLD_MS);
        }
        break;

    /* ================================================================
     * IDLE 保持 & 结束
     *
     * IDLE_HOLD：到达空闲位姿后短暂保持，确保机械臂稳定后释放执行权。
     * 保持时间到后调用 pc_action_4dof_finish 清除活跃标志并标记结束事件。
     * ================================================================ */
    case PC_ACT4_STATE_IDLE_HOLD:
        if (pc_action_4dof_timed_out()) {
            pc_action_4dof_finish();
        }
        break;

    /* ================================================================
     * 异常状态处理
     *
     * IDLE 状态不应在此处到达（正常情况下不会被调度），
     * 若因逻辑错误进入此分支则执行异常中止。
     * ================================================================ */
    case PC_ACT4_STATE_IDLE:
    default:
        pc_action_4dof_begin_abort_return();
        break;
    }
}
