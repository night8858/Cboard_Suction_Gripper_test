/**
 * @file    pc_action_executor_4dof.c
 * @brief   PC 下发动作专用 4DOF 状态机
 *
 * 本模块与 action_scheduler_4dof 的预设动作状态机相互独立。两者只共享
 * 机械臂底层目标接口、电磁阀接口和背部物块占用状态，并通过临界区保证同一时刻
 * 只有一个状态机取得执行权。
 *
 * 动态取放路径：
 *   当前 TCP -> 目标正上方悬停点（使用模板 approach pitch）-> 垂直下降至 PC 目标 TCP
 *   -> 吸取/释放 -> 撤离回悬停点 -> 自适应 IDLE
 *
 * 悬停点 xyz 固定在目标正上方，pitch 采用已调模板姿态，避免中间点因
 * 强制目标俯仰角导致 IK 不可达；下降段仅改变 Z 轴到达目标点。
 *
 * 背部固定路径使用关节角序列。路径数组和有效长度分离，后续实机调试需要增加
 * 中间点时，只需在对应数组中追加关节角并调整 count。
 *
 * 超时原则：
 *   - 运动阶段未到位：立即终止后续吸取/释放操作，尝试回自适应 IDLE。
 *   - 释放前终止：保持工作臂电磁阀打开，不修改背部占用状态，避免掉块。
 *   - 释放后终止：保留已经执行的阀门和背部占用结果。
 *   - 成功和超时终止都会产生一次 0xCC 结束事件；0xCC 不区分结果。
 */ 

#include "pc_action_executor_4dof.h"

#include "FreeRTOS.h"
#include "action_scheduler_4dof.h"
#include "command_decode_4dof.h"
#include "pneumatic_control.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "variables.h"

#include <math.h>
#include <string.h>

/* ======================== 电磁阀继电器映射 ======================== */
#define PC_ACT4_RELAY_LEFT_ARM   0U   /**< 左手臂吸盘继电器索引 */
#define PC_ACT4_RELAY_RIGHT_ARM  1U   /**< 右手臂吸盘继电器索引 */
#define PC_ACT4_RELAY_LEFT_BACK  2U   /**< 左背部吸盘继电器索引 */
#define PC_ACT4_RELAY_RIGHT_BACK 3U   /**< 右背部吸盘继电器索引 */
#define PC_ACT4_VALVE_OPEN       1U   /**< 电磁阀打开，系统默认状态 */
#define PC_ACT4_VALVE_CLOSED     0U   /**< 电磁阀关闭，用于释放/交接节点 */

/* ======================== 运动到位公差 ======================== */
/** 到位和超时参数沿用原 4DOF 调度器的量级，便于实机统一调试。 */
#define PC_ACT4_MOVE_TIMEOUT_MS              2500U   /**< 单段路径运动超时 (ms) */
#define PC_ACT4_POSE_POS_TOL_M                0.05f  /**< 位姿位置到位公差 (m) */
#define PC_ACT4_POSE_PITCH_TOL_RAD            0.03f  /**< 位姿俯仰角到位公差 (rad) */
#define PC_ACT4_JOINT_TOL_RAD                  0.06f /**< 关节角到位公差 (rad) */
#define PC_ACT4_BACK_SERVO_SPEED              6000U  /**< PC 背部固定关节动作专用舵机速度 */
#define PC_ACT4_DYNAMIC_HOLD_PRE_INDEX         0U    /**< 动态路径到达悬停点后等待（简化后仅一个中间点） */
#define PC_ACT4_DYNAMIC_HOVER_CLEARANCE_M      0.06f /**< 动态取/放块目标上方悬停高度，需明显大于到位公差 */

/* ======================== PC 动作阀门时序，可按实机效果集中调节 ======================== */
#define PC_ACT4_DELAY_DYNAMIC_PICK_HOLD_MS       1500U /**< 动态取块：目标点吸附稳定等待 */
#define PC_ACT4_DELAY_DYNAMIC_PLACE_RELEASE_MS   1600U /**< 动态放块：关闭工作臂阀后的释放等待 */
#define PC_ACT4_DELAY_DYNAMIC_TARGET_SETTLE_MS    800U /**< 动态取/放块：到目标点后的稳定等待 */
#define PC_ACT4_DELAY_BACK_PRE_RELEASE_MS        1600U /**< 放到背部：打开背部阀后的预吸附等待 */
#define PC_ACT4_DELAY_BACK_POST_RELEASE_MS       1600U /**< 放到背部：关闭工作臂阀后的交接等待 */
#define PC_ACT4_DELAY_BACK_GET_ARM_HOLD_MS       1600U /**< 从背部取回：工作臂吸附稳定等待 */
#define PC_ACT4_DELAY_BACK_SOURCE_RELEASE_MS      800U /**< 从背部取回：关闭来源背部阀后的释放等待 */
#define PC_ACT4_DELAY_IDLE_HOLD_MS                100U /**< 到达 IDLE 后的结束保持 */
#define PC_ACT4_DELAY_DYNAMIC_HOVER_HOLD_MS      60U /**< 动态路径悬停点保持 */

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
 * 每个值对应一种由 PC 下发的动作类型，与 0x11/0x12/0x14/0x15/0x21/0x22/0x23/0x24
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
    PC_ACT4_COMMAND_DUAL_PLACE,     /**< 0x23: 双臂动态放块 */
    PC_ACT4_COMMAND_DUAL_GET_BACK,  /**< 0x24: 双臂从背部取块 */
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
 * @brief PC 状态机状态枚举（仅用于位姿模式 PICK/PLACE）
 */
typedef enum {
    PC_ACT4_STATE_IDLE = 0,             /**< 空闲态 */
    PC_ACT4_STATE_MOVE_PRE_PATH,        /**< 位姿: 悬停→目标 */
    PC_ACT4_STATE_PRE_WAYPOINT_HOLD,    /**< 位姿: 悬停点停留 */
    PC_ACT4_STATE_TARGET_SETTLE_HOLD,   /**< 位姿: 目标点稳定停留 */
    PC_ACT4_STATE_OPERATION_HOLD,       /**< 操作保持 */
    PC_ACT4_STATE_MOVE_POST_PATH,       /**< 位姿: 撤离段 */
    PC_ACT4_STATE_RETURN_IDLE,          /**< 位姿: 回归 POSE idle */
    PC_ACT4_STATE_ABORT_RETURN_IDLE,    /**< 超时中止归位 */
    PC_ACT4_STATE_IDLE_HOLD,            /**< IDLE 保持 */
} PcAction4DOF_State;

/** @brief 关节模式子状态（完全参照 action_scheduler 的 action_4dof_substate_e） */
typedef enum {
    PC_ACT4_JNT_APPROACH = 0,        /**< GET_BACK: 预就位 */
    PC_ACT4_JNT_PLACE_APPROACH,      /**< PUT_BACK: 放置预就位 */
    PC_ACT4_JNT_INTERMEDIATE,        /**< 中间途经点 wp1/wp2 */
    PC_ACT4_JNT_GRAB,                /**< GET_BACK: 操作目标点 */
    PC_ACT4_JNT_PLACE,               /**< PUT_BACK: 操作目标点 */
    PC_ACT4_JNT_SUCTION_GRAB,        /**< 吸取控制 */
    PC_ACT4_JNT_SUCTION_PLACE,       /**< 释放控制 */
    PC_ACT4_JNT_RETREAT,             /**< 撤离 */
    PC_ACT4_JNT_COMPLETE,            /**< 归位(关节→POSE idle) */
} PcAction4DOF_JointSubstate;

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
 * PC 只提供最终 xyz；当前动态路径保留 target 正上方中间点，
 * 其中 entry_offset.pitch 作为中间点的绝对俯仰角使用。
 * 每个手臂（左/右）各有一个独立模板，以适应双侧不同的工作空间约束。
 */
typedef struct {
    Dof4_Pose entry_offset;  /**< 入口偏移量；pitch 为中间点绝对俯仰角 */
    Dof4_Pose exit_offset;   /**< 出口偏移量；pitch 保留给后续撤离路径调参 */
    float target_pitch;      /**< 操作目标点的俯仰角，单位 rad */
    float vertical_clearance_m; /**< 中间点相对 target 的 z 向上偏移量，单位 m */
} PcAction4DOF_DynamicTemplate;

/**
 * @brief PC 动作状态机运行时上下文。
 *
 * 所有字段在 claim 时一次写入，运行期间由 pc_action_4dof_loop 维护。
 * 临界区保护保证与预设动作状态机互斥访问。
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
    bool operation_phase_started;       /**< 位姿模式操作保持阶段是否已开始 */
    bool joint_phase_started;           /**< 关节模式当前阀门保持阶段是否已开始 */
    bool error_reported;                /**< 本次动作失败事件是否已上报给 LED。 */
    PcAction4DOF_JointSubstate jnt_substate; /**< 关节模式子状态 */
    union {
        PcAction4DOF_PosePath pose[PC_ACT4_ARM_COUNT];   /**< 位姿路径（双臂各一份） */
        PcAction4DOF_JointPath joint[PC_ACT4_ARM_COUNT]; /**< 关节角路径（双臂各一份） */
    } path;                             /**< 路径数据，根据 mode 选择访问 pose 或 joint */
} PcAction4DOF_Context;

/**
 * @brief PC 动作待启动请求。
 *
 * USB/update 任务只负责写入这一条请求；真正的路径构建与 IK 校验在
 * pc_action_4dof_loop() 所在的 4DOF 控制任务中完成，避免启动位姿尚未完成时
 * 使用过期或未初始化的 current_pose。
 */
typedef struct {
    bool valid;                         /**< 是否有待处理请求 */
    PcAction4DOF_Command command;       /**< 请求动作类型 */
    bool use_left;                      /**< 请求是否使用左臂 */
    bool use_right;                     /**< 请求是否使用右臂 */
    Dof4_Pose target[PC_ACT4_ARM_COUNT];/**< 动态动作目标；背部动作忽略 */
} PcAction4DOF_PendingRequest;

/* ======================== 外部全局变量引用 ======================== */
extern Dof4_Arm g_dof4_arm_left;   /**< 左机械臂实例（定义于 Dof4_Arm.c） */
extern Dof4_Arm g_dof4_arm_right;  /**< 右机械臂实例（定义于 Dof4_Arm.c） */

/* ======================== 模块内部静态变量 ======================== */
static PcAction4DOF_Context s_pc_ctx;            /**< PC 状态机运行时上下文 */
static volatile bool s_pc_completion_pending;    /**< 是否有待发送的 0xCC 结束事件 */
static PcAction4DOF_PendingRequest s_pc_pending; /**< 待启动 PC 动作请求 */
static uint16_t s_pc_saved_servo_speed[PC_ACT4_ARM_COUNT]; /**< 背部动作速度覆盖前的原速度 */
static bool s_pc_servo_speed_overridden[PC_ACT4_ARM_COUNT]; /**< 对应手臂速度是否被覆盖 */

static void pc_action_4dof_report_error_event(void)
{
    taskENTER_CRITICAL();
    g_pc_action_error_event_count++;
    taskEXIT_CRITICAL();
}

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
        .target_pitch = -1.48f,
        .vertical_clearance_m = PC_ACT4_DYNAMIC_HOVER_CLEARANCE_M,
    },
    /* 右臂 */
    {
        .entry_offset = {0.10f, 0.15f, 0.40f, -0.60f},
        .exit_offset  = {0.10f, 0.07f, 0.40f, -0.30f},
        .target_pitch = -1.48f,
        .vertical_clearance_m = PC_ACT4_DYNAMIC_HOVER_CLEARANCE_M,
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
        .target_pitch = -1.48f,
        .vertical_clearance_m = PC_ACT4_DYNAMIC_HOVER_CLEARANCE_M,
    },
    /* 右臂 */
    {
        .entry_offset = {0.10f, 0.15f, 0.30f, -0.60f},
        .exit_offset  = {0.10f, 0.07f, 0.385f, -0.30f},
        .target_pitch = -1.48f,
        .vertical_clearance_m = PC_ACT4_DYNAMIC_HOVER_CLEARANCE_M,
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
        .entry_offset = {-0.075f, -0.10f, 0.49f, -0.30f},
        .exit_offset  = {-0.175f, -0.15f, 0.44f, -0.50f},
        .target_pitch = -1.48f,
        .vertical_clearance_m = PC_ACT4_DYNAMIC_HOVER_CLEARANCE_M,
    },
    /* 右臂 */
    {
        .entry_offset = {-0.075f, 0.10f, 0.52f, -1.00f},
        .exit_offset  = {-0.175f, 0.15f, 0.43f, -0.50f},
        .target_pitch = -1.48f,
        .vertical_clearance_m = PC_ACT4_DYNAMIC_HOVER_CLEARANCE_M,
    },
};

#define PC_JOINT_POINT(j1_, j2_, j3_, j4_) \
    { .q = {(j1_), (j2_), (j3_), (j4_)} }

/* ======================== 背部固定路径 - 关节角序列 ======================== */
/**
 * @brief 单臂放块到背部路径（0x14）。
 *
 * 关节角保留实机已调的 waypoint_2、target、retreat、complete 四点。
 * 左臂 J1 向正方向旋转至左后方，右臂 J1 向负方向旋转至右后方。
 * pre[0]=waypoint_2, pre[1]=target
 * post[0]=retreat, post[1]=complete（关节归位点，之后切 POSE 回 IDLE）
 */
static const PcAction4DOF_JointPath s_put_back_paths[PC_ACT4_ARM_COUNT] = {
    /* 左臂：J1 向正方向旋转到左后方（匹配 ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_BACK） */
    {
        .pre = {
            PC_JOINT_POINT( 2.800f, 1.50f, -1.20f, -1.680f),   /* waypoint_2 */
            PC_JOINT_POINT( 2.830f, 1.60f, -1.74f, -1.26f),   /* target */
        },
        .post = {
            PC_JOINT_POINT( 2.000f, 1.50f, -1.20f, -1.463f),   /* retreat */
            PC_JOINT_POINT( 1.0f, 1.50f, -1.70f, -1.463f),   /* complete */
        },
        .pre_count = 2U,
        .post_count = 2U,
    },
    /* 右臂：J1 向负方向旋转到右后方（匹配 ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_BACK） */
    {
        .pre = {
            PC_JOINT_POINT(-2.800f, 1.50f, -1.20f, -1.680f),   /* waypoint_2 */
            PC_JOINT_POINT( -2.860f, 1.65f, -1.79f, -1.31f),   /* target */
        },
        .post = {
            PC_JOINT_POINT(-2.000f, 1.50f, -1.20f, -1.463f),   /* retreat */
            PC_JOINT_POINT( -1.0f, 1.50f, -1.70f, -1.463f),   /* complete */
        },
        .pre_count = 2U,
        .post_count = 2U,
    },
};

/**
 * @brief 双臂同时放块到背部路径（0x22）。
 *
 * 匹配 ACTION_BLOCK_PLACE_BACK，双臂模式下展开角度更大（J1=±3.10）。
 * pre[0]=waypoint_2, pre[1]=target
 * post[0]=retreat, post[1]=complete
 */
static const PcAction4DOF_JointPath s_dual_put_back_paths[PC_ACT4_ARM_COUNT] = {
    /* 左臂 */
    {
        .pre = {
            PC_JOINT_POINT( 2.800f, 1.50f, -1.20f, -1.680f),   /* waypoint_2 */
            PC_JOINT_POINT( 2.830f, 1.60f, -1.74f, -1.26f),   /* target */
        },
        .post = {
            PC_JOINT_POINT( 2.800f, 1.50f, -1.20f, -1.463f),   /* retreat */
            PC_JOINT_POINT( 1.0f, 1.50f, -1.70f, -1.463f),   /* complete */
        },
        .pre_count = 2U,
        .post_count = 2U,
    },
    /* 右臂 */
    {
        .pre = {
            PC_JOINT_POINT(-2.800f, 1.50f, -1.20f, -1.680f),   /* waypoint_2 */
            PC_JOINT_POINT( -2.860f, 1.65f, -1.79f, -1.31f),   /* target */
        },
        .post = {
            PC_JOINT_POINT(-2.800f, 1.50f, -1.20f, -1.463f),   /* retreat */
            PC_JOINT_POINT( -1.0f, 1.50f, -1.70f, -1.463f),   /* complete */
        },
        .pre_count = 2U,
        .post_count = 2U,
    },
};

/**
 * @brief 单臂从背部取块路径（0x15）。
 *
 * 匹配 ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_LEFT_ARM（左）和
 * ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_RIGHT_ARM（右）。
 * pre[0]=waypoint_2, pre[1]=target
 * post[0]=retreat, post[1]=complete
 */
static const PcAction4DOF_JointPath s_get_back_paths[PC_ACT4_ARM_COUNT] = {
    /* 左臂 */
    {
        .pre = {
            PC_JOINT_POINT(2.800f, 1.50f, -1.00f, -1.680f),   /* waypoint_2 */
            PC_JOINT_POINT( 2.830f, 1.60f, -1.74f, -1.26f),   /* target */
        },
        .post = {
            PC_JOINT_POINT(2.800f, 1.50f, -1.48f, -1.463f),   /* retreat */
            PC_JOINT_POINT(1.042f, 1.50f, -1.70f, -1.463f),   /* complete */
        },
        .pre_count = 2U,
        .post_count = 2U,
    },
    /* 右臂 */
    {
        .pre = {
            PC_JOINT_POINT(-2.800f, 1.50f, -1.00f, -1.680f),   /* waypoint_2 */
            PC_JOINT_POINT( -2.860f, 1.65f, -1.79f, -1.31f),   /* target */
        },
        .post = {
            PC_JOINT_POINT(-2.800f, 1.50f, -1.48f, -1.463f),   /* retreat */
            PC_JOINT_POINT(-1.042f, 1.50f, -1.70f, -1.463f),   /* complete */
        },
        .pre_count = 2U,
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
 * @brief 校验固定关节路径是否可转换为舵机可执行目标。
 *
 * J1 允许 Dof4_angle_to_servo() 选择等效角；J2/J3/J4 由底层转换函数
 * 统一处理关节限位和舵机限位。这里不再提前用 joint_min/max 拒绝，
 * 避免背部固定动作的大角度 J1 被误判为非法。
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
 * @brief 将固定关节路径归一化为实际舵机可执行角度。
 *
 * 背部固定路径来自实机调参，可能包含 J1 等效大角度或略超关节软限位的
 * 角度。先转换为舵机步进，再从步进反算实际关节角，确保后续到位判断
 * 与真正下发给舵机的目标一致。
 */
static bool pc_action_4dof_normalize_joint_path(
    uint8_t arm_index,
    PcAction4DOF_JointPath *path)
{
    if (!pc_action_4dof_joint_path_valid(arm_index, path)) {
        return false;
    }

    const Dof4_Arm *arm = pc_action_4dof_arm(arm_index);
    for (uint8_t segment = 0U; segment < 2U; ++segment) {
        PcAction4DOF_JointPoint *points =
            (segment == 0U) ? path->pre : path->post;
        const uint8_t count =
            (segment == 0U) ? path->pre_count : path->post_count;

        for (uint8_t point_index = 0U; point_index < count; ++point_index) {
            for (uint8_t joint_index = 0U;
                 joint_index < DOF4_JOINT_COUNT;
                 ++joint_index) {
                int16_t servo_pos = 0;
                float executable_angle = points[point_index].q[joint_index];
                if (Dof4_angle_to_servo(arm,
                                        joint_index,
                                        points[point_index].q[joint_index],
                                        &servo_pos) != DOF4_STATUS_OK ||
                    Dof4_servo_to_angle(arm,
                                        joint_index,
                                        servo_pos,
                                        &executable_angle) != DOF4_STATUS_OK ||
                    !isfinite(executable_angle)) {
                    return false;
                }
                points[point_index].q[joint_index] = executable_angle;
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

static bool pc_action_4dof_is_dynamic_pick_command(PcAction4DOF_Command command)
{
    return command == PC_ACT4_COMMAND_PICK ||
           command == PC_ACT4_COMMAND_DUAL_PICK;
}

static bool pc_action_4dof_is_dynamic_place_command(PcAction4DOF_Command command)
{
    return command == PC_ACT4_COMMAND_PLACE ||
           command == PC_ACT4_COMMAND_DUAL_PLACE;
}

static bool pc_action_4dof_is_back_put_command(PcAction4DOF_Command command)
{
    return command == PC_ACT4_COMMAND_PUT_BACK ||
           command == PC_ACT4_COMMAND_DUAL_PUT_BACK;
}

static bool pc_action_4dof_is_back_get_command(PcAction4DOF_Command command)
{
    return command == PC_ACT4_COMMAND_GET_BACK ||
           command == PC_ACT4_COMMAND_DUAL_GET_BACK;
}

static bool pc_action_4dof_is_back_command(PcAction4DOF_Command command)
{
    return pc_action_4dof_is_back_put_command(command) ||
           pc_action_4dof_is_back_get_command(command);
}

static void pc_action_4dof_restore_servo_speed(void)
{
    for (uint8_t arm_index = 0U; arm_index < PC_ACT4_ARM_COUNT; ++arm_index) {
        if (!s_pc_servo_speed_overridden[arm_index]) {
            continue;
        }

        pc_action_4dof_arm(arm_index)->cfg.servo_speed =
            s_pc_saved_servo_speed[arm_index];
        s_pc_saved_servo_speed[arm_index] = 0U;
        s_pc_servo_speed_overridden[arm_index] = false;
    }
}

static void pc_action_4dof_apply_back_servo_speed(void)
{
    if (!pc_action_4dof_is_back_command(s_pc_ctx.command)) {
        return;
    }

    for (uint8_t arm_index = 0U; arm_index < PC_ACT4_ARM_COUNT; ++arm_index) {
        if (!pc_action_4dof_arm_used(&s_pc_ctx, arm_index)) {
            continue;
        }

        Dof4_Arm *arm = pc_action_4dof_arm(arm_index);
        if (!s_pc_servo_speed_overridden[arm_index]) {
            s_pc_saved_servo_speed[arm_index] = arm->cfg.servo_speed;
            s_pc_servo_speed_overridden[arm_index] = true;
        }
        arm->cfg.servo_speed = PC_ACT4_BACK_SERVO_SPEED;
    }
}

/**
 * @brief 设置参与动作的工作臂电磁阀状态，并同步反馈镜像。
 *
 * @param state PC_ACT4_VALVE_OPEN 或 PC_ACT4_VALVE_CLOSED
 */
static void pc_action_4dof_set_working_arm_valves(uint8_t state)
{
    if (s_pc_ctx.use_left) {
        relay_control(PC_ACT4_RELAY_LEFT_ARM, state);
        cmd4_update_valve_shadow(PC_ACT4_RELAY_LEFT_ARM, state);
    }
    if (s_pc_ctx.use_right) {
        relay_control(PC_ACT4_RELAY_RIGHT_ARM, state);
        cmd4_update_valve_shadow(PC_ACT4_RELAY_RIGHT_ARM, state);
    }
}

/**
 * @brief 打开参与动作的工作臂阀。
 */
static void pc_action_4dof_open_working_arm_valves(void)
{
    pc_action_4dof_set_working_arm_valves(PC_ACT4_VALVE_OPEN);
}

/**
 * @brief 关闭参与动作的工作臂阀。
 */
static void pc_action_4dof_close_working_arm_valves(void)
{
    pc_action_4dof_set_working_arm_valves(PC_ACT4_VALVE_CLOSED);
}

/**
 * @brief 设置参与动作的背部电磁阀状态，并同步反馈镜像。
 *
 * @param state PC_ACT4_VALVE_OPEN 或 PC_ACT4_VALVE_CLOSED
 */
static void pc_action_4dof_set_selected_back_valves(uint8_t state)
{
    if (s_pc_ctx.use_left) {
        relay_control(PC_ACT4_RELAY_LEFT_BACK, state);
        cmd4_update_valve_shadow(PC_ACT4_RELAY_LEFT_BACK, state);
    }
    if (s_pc_ctx.use_right) {
        relay_control(PC_ACT4_RELAY_RIGHT_BACK, state);
        cmd4_update_valve_shadow(PC_ACT4_RELAY_RIGHT_BACK, state);
    }
}

/**
 * @brief 放到背部时打开目标背部阀。
 */
static void pc_action_4dof_open_target_back_valves(void)
{
    pc_action_4dof_set_selected_back_valves(PC_ACT4_VALVE_OPEN);
}

/**
 * @brief 从背部取回时关闭来源背部阀。
 */
static void pc_action_4dof_close_source_back_valves(void)
{
    pc_action_4dof_set_selected_back_valves(PC_ACT4_VALVE_CLOSED);
}

/**
 * @brief 从背部取回动作完成后恢复来源背部阀打开。
 */
static void pc_action_4dof_open_source_back_valves(void)
{
    pc_action_4dof_set_selected_back_valves(PC_ACT4_VALVE_OPEN);
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
 * @brief 根据当前动作是否存在撤离段，进入撤离段或直接回 IDLE。
 */
static void pc_action_4dof_begin_post_or_idle(void)
{
    s_pc_ctx.path_index = 0U;
    if (pc_action_4dof_current_path_count(false) == 0U) {
        pc_action_4dof_set_state(PC_ACT4_STATE_RETURN_IDLE,
                                 PC_ACT4_MOVE_TIMEOUT_MS);
    } else {
        pc_action_4dof_set_state(PC_ACT4_STATE_MOVE_POST_PATH,
                                 PC_ACT4_MOVE_TIMEOUT_MS);
    }
}

/**
 * @brief 正常完成动作，释放执行权并标记结束事件待发送。
 *
 * 阀门处理遵循遥控任务模式：
 * - 参与动作的工作臂在完成后恢复电磁阀打开；
 * - 放置类动作的释放等待仍在运动状态机中完成，回到 idle 后再恢复默认打开。
 *
 * 在临界区内清除活跃标志、重置上下文，设置 completion_pending 以通知
 * 上层发送 0xCC 结束事件。
 */
static void pc_action_4dof_finish(void)
{
    pc_action_4dof_open_working_arm_valves();
    if (pc_action_4dof_is_back_get_command(s_pc_ctx.command)) {
        pc_action_4dof_open_source_back_valves();
    }
    pc_action_4dof_restore_servo_speed();

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
 * - release_committed=false 时，工作臂电磁阀仍保持打开，继续保载；
 * - release_committed=true 时，保留已经完成的释放和背部占用结果。
 *
 * 状态机进入 ABORT_RETURN_IDLE 后尝试一个超时周期归位。
 */
static void pc_action_4dof_begin_abort_return(void)
{
    if (!s_pc_ctx.error_reported) {
        s_pc_ctx.error_reported = true;
        pc_action_4dof_report_error_event();
    }
    pc_action_4dof_set_state(PC_ACT4_STATE_ABORT_RETURN_IDLE,
                             PC_ACT4_MOVE_TIMEOUT_MS);
}

/* ════════════════════════════════════════════════════════════════
 * 关节模式独立状态机 —— 完全参照 action_scheduler_4dof 的 action_4dof_handle_joint
 * ════════════════════════════════════════════════════════════════ */
#define PC_ACT4_JOINT_BLEND_TOL_RAD (PC_ACT4_JOINT_TOL_RAD * 3.0f)

static void pc_action_4dof_jnt_drive_pre(uint8_t pre_idx)
{
    for (uint8_t ai = 0U; ai < PC_ACT4_ARM_COUNT; ++ai) {
        if (!pc_action_4dof_arm_used(&s_pc_ctx, ai)) continue;
        const PcAction4DOF_JointPoint *wp = &s_pc_ctx.path.joint[ai].pre[pre_idx];
        Dof4_JointState js;
        memcpy(js.q, wp->q, sizeof(js.q));
        (void)Dof4_arm_set_joint_target(pc_action_4dof_arm(ai), &js);
    }
}
static void pc_action_4dof_jnt_drive_post(uint8_t post_idx)
{
    for (uint8_t ai = 0U; ai < PC_ACT4_ARM_COUNT; ++ai) {
        if (!pc_action_4dof_arm_used(&s_pc_ctx, ai)) continue;
        const PcAction4DOF_JointPoint *wp = &s_pc_ctx.path.joint[ai].post[post_idx];
        Dof4_JointState js;
        memcpy(js.q, wp->q, sizeof(js.q));
        (void)Dof4_arm_set_joint_target(pc_action_4dof_arm(ai), &js);
    }
}
static bool pc_action_4dof_jnt_pre_blend(uint8_t idx)
{
    for (uint8_t ai = 0U; ai < PC_ACT4_ARM_COUNT; ++ai) {
        if (!pc_action_4dof_arm_used(&s_pc_ctx, ai)) continue;
        const float *tgt = s_pc_ctx.path.joint[ai].pre[idx].q;
        const Dof4_Arm *arm = pc_action_4dof_arm(ai);
        for (uint8_t j = 0U; j < DOF4_JOINT_COUNT; ++j)
            if (fabsf(Dof4_normalize_angle(arm->joint_actual.q[j] - tgt[j])) > PC_ACT4_JOINT_BLEND_TOL_RAD) return false;
    }
    return true;
}
static bool pc_action_4dof_jnt_pre_reach(uint8_t idx)
{
    for (uint8_t ai = 0U; ai < PC_ACT4_ARM_COUNT; ++ai) {
        if (!pc_action_4dof_arm_used(&s_pc_ctx, ai)) continue;
        if (!pc_action_4dof_joint_reached(pc_action_4dof_arm(ai), &s_pc_ctx.path.joint[ai].pre[idx])) return false;
    }
    return true;
}
static bool pc_action_4dof_jnt_post_blend(uint8_t idx)
{
    for (uint8_t ai = 0U; ai < PC_ACT4_ARM_COUNT; ++ai) {
        if (!pc_action_4dof_arm_used(&s_pc_ctx, ai)) continue;
        const float *tgt = s_pc_ctx.path.joint[ai].post[idx].q;
        const Dof4_Arm *arm = pc_action_4dof_arm(ai);
        for (uint8_t j = 0U; j < DOF4_JOINT_COUNT; ++j)
            if (fabsf(Dof4_normalize_angle(arm->joint_actual.q[j] - tgt[j])) > PC_ACT4_JOINT_BLEND_TOL_RAD) return false;
    }
    return true;
}
static bool pc_action_4dof_jnt_post_reach(uint8_t idx)
{
    for (uint8_t ai = 0U; ai < PC_ACT4_ARM_COUNT; ++ai) {
        if (!pc_action_4dof_arm_used(&s_pc_ctx, ai)) continue;
        if (!pc_action_4dof_joint_reached(pc_action_4dof_arm(ai), &s_pc_ctx.path.joint[ai].post[idx])) return false;
    }
    return true;
}
static void pc_action_4dof_jnt_set_tmo(uint32_t ms) { s_pc_ctx.enter_tick = HAL_GetTick(); s_pc_ctx.timeout_ms = ms; }

static void pc_action_4dof_handle_joint(void)
{
    const uint8_t last_pre = pc_action_4dof_current_path_count(true) - 1U;

    switch (s_pc_ctx.jnt_substate) {
    case PC_ACT4_JNT_APPROACH:
    case PC_ACT4_JNT_PLACE_APPROACH:
        pc_action_4dof_jnt_drive_pre(0U);
        if (pc_action_4dof_jnt_pre_blend(0U) || pc_action_4dof_timed_out()) {
            s_pc_ctx.path_index = 0U;
            s_pc_ctx.jnt_substate = PC_ACT4_JNT_INTERMEDIATE;
            pc_action_4dof_jnt_set_tmo(PC_ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    case PC_ACT4_JNT_INTERMEDIATE: {
        uint8_t wp_i = s_pc_ctx.path_index;
        uint8_t pre_i = wp_i + 1U;
        pc_action_4dof_jnt_drive_pre(pre_i);
        if (pc_action_4dof_jnt_pre_blend(pre_i) || pc_action_4dof_timed_out()) {
            s_pc_ctx.path_index = wp_i + 1U;
            if (s_pc_ctx.path_index < last_pre) {
                s_pc_ctx.jnt_substate = PC_ACT4_JNT_INTERMEDIATE;
                pc_action_4dof_jnt_set_tmo(PC_ACT4_MOVE_TIMEOUT_MS);
            } else {
                bool is_place = pc_action_4dof_is_back_put_command(s_pc_ctx.command);
                s_pc_ctx.jnt_substate = is_place ? PC_ACT4_JNT_PLACE : PC_ACT4_JNT_GRAB;
                pc_action_4dof_jnt_set_tmo(PC_ACT4_MOVE_TIMEOUT_MS);
            }
        }
        break;
    }

    case PC_ACT4_JNT_GRAB:
        pc_action_4dof_jnt_drive_pre(last_pre);
        if (pc_action_4dof_jnt_pre_reach(last_pre) || pc_action_4dof_timed_out()) {
            s_pc_ctx.joint_phase_started = false;
            s_pc_ctx.jnt_substate = PC_ACT4_JNT_SUCTION_GRAB;
            pc_action_4dof_jnt_set_tmo(PC_ACT4_DELAY_BACK_GET_ARM_HOLD_MS);
        }
        break;

    case PC_ACT4_JNT_SUCTION_GRAB:
        pc_action_4dof_jnt_drive_pre(last_pre);
        if (pc_action_4dof_timed_out()) {
            if (!pc_action_4dof_is_back_get_command(s_pc_ctx.command)) {
                s_pc_ctx.jnt_substate = PC_ACT4_JNT_RETREAT;
                pc_action_4dof_jnt_set_tmo(PC_ACT4_MOVE_TIMEOUT_MS);
            } else if (!s_pc_ctx.joint_phase_started) {
                /* 背部取回：手臂已吸附稳定，关闭来源背部阀并清空背部占用。 */
                s_pc_ctx.joint_phase_started = true;
                pc_action_4dof_close_source_back_valves();
                pc_action_4dof_apply_back_state(false);
                s_pc_ctx.release_committed = true;
                pc_action_4dof_jnt_set_tmo(PC_ACT4_DELAY_BACK_SOURCE_RELEASE_MS);
            } else {
                s_pc_ctx.jnt_substate = PC_ACT4_JNT_RETREAT;
                pc_action_4dof_jnt_set_tmo(PC_ACT4_MOVE_TIMEOUT_MS);
            }
        }
        break;

    case PC_ACT4_JNT_PLACE:
        pc_action_4dof_jnt_drive_pre(last_pre);
        if (!s_pc_ctx.joint_phase_started) {
            if (pc_action_4dof_jnt_pre_reach(last_pre) ||
                pc_action_4dof_timed_out()) {
                /* 背部放置：到达或目标点运动超时后，仍按时序打开目标背部阀。 */
                s_pc_ctx.joint_phase_started = true;
                pc_action_4dof_open_target_back_valves();
                pc_action_4dof_jnt_set_tmo(PC_ACT4_DELAY_BACK_PRE_RELEASE_MS);
            }
        } else if (pc_action_4dof_timed_out()) {
            s_pc_ctx.joint_phase_started = false;
            s_pc_ctx.jnt_substate = PC_ACT4_JNT_SUCTION_PLACE;
            pc_action_4dof_jnt_set_tmo(0U);
        }
        break;

    case PC_ACT4_JNT_SUCTION_PLACE:
        pc_action_4dof_jnt_drive_pre(last_pre);
        if (!s_pc_ctx.joint_phase_started) {
            /* 背部放置：背部已吸附稳定，关闭工作臂阀完成交接。 */
            s_pc_ctx.joint_phase_started = true;
            pc_action_4dof_close_working_arm_valves();
            pc_action_4dof_apply_back_state(true);
            s_pc_ctx.release_committed = true;
            pc_action_4dof_jnt_set_tmo(PC_ACT4_DELAY_BACK_POST_RELEASE_MS);
        } else if (pc_action_4dof_timed_out()) {
            s_pc_ctx.jnt_substate = PC_ACT4_JNT_RETREAT;
            pc_action_4dof_jnt_set_tmo(PC_ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    case PC_ACT4_JNT_RETREAT:
        pc_action_4dof_jnt_drive_post(0U);
        if (pc_action_4dof_jnt_post_blend(0U) || pc_action_4dof_timed_out()) {
            s_pc_ctx.jnt_substate = PC_ACT4_JNT_COMPLETE;
            pc_action_4dof_jnt_set_tmo(PC_ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    case PC_ACT4_JNT_COMPLETE:
        if (s_pc_ctx.timeout_ms == PC_ACT4_MOVE_TIMEOUT_MS) {
            pc_action_4dof_jnt_drive_post(1U);
            if (pc_action_4dof_jnt_post_reach(1U) || pc_action_4dof_timed_out()) {
                pc_action_4dof_jnt_set_tmo(PC_ACT4_DELAY_IDLE_HOLD_MS);
            }
        } else {
            if (pc_action_4dof_idle_reached() || pc_action_4dof_timed_out()) {
                pc_action_4dof_finish();
            }
        }
        break;

    default:
        break;
    }
}

void pc_action_4dof_record_reject(Dof4_ArmId arm_id,
                                  PcAction4DOF_RejectReason reason,
                                  const Dof4_Pose *target_world)
{
    pc_action_4dof_report_error_event();

    Dof4_Arm *arm = (arm_id == DOF4_ARM_RIGHT)
        ? &g_dof4_arm_right
        : &g_dof4_arm_left;

    Dof4_ClipDiagnostic *diagnostic = &arm->clip_diagnostic;
    memset(diagnostic, 0, sizeof(*diagnostic));
    if (target_world != NULL) {
        diagnostic->requested_pose = *target_world;
        diagnostic->limited_pose = *target_world;
    }
    diagnostic->control_mode = DOF4_CONTROL_MODE_POSE;
    diagnostic->reason = DOF4_CLIP_REASON_PC_ACTION_REJECT;
    diagnostic->joint_mask = (uint8_t)reason;
    arm->clip_event_counter++;
    diagnostic->event_id = arm->clip_event_counter;
    diagnostic->pending = true;
}

/**
 * @brief 构造简化动态取放路径（中间点悬停后垂直下降）。
 *
 * 路径结构（单臂）：
 *   pre[0] = 悬停点（target 正上方，俯仰角 = 模板 entry_offset.pitch）
 *   pre[1] = 目标点（PC 下发的 x, y, z，俯仰角由模板指定）
 *   post[0]= 悬停点（撤离时回到正上方）
 *
 * 悬停点 xyz 保持在 target 正上方，pitch 使用实机已调的 approach 姿态，
 * 避免正上方中间点因强制目标俯仰角导致 IK 不可达。
 *
 * @param candidate     待填充的上下文（输出参数）
 * @param arm_index     手臂索引 (0=左, 1=右)
 * @param target_world  PC 下发的目标世界坐标
 * @param template_data 动态动作模板
 * @param reject_reason 拒绝原因（输出参数）
 * @return true 表示路径构造成功且所有点可达
 */
static bool pc_action_4dof_build_dynamic_arm_path(
    PcAction4DOF_Context *candidate,
    uint8_t arm_index,
    const Dof4_Pose *target_world,
    const PcAction4DOF_DynamicTemplate *template_data,
    PcAction4DOF_RejectReason *reject_reason)
{
    if (reject_reason != NULL) {
        *reject_reason = PC_ACTION_4DOF_REJECT_NONE;
    }

    if (candidate == NULL || arm_index >= PC_ACT4_ARM_COUNT ||
        template_data == NULL) {
        if (reject_reason != NULL) {
            *reject_reason = PC_ACTION_4DOF_REJECT_BAD_TARGET;
        }
        return false;
    }

    if (!pc_action_4dof_pose_finite(target_world)) {
        if (reject_reason != NULL) {
            *reject_reason = PC_ACTION_4DOF_REJECT_BAD_TARGET;
        }
        return false;
    }

    PcAction4DOF_PosePath *path = &candidate->path.pose[arm_index];
    Dof4_Arm *arm = pc_action_4dof_arm(arm_index);

    /* ── 构造目标点位姿 ──────────────────────────────────────────── */
    const Dof4_Pose target = {
        .x = target_world->x,
        .y = target_world->y,
        .z = target_world->z,
        .pitch = template_data->target_pitch,
    };
    if (!pc_action_4dof_pose_reachable(arm, &target)) {
        if (reject_reason != NULL) {
            *reject_reason = PC_ACTION_4DOF_REJECT_TARGET_UNREACHABLE;
        }
        return false;
    }

    /* ── 构造悬停点：目标正上方，末端优先垂直向下 ─────────── */
    Dof4_Pose hover = target;
    hover.z += template_data->vertical_clearance_m;
    (void)Dof4_clamp_to_workspace(arm, &hover);

    /* 逐级降级俯仰角：优先 target_pitch（末端垂直向下），
     * 不可达则降级到模板 approach 姿态，再不可达则用钳位 pitch */
    hover.pitch = template_data->target_pitch;
    if (!pc_action_4dof_pose_reachable(arm, &hover)) {
        hover.pitch = template_data->entry_offset.pitch;
        if (!pc_action_4dof_pose_reachable(arm, &hover)) {
            Dof4_Pose clamped2 = target;
            clamped2.z += template_data->vertical_clearance_m;
            (void)Dof4_clamp_to_workspace(arm, &clamped2);
            if (!pc_action_4dof_pose_reachable(arm, &clamped2)) {
                if (reject_reason != NULL) {
                    *reject_reason = PC_ACTION_4DOF_REJECT_TARGET_ABOVE_UNREACHABLE;
                }
                return false;
            }
            hover = clamped2;
        }
    }

    /* ── 简化路径：悬停点 → 目标点（无中间插值） ────────────────── */
    path->pre[0] = hover;
    path->pre[1] = target;
    path->pre_count = 2U;
    path->post[0] = hover;   /* 撤离时回到悬停点 */
    path->post_count = 1U;

    return true;
}

/**
 * @brief 原子取得 PC 动作执行权。
 *
 * 路径在临界区外构造和校验，最终只在短临界区内检查两个状态机并复制上下文。
 * 因此 PC 动作与预设动作即使同时触发，也只会有一方成功。
 *
 * 执行条件：
 * - PC 状态机不在活跃状态
 * - 无待发送的结束事件
 * - 预设动作状态机不在活跃状态
 *
 * @param candidate 已构造好的候选上下文
 * @return true 表示成功取得执行权，动作已启动
 */
static bool pc_action_4dof_claim(const PcAction4DOF_Context *candidate,
                                 Dof4_ArmId reject_arm_id,
                                 const Dof4_Pose *target_world)
{
    bool accepted = false;
    PcAction4DOF_RejectReason reject_reason = PC_ACTION_4DOF_REJECT_NONE;

    if (candidate == NULL) {
        pc_action_4dof_record_reject(reject_arm_id,
                                     PC_ACTION_4DOF_REJECT_BAD_TARGET,
                                     target_world);
        return false;
    }

    taskENTER_CRITICAL();
    if (s_pc_ctx.active) {
        reject_reason = PC_ACTION_4DOF_REJECT_BUSY;
    } else if (s_pc_completion_pending) {
        reject_reason = PC_ACTION_4DOF_REJECT_COMPLETION_PENDING;
    } else if (action_4dof_is_active()) {
        reject_reason = PC_ACTION_4DOF_REJECT_ACTION_ACTIVE;
    } else {
        s_pc_ctx = *candidate;
        s_pc_ctx.active = true;
        accepted = true;
    }
    taskEXIT_CRITICAL();

    if (accepted) {
        /* 所有取放动作启动时先打开工作臂电磁阀，确保接近过程中保持默认阀态。 */
        pc_action_4dof_apply_back_servo_speed();
        pc_action_4dof_open_working_arm_valves();
    } else {
        pc_action_4dof_record_reject(reject_arm_id, reject_reason, target_world);
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
    candidate->use_left = use_left;
    candidate->use_right = use_right;
    candidate->enter_tick = HAL_GetTick();
    candidate->timeout_ms = PC_ACT4_MOVE_TIMEOUT_MS;
    if (mode == PC_ACT4_MODE_JOINT) {
        candidate->state = PC_ACT4_STATE_IDLE;
        candidate->jnt_substate = pc_action_4dof_is_back_put_command(command)
                                  ? PC_ACT4_JNT_PLACE_APPROACH
                                  : PC_ACT4_JNT_APPROACH;
    } else {
        candidate->state = PC_ACT4_STATE_MOVE_PRE_PATH;
    }
}

static bool pc_action_4dof_command_uses_pose(PcAction4DOF_Command command)
{
    return pc_action_4dof_is_dynamic_pick_command(command) ||
           pc_action_4dof_is_dynamic_place_command(command);
}

static bool pc_action_4dof_should_hold_pre_waypoint(void)
{
    return s_pc_ctx.mode == PC_ACT4_MODE_POSE &&
           pc_action_4dof_command_uses_pose(s_pc_ctx.command) &&
           s_pc_ctx.path_index == PC_ACT4_DYNAMIC_HOLD_PRE_INDEX;
}

static void pc_action_4dof_clear_pending(void)
{
    taskENTER_CRITICAL();
    memset(&s_pc_pending, 0, sizeof(s_pc_pending));
    taskEXIT_CRITICAL();
}

static bool pc_action_4dof_submit_pending(PcAction4DOF_Command command,
                                          bool use_left,
                                          bool use_right,
                                          const Dof4_Pose *left_target_world,
                                          const Dof4_Pose *right_target_world,
                                          Dof4_ArmId reject_arm_id,
                                          const Dof4_Pose *reject_target_world)
{
    PcAction4DOF_RejectReason reject_reason = PC_ACTION_4DOF_REJECT_NONE;
    bool accepted = false;

    if (pc_action_4dof_command_uses_pose(command)) {
        if ((use_left && !pc_action_4dof_pose_finite(left_target_world)) ||
            (use_right && !pc_action_4dof_pose_finite(right_target_world))) {
            pc_action_4dof_record_reject(reject_arm_id,
                                         PC_ACTION_4DOF_REJECT_BAD_TARGET,
                                         reject_target_world);
            return false;
        }
    }

    taskENTER_CRITICAL();
    if (s_pc_pending.valid) {
        reject_reason = PC_ACTION_4DOF_REJECT_PENDING_FULL;
    } else if (s_pc_ctx.active) {
        reject_reason = PC_ACTION_4DOF_REJECT_BUSY;
    } else if (s_pc_completion_pending) {
        reject_reason = PC_ACTION_4DOF_REJECT_COMPLETION_PENDING;
    } else if (action_4dof_is_active()) {
        reject_reason = PC_ACTION_4DOF_REJECT_ACTION_ACTIVE;
    } else {
        memset(&s_pc_pending, 0, sizeof(s_pc_pending));
        s_pc_pending.valid = true;
        s_pc_pending.command = command;
        s_pc_pending.use_left = use_left;
        s_pc_pending.use_right = use_right;
        if (use_left && left_target_world != NULL) {
            s_pc_pending.target[0] = *left_target_world;
        }
        if (use_right && right_target_world != NULL) {
            s_pc_pending.target[1] = *right_target_world;
        }
        accepted = true;
    }
    taskEXIT_CRITICAL();

    if (!accepted) {
        pc_action_4dof_record_reject(reject_arm_id,
                                     reject_reason,
                                     reject_target_world);
    }
    return accepted;
}

/* @brief 从待处理请求构建候选动作上下文。
 *
 * 根据待处理请求的命令类型和手臂选择，构造对应的路径数据并填充到候选上下文中。
 * 如果路径构造失败，则返回 false 并设置 reject_arm_id 和 reject_target_world 以记录拒绝原因。
 *
 * @param request 待处理请求
 * @param candidate 待填充的候选上下文（输出参数）
 * @param reject_arm_id 拒绝的手臂 ID（输出参数）
 * @param reject_target_world 拒绝的目标位姿（输出参数）
 * @return true 表示成功构建候选上下文，false 表示构建失败
 */
static bool pc_action_4dof_build_candidate_from_pending(
    const PcAction4DOF_PendingRequest *request,
    PcAction4DOF_Context *candidate,
    Dof4_ArmId *reject_arm_id,
    const Dof4_Pose **reject_target_world)
{
    if (request == NULL || candidate == NULL ||
        reject_arm_id == NULL || reject_target_world == NULL) {
        return false;
    }

    *reject_arm_id = request->use_right ? DOF4_ARM_RIGHT : DOF4_ARM_LEFT;
    *reject_target_world = NULL;

    switch (request->command) {
    case PC_ACT4_COMMAND_PICK:
    case PC_ACT4_COMMAND_PLACE: {
        const bool use_left = request->use_left;
        const uint8_t arm_index = use_left ? 0U : 1U;
        const Dof4_ArmId arm_id = use_left ? DOF4_ARM_LEFT : DOF4_ARM_RIGHT;
        const PcAction4DOF_DynamicTemplate *template_data =
            (request->command == PC_ACT4_COMMAND_PICK)
                ? &s_pick_templates[arm_index]
                : &s_place_templates[arm_index];
        PcAction4DOF_RejectReason reject_reason = PC_ACTION_4DOF_REJECT_NONE;

        *reject_arm_id = arm_id;
        *reject_target_world = &request->target[arm_index];
        pc_action_4dof_prepare_candidate(
            candidate,
            request->command,
            PC_ACT4_MODE_POSE,
            arm_index == 0U,
            arm_index == 1U);
        if (!pc_action_4dof_build_dynamic_arm_path(
                candidate,
                arm_index,
                &request->target[arm_index],
                template_data,
                &reject_reason)) {
            pc_action_4dof_record_reject(
                arm_id, reject_reason, &request->target[arm_index]);
            return false;
        }
        return true;
    }

    case PC_ACT4_COMMAND_DUAL_PICK:
    case PC_ACT4_COMMAND_DUAL_PLACE: {
        const PcAction4DOF_DynamicTemplate *left_template =
            (request->command == PC_ACT4_COMMAND_DUAL_PICK)
                ? &s_dual_pick_templates[0]
                : &s_place_templates[0];
        const PcAction4DOF_DynamicTemplate *right_template =
            (request->command == PC_ACT4_COMMAND_DUAL_PICK)
                ? &s_dual_pick_templates[1]
                : &s_place_templates[1];
        PcAction4DOF_RejectReason reject_reason = PC_ACTION_4DOF_REJECT_NONE;

        *reject_arm_id = DOF4_ARM_LEFT;
        *reject_target_world = &request->target[0];
        pc_action_4dof_prepare_candidate(
            candidate, request->command, PC_ACT4_MODE_POSE, true, true);
        if (!pc_action_4dof_build_dynamic_arm_path(
                candidate, 0U, &request->target[0],
                left_template, &reject_reason)) {
            pc_action_4dof_record_reject(
                DOF4_ARM_LEFT, reject_reason, &request->target[0]);
            return false;
        }
        if (!pc_action_4dof_build_dynamic_arm_path(
                candidate, 1U, &request->target[1],
                right_template, &reject_reason)) {
            *reject_arm_id = DOF4_ARM_RIGHT;
            *reject_target_world = &request->target[1];
            pc_action_4dof_record_reject(
                DOF4_ARM_RIGHT, reject_reason, &request->target[1]);
            return false;
        }
        return true;
    }

    case PC_ACT4_COMMAND_PUT_BACK:
    case PC_ACT4_COMMAND_GET_BACK: {
        const bool use_left = request->use_left;
        const uint8_t arm_index = use_left ? 0U : 1U;
        const Dof4_ArmId arm_id = use_left ? DOF4_ARM_LEFT : DOF4_ARM_RIGHT;
        *reject_arm_id = arm_id;
        pc_action_4dof_prepare_candidate(
            candidate,
            request->command,
            PC_ACT4_MODE_JOINT,
            arm_index == 0U,
            arm_index == 1U);
        candidate->path.joint[arm_index] =
            (request->command == PC_ACT4_COMMAND_PUT_BACK)
                ? s_put_back_paths[arm_index]
                : s_get_back_paths[arm_index];
        if (!pc_action_4dof_normalize_joint_path(
                arm_index, &candidate->path.joint[arm_index])) {
            pc_action_4dof_record_reject(
                arm_id, PC_ACTION_4DOF_REJECT_JOINT_PATH_INVALID, NULL);
            return false;
        }
        return true;
    }

    case PC_ACT4_COMMAND_DUAL_PUT_BACK:
    case PC_ACT4_COMMAND_DUAL_GET_BACK:
        *reject_arm_id = DOF4_ARM_LEFT;
        pc_action_4dof_prepare_candidate(
            candidate, request->command, PC_ACT4_MODE_JOINT, true, true);
        if (request->command == PC_ACT4_COMMAND_DUAL_PUT_BACK) {
            candidate->path.joint[0] = s_dual_put_back_paths[0];
            candidate->path.joint[1] = s_dual_put_back_paths[1];
        } else {
            candidate->path.joint[0] = s_get_back_paths[0];
            candidate->path.joint[1] = s_get_back_paths[1];
        }
        if (!pc_action_4dof_normalize_joint_path(0U, &candidate->path.joint[0]) ||
            !pc_action_4dof_normalize_joint_path(1U, &candidate->path.joint[1])) {
            pc_action_4dof_record_reject(
                DOF4_ARM_LEFT, PC_ACTION_4DOF_REJECT_JOINT_PATH_INVALID, NULL);
            return false;
        }
        return true;

    case PC_ACT4_COMMAND_NONE:
    default:
        pc_action_4dof_record_reject(
            DOF4_ARM_LEFT, PC_ACTION_4DOF_REJECT_BAD_TARGET, NULL);
        return false;
    }
}

static void pc_action_4dof_try_start_pending(void)
{
    PcAction4DOF_PendingRequest request;
    bool has_request = false;

    taskENTER_CRITICAL();
    if (s_pc_pending.valid && !s_pc_ctx.active) {
        request = s_pc_pending;
        has_request = true;
    }
    taskEXIT_CRITICAL();

    if (!has_request) {
        return;
    }

    PcAction4DOF_Context candidate;
    Dof4_ArmId reject_arm_id = DOF4_ARM_LEFT;
    const Dof4_Pose *reject_target_world = NULL;

    if (pc_action_4dof_build_candidate_from_pending(
            &request, &candidate, &reject_arm_id, &reject_target_world)) {
        (void)pc_action_4dof_claim(
            &candidate, reject_arm_id, reject_target_world);
    }

    pc_action_4dof_clear_pending();
}

/* ======================== 公有 API 实现 ======================== */

void pc_action_4dof_init(void)
{
    pc_action_4dof_restore_servo_speed();
    taskENTER_CRITICAL();
    memset(&s_pc_ctx, 0, sizeof(s_pc_ctx));
    memset(&s_pc_pending, 0, sizeof(s_pc_pending));
    memset(s_pc_saved_servo_speed, 0, sizeof(s_pc_saved_servo_speed));
    memset(s_pc_servo_speed_overridden, 0, sizeof(s_pc_servo_speed_overridden));
    s_pc_ctx.command = PC_ACT4_COMMAND_NONE;
    s_pc_ctx.state = PC_ACT4_STATE_IDLE;
    s_pc_completion_pending = false;
    taskEXIT_CRITICAL();
}

void pc_action_4dof_abort(void)
{
    pc_action_4dof_restore_servo_speed();
    taskENTER_CRITICAL();
    memset(&s_pc_ctx, 0, sizeof(s_pc_ctx));
    memset(&s_pc_pending, 0, sizeof(s_pc_pending));
    memset(s_pc_saved_servo_speed, 0, sizeof(s_pc_saved_servo_speed));
    memset(s_pc_servo_speed_overridden, 0, sizeof(s_pc_servo_speed_overridden));
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
        pc_action_4dof_record_reject(DOF4_ARM_LEFT,
                                     PC_ACTION_4DOF_REJECT_INVALID_ARM,
                                     target_world);
        return false;
    }

    const uint8_t arm_index = (arm_id == DOF4_ARM_LEFT) ? 0U : 1U;
    return pc_action_4dof_submit_pending(
        PC_ACT4_COMMAND_PICK,
        arm_index == 0U,
        arm_index == 1U,
        arm_index == 0U ? target_world : NULL,
        arm_index == 1U ? target_world : NULL,
        arm_id,
        target_world);
}

bool pc_action_4dof_start_place(Dof4_ArmId arm_id,
                                const Dof4_Pose *target_world)
{
    /* 校验手臂 ID 合法性 */
    if (arm_id != DOF4_ARM_LEFT && arm_id != DOF4_ARM_RIGHT) {
        pc_action_4dof_record_reject(DOF4_ARM_LEFT,
                                     PC_ACTION_4DOF_REJECT_INVALID_ARM,
                                     target_world);
        return false;
    }

    const uint8_t arm_index = (arm_id == DOF4_ARM_LEFT) ? 0U : 1U;
    return pc_action_4dof_submit_pending(
        PC_ACT4_COMMAND_PLACE,
        arm_index == 0U,
        arm_index == 1U,
        arm_index == 0U ? target_world : NULL,
        arm_index == 1U ? target_world : NULL,
        arm_id,
        target_world);
}

bool pc_action_4dof_start_put_back(Dof4_ArmId arm_id)
{
    /* 校验手臂 ID 合法性 */
    if (arm_id != DOF4_ARM_LEFT && arm_id != DOF4_ARM_RIGHT) {
        pc_action_4dof_record_reject(DOF4_ARM_LEFT,
                                     PC_ACTION_4DOF_REJECT_INVALID_ARM,
                                     NULL);
        return false;
    }

    const uint8_t arm_index = (arm_id == DOF4_ARM_LEFT) ? 0U : 1U;
    return pc_action_4dof_submit_pending(
        PC_ACT4_COMMAND_PUT_BACK,
        arm_index == 0U,
        arm_index == 1U,
        NULL,
        NULL,
        arm_id,
        NULL);
}

bool pc_action_4dof_start_get_back(Dof4_ArmId arm_id)
{
    /* 校验手臂 ID 合法性 */
    if (arm_id != DOF4_ARM_LEFT && arm_id != DOF4_ARM_RIGHT) {
        pc_action_4dof_record_reject(DOF4_ARM_LEFT,
                                     PC_ACTION_4DOF_REJECT_INVALID_ARM,
                                     NULL);
        return false;
    }

    const uint8_t arm_index = (arm_id == DOF4_ARM_LEFT) ? 0U : 1U;
    return pc_action_4dof_submit_pending(
        PC_ACT4_COMMAND_GET_BACK,
        arm_index == 0U,
        arm_index == 1U,
        NULL,
        NULL,
        arm_id,
        NULL);
}

bool pc_action_4dof_start_dual_pick(const Dof4_Pose *left_target_world,
                                    const Dof4_Pose *right_target_world)
{
    return pc_action_4dof_submit_pending(
        PC_ACT4_COMMAND_DUAL_PICK,
        true,
        true,
        left_target_world,
        right_target_world,
        DOF4_ARM_LEFT,
        left_target_world);
}

bool pc_action_4dof_start_dual_place(const Dof4_Pose *left_target_world,
                                     const Dof4_Pose *right_target_world)
{
    return pc_action_4dof_submit_pending(
        PC_ACT4_COMMAND_DUAL_PLACE,
        true,
        true,
        left_target_world,
        right_target_world,
        DOF4_ARM_LEFT,
        left_target_world);
}

bool pc_action_4dof_start_dual_put_back(void)
{
    return pc_action_4dof_submit_pending(
        PC_ACT4_COMMAND_DUAL_PUT_BACK,
        true,
        true,
        NULL,
        NULL,
        DOF4_ARM_LEFT,
        NULL);
}

bool pc_action_4dof_start_dual_get_back(void)
{
    return pc_action_4dof_submit_pending(
        PC_ACT4_COMMAND_DUAL_GET_BACK,
        true,
        true,
        NULL,
        NULL,
        DOF4_ARM_LEFT,
        NULL);
}

bool pc_action_4dof_is_active(void)
{
    bool active;
    taskENTER_CRITICAL();
    active = s_pc_ctx.active || s_pc_pending.valid;
    taskEXIT_CRITICAL();
    return active;
}

void pc_action_4dof_get_debug_snapshot(PcAction4DOF_DebugSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    snapshot->active = s_pc_ctx.active;
    snapshot->pending = s_pc_pending.valid;
    snapshot->use_left = s_pc_ctx.use_left;
    snapshot->use_right = s_pc_ctx.use_right;
    snapshot->release_committed = s_pc_ctx.release_committed;
    snapshot->operation_phase_started = s_pc_ctx.operation_phase_started;
    snapshot->joint_phase_started = s_pc_ctx.joint_phase_started;
    snapshot->command = (uint8_t)s_pc_ctx.command;
    snapshot->mode = (uint8_t)s_pc_ctx.mode;
    snapshot->state = (uint8_t)s_pc_ctx.state;
    snapshot->joint_substate = (uint8_t)s_pc_ctx.jnt_substate;
    snapshot->path_index = s_pc_ctx.path_index;
    snapshot->timeout_ms = s_pc_ctx.timeout_ms;
    snapshot->elapsed_ms = HAL_GetTick() - s_pc_ctx.enter_tick;
    taskEXIT_CRITICAL();
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
    pc_action_4dof_try_start_pending();

    if (!s_pc_ctx.active) {
        return;
    }

    /* ── 关节模式：独立状态机，完全参照 action_scheduler_4dof ── */
    if (s_pc_ctx.mode == PC_ACT4_MODE_JOINT) {
        pc_action_4dof_handle_joint();
        return;
    }

    /* ── 位姿模式：纯 POSE 状态机（PICK/PLACE） ── */
    switch (s_pc_ctx.state) {

    case PC_ACT4_STATE_MOVE_PRE_PATH:
        if (pc_action_4dof_drive_current_path_point(true)) {
            if (pc_action_4dof_should_hold_pre_waypoint()) {
                ++s_pc_ctx.path_index;
                pc_action_4dof_set_state(PC_ACT4_STATE_PRE_WAYPOINT_HOLD,
                                         PC_ACT4_DELAY_DYNAMIC_HOVER_HOLD_MS);
                break;
            }
            ++s_pc_ctx.path_index;
            if (s_pc_ctx.path_index >= pc_action_4dof_current_path_count(true)) {
                s_pc_ctx.path_index = 0U;
                pc_action_4dof_set_state(PC_ACT4_STATE_TARGET_SETTLE_HOLD,
                                         PC_ACT4_DELAY_DYNAMIC_TARGET_SETTLE_MS);
            } else {
                pc_action_4dof_set_state(PC_ACT4_STATE_MOVE_PRE_PATH, PC_ACT4_MOVE_TIMEOUT_MS);
            }
        } else if (pc_action_4dof_timed_out()) {
            if ((s_pc_ctx.path_index + 1U) >= pc_action_4dof_current_path_count(true)) {
                s_pc_ctx.path_index = 0U;
                pc_action_4dof_set_state(PC_ACT4_STATE_TARGET_SETTLE_HOLD,
                                         PC_ACT4_DELAY_DYNAMIC_TARGET_SETTLE_MS);
            } else {
                pc_action_4dof_begin_abort_return();
            }
        }
        break;

    case PC_ACT4_STATE_PRE_WAYPOINT_HOLD:
        if (pc_action_4dof_timed_out()) {
            pc_action_4dof_set_state(PC_ACT4_STATE_MOVE_PRE_PATH, PC_ACT4_MOVE_TIMEOUT_MS);
        }
        break;

    case PC_ACT4_STATE_TARGET_SETTLE_HOLD:
        if (pc_action_4dof_timed_out()) {
            s_pc_ctx.operation_phase_started = false;
            pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_HOLD, 0U);
        }
        break;

    case PC_ACT4_STATE_OPERATION_HOLD:
        if (!s_pc_ctx.operation_phase_started) {
            s_pc_ctx.operation_phase_started = true;
            if (pc_action_4dof_is_dynamic_pick_command(s_pc_ctx.command)) {
                pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_HOLD,
                                         PC_ACT4_DELAY_DYNAMIC_PICK_HOLD_MS);
            } else if (pc_action_4dof_is_dynamic_place_command(s_pc_ctx.command)) {
                /* 外部动态放置：到目标点后关闭工作臂阀释放，背部阀不参与。 */
                pc_action_4dof_close_working_arm_valves();
                s_pc_ctx.release_committed = true;
                pc_action_4dof_set_state(PC_ACT4_STATE_OPERATION_HOLD,
                                         PC_ACT4_DELAY_DYNAMIC_PLACE_RELEASE_MS);
            } else {
                pc_action_4dof_begin_abort_return();
            }
        } else if (pc_action_4dof_timed_out()) {
            pc_action_4dof_begin_post_or_idle();
        }
        break;

    case PC_ACT4_STATE_MOVE_POST_PATH:
        if (pc_action_4dof_drive_current_path_point(false)) {
            ++s_pc_ctx.path_index;
            if (s_pc_ctx.path_index >= pc_action_4dof_current_path_count(false)) {
                s_pc_ctx.path_index = 0U;
                pc_action_4dof_set_state(PC_ACT4_STATE_RETURN_IDLE, PC_ACT4_MOVE_TIMEOUT_MS);
            } else {
                pc_action_4dof_set_state(PC_ACT4_STATE_MOVE_POST_PATH, PC_ACT4_MOVE_TIMEOUT_MS);
            }
        } else if (pc_action_4dof_timed_out()) {
            pc_action_4dof_begin_abort_return();
        }
        break;

    case PC_ACT4_STATE_RETURN_IDLE:
        if (pc_action_4dof_idle_reached()) {
            pc_action_4dof_set_state(PC_ACT4_STATE_IDLE_HOLD,
                                     PC_ACT4_DELAY_IDLE_HOLD_MS);
        } else if (pc_action_4dof_timed_out()) {
            pc_action_4dof_begin_abort_return();
        }
        break;

    case PC_ACT4_STATE_ABORT_RETURN_IDLE:
        if (pc_action_4dof_idle_reached() || pc_action_4dof_timed_out()) {
            pc_action_4dof_set_state(PC_ACT4_STATE_IDLE_HOLD,
                                     PC_ACT4_DELAY_IDLE_HOLD_MS);
        }
        break;

    case PC_ACT4_STATE_IDLE_HOLD:
        if (pc_action_4dof_timed_out()) {
            pc_action_4dof_finish();
        }
        break;

    case PC_ACT4_STATE_IDLE:
    default:
        pc_action_4dof_begin_abort_return();
        break;
    }
}
