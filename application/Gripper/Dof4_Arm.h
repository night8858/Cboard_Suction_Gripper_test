/**
 * @file Dof4_Arm.h
 * @brief 双四自由度机械臂几何运动学、配置和控制循环接口。
 */

#ifndef DOF4_ARM_H
#define DOF4_ARM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

/** @brief 弧度转角度比例。URDF <limit lower upper> 为弧度，下面的 DEG 限位由它换算得到。 */
#define DOF4_RAD_TO_DEG (180.0f / M_PI_F)

/**
 * @brief 舵机映射公共常量。
 *
 * ZERO_POS 是舵机 position 零位，默认 2048，不是角度。
 * ZERO_BIAS_DEG 是实际/URDF 角度偏置，用于安装后微调。
 * URDF_MIN/MAX_DEG 只作用在实际/URDF 角度上，正反向换算都会先/后夹紧。
 */
#define DOF4_SERVO_MIN_POS 0
#define DOF4_SERVO_MAX_POS 4095
#define DOF4_SERVO_CENTER_POS 2048
#define DOF4_SERVO_POS_PER_DEG (4096.0f / 360.0f)
#define DOF4_SERVO_POS_PER_RAD (4096.0f / (2.0f * M_PI_F))

/* 兼容旧命名：代码内部逐步统一到 DOF4_SERVO_*_POS。 */
#define DOF4_SERVO_STEPS_MAX DOF4_SERVO_MAX_POS
#define DOF4_SERVO_STEPS_CENTER DOF4_SERVO_CENTER_POS

/* R_J1: URDF +Z；舵机 position 顺时针增加；URDF 正角时 position 减少。 */
#define R_J1_ZERO_POS DOF4_SERVO_CENTER_POS
#define R_J1_ZERO_BIAS_DEG -1.57f
#define R_J1_SERVO_SIGN (-1)
#define R_J1_URDF_MIN_DEG (-1.135f * DOF4_RAD_TO_DEG)  /* +10° 扩展 */
#define R_J1_URDF_MAX_DEG (3.49f * DOF4_RAD_TO_DEG)

/* R_J2: 实物轴按安装对应 URDF -Y；舵机 position 逆时针增加；URDF 正角时 position 增加。 */
#define R_J2_ZERO_POS DOF4_SERVO_CENTER_POS
#define R_J2_ZERO_BIAS_DEG 2.67f
#define R_J2_SERVO_SIGN (1)
#define R_J2_URDF_MIN_DEG (-2.70f * DOF4_RAD_TO_DEG)
#define R_J2_URDF_MAX_DEG (0.0f * DOF4_RAD_TO_DEG)

/* R_J3: 实物轴按安装对应 URDF -Y；舵机 position 顺时针增加；URDF 正角时 position 减少。 */
#define R_J3_ZERO_POS DOF4_SERVO_CENTER_POS
#define R_J3_ZERO_BIAS_DEG -1.47f
#define R_J3_SERVO_SIGN (-1)
#define R_J3_URDF_MIN_DEG (-4.44f * DOF4_RAD_TO_DEG)
#define R_J3_URDF_MAX_DEG (0.0f * DOF4_RAD_TO_DEG)

/* R_J4: 实物轴按安装对应 URDF +Y；舵机 position 逆时针增加；URDF 正角时 position 增加。 */
#define R_J4_ZERO_POS DOF4_SERVO_CENTER_POS
#define R_J4_ZERO_BIAS_DEG 0.0f
#define R_J4_SERVO_SIGN (1)
#define R_J4_URDF_MIN_DEG (-1.57f * DOF4_RAD_TO_DEG)
#define R_J4_URDF_MAX_DEG (1.57f * DOF4_RAD_TO_DEG)

/**
 * L 臂舵机方向标定说明 ════════════════════════════════════════
 *
 * 以下 L_Jx_SERVO_SIGN 当前为占位值，必须实测验证后方可启用左臂。
 * 验证步骤（对每个关节逐一执行）：
 *   1. 将机械臂摆到全伸直参考位（所有关节角 ≈ 0）
 *   2. 单关节发送 position = 2048+200，观察物理转动方向
 *   3. 若转动方向与 URDF 正角方向一致 → SIGN = +1
 *      若转动方向与 URDF 正角方向相反 → SIGN = -1
 *   4. 更新对应 L_Jx_SERVO_SIGN 并删除本 TODO
 *
 * 同时需实测 ZERO_BIAS_DEG：在全伸直状态下读取 position 值，
 * 反算零位偏置角。当前偏置值直接从右臂镜像，仅作参考。
 * ═══════════════════════════════════════════════════════════════ */

#define L_J1_ZERO_POS DOF4_SERVO_CENTER_POS
#define L_J1_ZERO_BIAS_DEG 1.57f
#define L_J1_SERVO_SIGN (-1) /* ⚠️ TODO: 实测标定后修改；错误符号会导致左臂反向/抖动 */
#define L_J1_URDF_MIN_DEG (-3.75f * DOF4_RAD_TO_DEG)  /* -215°, 下限扩展 15° */
#define L_J1_URDF_MAX_DEG (1.135f * DOF4_RAD_TO_DEG)  /* +65°, 上限扩展 10° */

#define L_J2_ZERO_POS DOF4_SERVO_CENTER_POS
#define L_J2_ZERO_BIAS_DEG 2.67f
#define L_J2_SERVO_SIGN (1) /* ⚠️ TODO: 实测标定后修改；错误符号会导致左臂反向/抖动 */
#define L_J2_URDF_MIN_DEG (-2.70f * DOF4_RAD_TO_DEG)
#define L_J2_URDF_MAX_DEG (0.0f * DOF4_RAD_TO_DEG)

#define L_J3_ZERO_POS DOF4_SERVO_CENTER_POS
#define L_J3_ZERO_BIAS_DEG -1.57f
#define L_J3_SERVO_SIGN (-1) /* ⚠️ TODO: 实测标定后修改；错误符号会导致左臂反向/抖动 */
#define L_J3_URDF_MIN_DEG (-4.44f * DOF4_RAD_TO_DEG)
#define L_J3_URDF_MAX_DEG (0.0f * DOF4_RAD_TO_DEG)

#define L_J4_ZERO_POS DOF4_SERVO_CENTER_POS
#define L_J4_ZERO_BIAS_DEG -0.08f
#define L_J4_SERVO_SIGN (-1) /* ⚠️ TODO: 实测标定后修改；错误符号会导致左臂反向/抖动 */
#define L_J4_URDF_MIN_DEG (-1.57f * DOF4_RAD_TO_DEG)
#define L_J4_URDF_MAX_DEG (1.57f * DOF4_RAD_TO_DEG)



/** @brief 每条机械臂关节数量。 */
#define DOF4_JOINT_COUNT 4U

/** @brief 每条机械臂用于碰撞检测的线段数量。 */
#define DOF4_LINK_SEGMENT_COUNT 4U

/** @brief 默认轨迹最短时间，单位 s。 */
#define DOF4_TRAJ_MIN_DURATION_S 0.15f

/** @brief 默认轨迹最长时间，单位 s。 */
#define DOF4_TRAJ_MAX_DURATION_S 0.50f

/** @brief 默认笛卡尔最大速度，单位 m/s。 */
#define DOF4_DEFAULT_CART_VEL_MPS 2.0f

/** @brief 默认 pitch 最大速度，单位 rad/s。 */
#define DOF4_DEFAULT_PITCH_VEL_RPS 2.00f

/** @brief 默认目标变化重规划阈值，单位 m。 */
#define DOF4_REPLAN_POS_EPS_M 0.001f

/** @brief 默认 pitch 重规划阈值，单位 rad。 */
#define DOF4_REPLAN_PITCH_EPS_RAD 0.003f

/** @brief 默认舵机通信连续失败阈值。 */
#define DOF4_COMM_FAIL_THRESHOLD 5U

/**
 * @brief 状态码。
 */
typedef enum {
    DOF4_STATUS_OK = 0,                ///< 成功
    DOF4_STATUS_NULL_PARAM,            ///< 空参数
    DOF4_STATUS_BAD_CONFIG,            ///< 配置错误
    DOF4_STATUS_OUT_OF_WORKSPACE,      ///< 超出工作空间
    DOF4_STATUS_IK_UNREACHABLE,        ///< IK 无解
    DOF4_STATUS_JOINT_LIMIT,           ///< 关节限位
    DOF4_STATUS_SERVO_LIMIT,           ///< 舵机限位
    DOF4_STATUS_COLLISION_RISK,        ///< 碰撞风险
    DOF4_STATUS_COMM_FAIL,             ///< 通信失败
    DOF4_STATUS_NOT_READY              ///< 未就绪
} Dof4_Status;

/**
 * @brief 机械臂 ID。
 */
typedef enum {
    DOF4_ARM_LEFT = 0,
    DOF4_ARM_RIGHT = 1,
    DOF4_ARM_COUNT = 2
} Dof4_ArmId;

/**
 * @brief 机械臂运行状态。
 */
typedef enum {
    DOF4_ARM_STATE_INIT = 0,
    DOF4_ARM_STATE_IDLE,
    DOF4_ARM_STATE_MOVING,
    DOF4_ARM_STATE_ERROR
} Dof4_ArmState;

/**
 * @brief 单臂目标控制模式。
 */
typedef enum {
    DOF4_CONTROL_MODE_POSE = 0,   /**< 末端位姿目标，经轨迹规划和 IK 控制。 */
    DOF4_CONTROL_MODE_JOINT       /**< 关节角目标，绕过末端 IK 直接下发。 */
} Dof4_ControlMode;

/**
 * @brief 末端 TCP 位姿。
 */
typedef struct {
    float x;      /**< X 坐标，单位 m。 */
    float y;      /**< Y 坐标，单位 m。 */
    float z;      /**< Z 坐标，单位 m。 */
    float pitch;  /**< 末端俯仰角，单位 rad。 */
} Dof4_Pose;

/** @brief 兼容旧命名，语义改为 x/y/z/pitch。 */
typedef Dof4_Pose Dof4_EndEffectorPose;

/**
 * @brief 四关节状态。
 */
typedef struct {
    float q[DOF4_JOINT_COUNT];  /**< J1~J4 关节角，单位 rad。 */
} Dof4_JointState;

/**
 * @brief 机械臂配置。
 */
typedef struct {
    Dof4_ArmId arm_id;                     /**< 左臂或右臂。 */
    uint8_t servo_id[DOF4_JOINT_COUNT];    /**< 舵机 ID。 */

    float base[3];                         /**< J1 在 base_link 下的位置，单位 m。 */
    float base_offset[3];                  /**< 用户额外基座偏移，单位 m。 */
    float shoulder_r;                      /**< J1 到 J2 的水平等效偏移，单位 m。 */
    float shoulder_z;                      /**< J1 到 J2 的 Z 偏移，单位 m。 */
    float link_len[3];                     /**< J2-J3、J3-J4、J4-TCP 等效长度，单位 m。 */
    float tcp_offset[3];                   /**< 保留给吸盘 TCP 标定的偏移，单位 m。 */
    float pitch_offset;                    /**< pitch 零位偏置，单位 rad。 */

    float joint_min[DOF4_JOINT_COUNT];     /**< 关节下限，单位 rad。 */
    float joint_max[DOF4_JOINT_COUNT];     /**< 关节上限，单位 rad。 */

    int16_t servo_min[DOF4_JOINT_COUNT];   /**< 舵机步进下限。 */
    int16_t servo_max[DOF4_JOINT_COUNT];   /**< 舵机步进上限。 */
    int16_t servo_zero[DOF4_JOINT_COUNT];  /**< 舵机零位步进。 */
    float servo_offset[DOF4_JOINT_COUNT];  /**< 舵机角度零偏，单位 rad。 */
    int8_t servo_sign[DOF4_JOINT_COUNT];   /**< 舵机角度方向，取 +1 或 -1。 */
    uint8_t servo_reverse[DOF4_JOINT_COUNT]; /**< 舵机反向安装标志。 */
    uint16_t servo_speed;                  /**< 舵机写入速度参数。 */
    uint8_t servo_acc;                     /**< 舵机写入加速度参数。 */

    float ws_min[3];                       /**< 工作空间下限 x/y/z，单位 m。 */
    float ws_max[3];                       /**< 工作空间上限 x/y/z，单位 m。 */

    float cart_vel_mps;                    /**< 笛卡尔规划速度上限，单位 m/s。 */
    float pitch_vel_rps;                   /**< pitch 规划速度上限，单位 rad/s。 */
} Dof4_ArmConfig;

/**
 * @brief 全局世界坐标系原点偏移。
 *
 * @details
 * 用于将机械臂内部几何坐标系（arm frame）与外部雷达/世界坐标系（world frame）
 * 对齐。所有 FK 输出和 IK 输入均会叠加此偏移：
 *
 * ```text
 * 世界坐标 = 机械臂几何坐标 + world_offset
 * ```
 *
 * 偏移仅作用于位置 (x, y, z)，不影响 pitch 俯仰角。
 * 当前默认值为 0，后续通过 `Dof4_set_world_offset()` 标定雷达对齐。
 */
typedef struct {
    float x;  /**< 世界原点 X 偏移，单位 m。 */
    float y;  /**< 世界原点 Y 偏移，单位 m。 */
    float z;  /**< 世界原点 Z 偏移，单位 m。 */
} Dof4_WorldOffset;

/**
 * @brief 单臂运行实例。
 */
typedef struct {
    Dof4_ArmConfig cfg;                    /**< 当前配置。 */
    Dof4_ArmState state;                   /**< 当前状态。 */

    Dof4_JointState joint_actual;          /**< 当前真实反馈关节角，可超出 URDF 限位。 */
    Dof4_JointState joint_target;          /**< 当前安全目标关节角，限制在 URDF 限位内。 */
    int16_t servo_pos[DOF4_JOINT_COUNT];   /**< 当前反馈舵机位置。 */
    int16_t target_servo_pos[DOF4_JOINT_COUNT]; /**< 当前目标舵机位置。 */

    Dof4_Pose current_pose;                /**< 当前 FK 位姿。 */
    Dof4_Pose target_pose;                 /**< 用户目标位姿。 */
    bool target_valid;                     /**< 用户目标是否有效。 */
    Dof4_ControlMode control_mode;         /**< 当前目标控制模式。 */

    float joint_world[DOF4_LINK_SEGMENT_COUNT + 1U][3]; /**< J1、J2、J3、J4、TCP 世界坐标。 */
    uint8_t comm_fail_count;               /**< 连续通信失败次数。 */
    Dof4_Status last_status;               /**< 最近一次错误码。 */
} Dof4_Arm;

extern Dof4_Arm g_dof4_arm_left;
extern Dof4_Arm g_dof4_arm_right;

/** @brief 全局世界坐标系偏移实例，双臂共享，默认 {0,0,0}。 */
extern Dof4_WorldOffset g_dof4_world_offset;

/** @brief 双臂启动标志位：false=等待启动指令，true=控制循环放行。 */
extern bool g_dof4_arm_started;

/**
 * @brief 根据 URDF 默认参数生成单臂配置。
 * @param arm_id 机械臂 ID。
 * @retval Dof4_ArmConfig 默认配置对象。
 */
Dof4_ArmConfig Dof4_arm_default_config(Dof4_ArmId arm_id);

/**
 * @brief 初始化单臂实例。
 * @param arm 机械臂实例。
 * @param config 配置对象。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_config_init(Dof4_Arm *arm, const Dof4_ArmConfig *config);

/**
 * @brief 初始化左右双臂。
 * @param arm_left 左臂实例。
 * @param arm_right 右臂实例。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_dual_arm_init(Dof4_Arm *arm_left, Dof4_Arm *arm_right);

/**
 * @brief 设置基座附加偏移。
 * @param arm 机械臂实例。
 * @param dx X 偏移，单位 m。
 * @param dy Y 偏移，单位 m。
 * @param dz Z 偏移，单位 m。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_base_offset(Dof4_Arm *arm, float dx, float dy, float dz);

/**
 * @brief 设置 TCP 偏移。
 * @param arm 机械臂实例。
 * @param dx X 偏移，单位 m。
 * @param dy Y 偏移，单位 m。
 * @param dz Z 偏移，单位 m。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_tcp_offset(Dof4_Arm *arm, float dx, float dy, float dz);

/**
 * @brief 设置 pitch 零位偏置。
 * @param arm 机械臂实例。
 * @param pitch_offset pitch 偏置，单位 rad。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_pitch_offset(Dof4_Arm *arm, float pitch_offset);

/**
 * @brief 设置舵机零位偏置。
 * @param arm 机械臂实例。
 * @param joint_index 关节索引 0~3。
 * @param zero_pos 零位步进。
 * @param offset_rad 角度零偏，单位 rad。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_servo_offset(Dof4_Arm *arm,
                                      uint8_t joint_index,
                                      int16_t zero_pos,
                                      float offset_rad);

/**
 * @brief 几何正运动学。
 * @param arm 机械臂实例。
 * @param joints 输入关节角。
 * @param pose 输出 TCP 位姿。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_forward_kinematics(Dof4_Arm *arm,
                                        const Dof4_JointState *joints,
                                        Dof4_Pose *pose);

/**
 * @brief 几何逆运动学。
 *
 * 调用方通过 @p elbow_sign 显式指定肘型：
 * - `+1.0f` = 肘上解（elbow up）
 * - `-1.0f` = 肘下解（elbow down）
 *
 * @param arm        机械臂实例。
 * @param target     目标 TCP 位姿。
 * @param elbow_sign 肘型符号，+1=肘上，-1=肘下。
 * @param joints     输出关节角。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_inverse_kinematics(Dof4_Arm *arm,
                                        const Dof4_Pose *target,
                                        float elbow_sign,
                                        Dof4_JointState *joints);

/**
 * @brief 从舵机步进转换到真实反馈关节角，不做 URDF 关节限位钳制。
 * @param arm 机械臂实例。
 * @param joint_index 关节索引 0~3。
 * @param servo_pos 舵机步进值。
 * @param angle_rad 输出关节角，单位 rad。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_servo_to_angle(const Dof4_Arm *arm,
                                uint8_t joint_index,
                                int16_t servo_pos,
                                float *angle_rad);

/**
 * @brief 从目标关节角转换到舵机步进，目标角会先限制到 URDF 安全范围。
 * @param arm 机械臂实例。
 * @param joint_index 关节索引 0~3。
 * @param angle_rad 关节角，单位 rad。
 * @param servo_pos 输出舵机步进值。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_angle_to_servo(const Dof4_Arm *arm,
                                uint8_t joint_index,
                                float angle_rad,
                                int16_t *servo_pos);

/**
 * @brief 设置单臂目标位姿并触发外部控制循环后续重规划。
 * @param arm 机械臂实例。
 * @param target_x 目标 X，单位 m。
 * @param target_y 目标 Y，单位 m。
 * @param target_z 目标 Z，单位 m。
 * @param target_pitch 目标 pitch，单位 rad。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_target(Dof4_Arm *arm,
                                float target_x,
                                float target_y,
                                float target_z,
                                float target_pitch);

/**
 * @brief 设置单臂关节目标并切换到关节控制模式。
 * @param arm 机械臂实例。
 * @param joints 目标关节角，单位 rad。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_joint_target(Dof4_Arm *arm,
                                      const Dof4_JointState *joints);

/**
 * @brief 读取单臂舵机反馈。
 * @param arm 机械臂实例。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_read_servo_pos(Dof4_Arm *arm);

/**
 * @brief 读取双臂舵机反馈。
 * @param arm_left 左臂实例。
 * @param arm_right 右臂实例。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_batch_read_all_servo(Dof4_Arm *arm_left, Dof4_Arm *arm_right);

/**
 * @brief 写入双臂舵机目标。
 * @param arm_left 左臂实例。
 * @param arm_right 右臂实例。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_batch_write_all_servo(const Dof4_Arm *arm_left,
                                       const Dof4_Arm *arm_right);

/**
 * @brief 双臂一步式控制循环，由外部任务按目标频率调用。
 * @param arm_left 左臂实例。
 * @param arm_right 右臂实例。
 * @param now_ms 当前时间戳，单位 ms。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_dual_arm_control_loop(Dof4_Arm *arm_left,
                                       Dof4_Arm *arm_right,
                                       uint32_t now_ms);

Dof4_Status Dof4_dual_arm_startup_pose(Dof4_Arm *arm_left,
                                       Dof4_Arm *arm_right,
                                       const Dof4_Pose *target_left,
                                       const Dof4_Pose *target_right,
                                       uint32_t timeout_ms,
                                       float pos_tol_m,
                                       float pitch_tol_rad);

/**
 * @brief 将位姿限制在工作空间内。
 * @param arm 机械臂实例。
 * @param pose 输入输出位姿。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_clamp_to_workspace(const Dof4_Arm *arm, Dof4_Pose *pose);

/**
 * @brief 获取机械臂连杆端点。
 * @param arm 机械臂实例。
 * @param endpoints 输出线段端点 `[4][2][3]`。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_get_link_endpoints(const Dof4_Arm *arm,
                                        float endpoints[DOF4_LINK_SEGMENT_COUNT][2][3]);

/**
 * @brief 根据 ID 获取机械臂实例。
 * @param arm_id 机械臂 ID。
 * @param left 左臂实例。
 * @param right 右臂实例。
 * @retval Dof4_Arm* 对应实例，非法 ID 返回 NULL。
 */
Dof4_Arm *Dof4_arm_get_by_id(Dof4_ArmId arm_id, Dof4_Arm *left, Dof4_Arm *right);

/**
 * @brief 角度归一化到 [-pi, pi]。
 * @param angle_rad 输入角度，单位 rad。
 * @retval float 归一化后的角度。
 */
float Dof4_normalize_angle(float angle_rad);

/**
 * @brief 设置全局世界坐标系原点偏移。
 *
 * 偏移用于对齐雷达/外部坐标系。世界坐标 = 机械臂几何坐标 + world_offset。
 * FK 输出会自动叠加此偏移，IK 输入会自动扣除。
 *
 * @param dx X 偏移，单位 m。
 * @param dy Y 偏移，单位 m。
 * @param dz Z 偏移，单位 m。
 * @retval Dof4_Status 总是返回 DOF4_STATUS_OK。
 */
Dof4_Status Dof4_set_world_offset(float dx, float dy, float dz);

/**
 * @brief 读取全局世界坐标系原点偏移。
 * @param dx 输出 X 偏移，单位 m（可为 NULL）。
 * @param dy 输出 Y 偏移，单位 m（可为 NULL）。
 * @param dz 输出 Z 偏移，单位 m（可为 NULL）。
 * @retval Dof4_Status 总是返回 DOF4_STATUS_OK。
 */
Dof4_Status Dof4_get_world_offset(float *dx, float *dy, float *dz);

/**
 * @brief 设置双臂启动标志位，放行主控制循环。
 *
 * 调用后 g_dof4_arm_started 置 true，arm_control_task 的主循环
 * 在下一帧检测到后开始执行正常的控制流水线。置位不可逆。
 */
void Dof4_double_arm_start(void);

void Dof4_double_arm_Desable(void);
void Dof4_double_arm_Enable(void);




#ifdef __cplusplus
}
#endif

#endif /* DOF4_ARM_H */
