/**
 * @file Dof4_Arm.c
 * @brief 双四自由度机械臂几何解析运动学与一步式控制循环实现。
 *
 * @details
 * ## 现代 DH / 修正 DH 规则说明
 *
 * 本文件的机械臂参数来自 `double_arm4_urdf.urdf`。URDF 中左右两条机械臂均可
 * 等效为经典 4DOF 结构：
 *
 * - J1：基座绕 `base_link` 的 Z 轴旋转，用于确定垂直工作平面；
 * - J2：肩关节，在 J1 确定的垂直平面内俯仰；
 * - J3：肘关节，在同一垂直平面内俯仰；
 * - J4：腕关节，用于满足末端 `pitch` 俯仰角；
 * - TCP：吸盘中心，可通过 `tcp_offset` 和 `pitch_offset` 后续标定。
 *
 * 若按 Craig 约定的现代 DH，也称 Modified DH，单节变换写为：
 *
 * ```text
 * ^{i-1}T_i =
 *     RotX(alpha_{i-1}) *
 *     TransX(a_{i-1}) *
 *     RotZ(theta_i) *
 *     TransZ(d_i)
 * ```
 *
 * 其中：
 *
 * - `alpha_{i-1}`：上一关节轴 `Z_{i-1}` 到当前关节轴 `Z_i` 绕 `X_{i-1}` 的扭转角；
 * - `a_{i-1}`：上一关节轴到当前关节轴沿 `X_{i-1}` 的公法线长度；
 * - `theta_i`：绕当前关节轴 `Z_i` 的旋转关节变量；
 * - `d_i`：沿当前关节轴 `Z_i` 的偏移量。
 *
 * 完整矩阵链可写为：
 *
 * ```text
 * ^0T_TCP =
 *     ^0T_1(theta_1) *
 *     ^1T_2(theta_2) *
 *     ^2T_3(theta_3) *
 *     ^3T_4(theta_4) *
 *     ^4T_TCP
 * ```
 *
 * 从 URDF 零位轴向换算可知：
 *
 * - J1 轴近似为 `base_link` 下的 `+Z`；
 * - J2/J3/J4 轴在零位下近似互相平行，并近似平行于水平 `Y` 方向；
 * - 因此完整 Modified DH 链可以进一步降维成“J1 水平旋转 + J2/J3/J4
 *   垂直平面 3R”的几何解析模型。
 *
 * 为降低 STM32F407 上的实时计算量，本实现不在 200 Hz 控制循环中逐周期计算
 * 完整 4x4 DH 矩阵，也不做数值迭代 IK。初始化配置中将 URDF / Modified DH 信息
 * 折算为以下等效几何参数：
 *
 * | 等效参数 | 来源 | 含义 |
 * | --- | --- | --- |
 * | `base[3]` | URDF J1 origin | J1 在 `base_link` 下的位置 |
 * | `shoulder_r` | J1->J2 origin 的 XY 投影 | J1 到 J2 的水平等效偏移 |
 * | `shoulder_z` | J1->J2 origin 的 Z 分量 | J1 到 J2 的高度偏移 |
 * | `link_len[0]` | J2->J3 | 上臂等效长度 |
 * | `link_len[1]` | J3->J4 | 前臂等效长度 |
 * | `link_len[2]` | J4->TCP | 末端工具等效长度 |
 * | `pitch_offset` | 末端安装标定 | TCP pitch 零位偏置 |
 *
 * 在该等效模型中，正运动学为：
 *
 * ```text
 * theta2   = q2
 * theta3   = q3
 * theta4   = q4
 * phi      = theta2 + theta3 + theta4
 *
 * r_J2  = shoulder_r
 * z_J2  = shoulder_z
 * r_J3  = r_J2 + L2 * cos(theta2)
 * z_J3  = z_J2 + L2 * sin(theta2)
 * r_J4  = r_J3 + L3 * cos(theta2 + theta3)
 * z_J4  = z_J3 + L3 * sin(theta2 + theta3)
 * r_TCP = r_J4 + LT * cos(phi)
 * z_TCP = z_J4 + LT * sin(phi)
 *
 * x_TCP = base_x + cos(q1) * r_TCP + tcp_offset_x
 * y_TCP = base_y + sin(q1) * r_TCP + tcp_offset_y
 * z_TCP = base_z + z_TCP           + tcp_offset_z
 * pitch = phi + pitch_offset
 * ```
 *
 * 逆运动学为：
 *
 * ```text
 * q1 = atan2(target_y - base_y, target_x - base_x)
 *
 * r = sqrt(dx^2 + dy^2) - shoulder_r
 * z = target_z - base_z - shoulder_z
 * phi = target_pitch - pitch_offset
 *
 * wrist_r = r - LT * cos(phi)
 * wrist_z = z - LT * sin(phi)
 *
 * cos(theta3) =
 *     (wrist_r^2 + wrist_z^2 - L2^2 - L3^2) / (2 * L2 * L3)
 *
 * theta3 = atan2(elbow_sign * sqrt(1 - cos(theta3)^2), cos(theta3))
 * theta2 = atan2(wrist_z, wrist_r)
 *        - atan2(L3 * sin(theta3), L2 + L3 * cos(theta3))
 * theta4 = phi - theta2 - theta3
 *
 * q2 = theta2
 * q3 = theta3
 * q4 = theta4
 * ```
 *
 * `elbow_sign` 由调用方在 `Dof4_arm_inverse_kinematics()` 中显式传入：
 * `+1.0f` = 肘上解，`-1.0f` = 肘下解。这样既保留了现代 DH/URDF 的
 * 参数来源，也满足 F407 上低计算量、确定性强的实时控制需求。
 *
 * 当前代码为了 STM32F407 的实时性，把该矩阵链展开成三角函数形式执行。

 * 当前实现还包含了碰撞检测、轨迹规划等功能，但不在本文件中展开说明。
 *此外0点的偏置需要在机械臂全向x轴伸直的状态下测量得到，并且需要尽可能保证安装的机械臂与 URDF 中的轴向定义一致。
 */

#include "Dof4_Arm.h"
#include "Dof4_Collision.h"
#include "Trajectory_Planning.h"

#include <math.h>
#include <string.h>

#ifndef DOF4_HOST_TEST
#include "pneumatic_control.h"
#include "cmsis_os.h"
#include "SCS.h"
#include "SCSCL.h"
#include "SCSerail.h"
#include "SMS_STS.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#else
static void dof4_host_relay_control(uint8_t relay_id, uint8_t state)
{
    (void)relay_id;
    (void)state;
}
static void dof4_host_enable_torque(int servo_id, bool enable)
{
    (void)servo_id;
    (void)enable;
}
#define relay_control dof4_host_relay_control
#define EnableTorque dof4_host_enable_torque
#endif

#define RELAY_LEFT_ARM    0U  /**< 左臂吸盘电磁阀（一号） */
#define RELAY_RIGHT_ARM   1U  /**< 右臂吸盘电磁阀（二号） */
#define RELAY_LEFT_BACK   2U  /**< 左背吸盘电磁阀（三号） */
#define RELAY_RIGHT_BACK  3U  /**< 右背吸盘电磁阀（四号） */
#define SUCTION_ON        1
#define SUCTION_OFF       0


/// ════════════════════════════════════════════════════════════════

//两个后侧吸盘的间距是265mm，两个基座的距离是266

/// ════════════════════════════════════════════════════════════════

/** @brief URDF 左臂 J1 位置 X，单位 m。 */
#define DOF4_URDF_L_BASE_X 0.0f
/** @brief URDF 左臂 J1 位置 Y，单位 m。 */
#define DOF4_URDF_L_BASE_Y 0.13342f
/** @brief URDF 左臂 J1 位置 Z，单位 m。 */
#define DOF4_URDF_L_BASE_Z 0.0f
/** @brief URDF 右臂 J1 位置 X，单位 m。 */
#define DOF4_URDF_R_BASE_X 0.0f
/** @brief URDF 右臂 J1 位置 Y，单位 m。 */
#define DOF4_URDF_R_BASE_Y (-0.13265f)
/** @brief URDF 右臂 J1 位置 Z，单位 m。 */
#define DOF4_URDF_R_BASE_Z 0.0f

// /** @brief URDF 左臂 J1 位置 X，单位 m。 */
// #define DOF4_URDF_L_BASE_X 0
// /** @brief URDF 左臂 J1 位置 Y，单位 m。 */
// #define DOF4_URDF_L_BASE_Y 0
// /** @brief URDF 左臂 J1 位置 Z，单位 m。 */
// #define DOF4_URDF_L_BASE_Z 0
// /** @brief URDF 右臂 J1 位置 X，单位 m。 */
// #define DOF4_URDF_R_BASE_X 0
// /** @brief URDF 右臂 J1 位置 Y，单位 m。 */
// #define DOF4_URDF_R_BASE_Y 
// /** @brief URDF 右臂 J1 位置 Z，单位 m。 */
// #define DOF4_URDF_R_BASE_Z 0

/** @brief URDF J1 到 J2 水平等效偏移，单位 m。 */
#define DOF4_URDF_SHOULDER_R 0.02716f
/** @brief URDF J1 到 J2 Z 偏移，单位 m。 */
#define DOF4_URDF_SHOULDER_Z 0.022319f
/** @brief URDF J2-J3 等效长度，单位 m。 */
#define DOF4_URDF_LINK2_LEN 0.35548f
/** @brief URDF J3-J4 等效长度，单位 m。 */
#define DOF4_URDF_LINK3_LEN 0.27130f
/** @brief URDF 左臂 J4-TCP 等效长度，单位 m。 */
#define DOF4_URDF_L_TOOL_LEN 0.03600f
/** @brief URDF 右臂 J4-TCP 等效长度，单位 m。 */
#define DOF4_URDF_R_TOOL_LEN 0.03620f

/** @brief 左臂 J1 下限，单位 rad。 */
#define DOF4_L_J1_MIN (-3.75f)  /* -215°, 下限较 URDF 原值扩展 15° */
/** @brief 左臂 J1 上限，单位 rad。+10° 扩展 (原 55° → 65°) */
#define DOF4_L_J1_MAX 1.135f
/** @brief 右臂 J1 下限，单位 rad。+10° 扩展 (原 -55° → -65°) */
#define DOF4_R_J1_MIN (-1.135f)
/** @brief 右臂 J1 上限，单位 rad。 */
#define DOF4_R_J1_MAX 3.49f
/** @brief J2 相对舵机中位的原有效范围，初始化时换算为绝对关节角。 */
#define DOF4_J2_SERVO_REL_MIN (-2.70f)
#define DOF4_J2_SERVO_REL_MAX 1.75f
/** @brief J3 相对舵机中位的下限；绝对上限允许到 q3=0 的伸直姿态。 */
#define DOF4_J3_SERVO_REL_MIN (-4.44f)
#define DOF4_J3_ABS_MAX 0.0f
/** @brief J4 相对舵机中位的原有效范围，初始化时换算为绝对关节角。 */
#define DOF4_J4_SERVO_REL_MIN (-1.68f)
#define DOF4_J4_SERVO_REL_MAX 1.68f

/** @brief 几何解算退化阈值。 */
#define DOF4_GEOM_EPS 1.0e-6f
/** @brief J1 轴线附近保持当前方位角的水平距离阈值，单位 m。 */
#define DOF4_AXIS_SINGULAR_EPS_M 0.001f
/** @brief 2R 余弦定理数值容差。 */
#define DOF4_COS_EPS 1.0e-5f
/** @brief Startup pose control-loop period, in ms. */
#define DOF4_STARTUP_CONTROL_PERIOD_MS 5U

/** @brief 碰撞规避：沿最近距离方向推开右臂的距离，单位 m。 */
#define DOF4_AVOID_PUSH_M      0.25f
/** @brief 碰撞规避：附加 Z 抬高量，单位 m。 */
#define DOF4_AVOID_Z_LIFT_M    0.05f
/** @brief 碰撞规避：位姿到位判定容差，单位 m。 */
#define DOF4_AVOID_REACH_TOL_M 0.03f
/** @brief 碰撞规避：各阶段超时，单位 ms。 */
#define DOF4_AVOID_TIMEOUT_MS  3000U

/* URDF joint origins/rpy are kept in meters/radians and applied as:
 * T_parent_child = Origin(xyz, rpy) * Rot(axis, q).
 */

#define DOF4_URDF_J2_X 0.015518f
#define DOF4_URDF_J2_Y 0.022295f
#define DOF4_URDF_J2_Z 0.022319f
#define DOF4_URDF_J2_ROLL 1.565f
#define DOF4_URDF_J2_PITCH (-0.52357f)
#define DOF4_URDF_J2_YAW (-3.13f)
#define DOF4_URDF_J2_AXIS_X 0.0099995f
#define DOF4_URDF_J2_AXIS_Y 0.0f
#define DOF4_URDF_J2_AXIS_Z (-0.99995f)

#define DOF4_URDF_J3_X 0.35543f
#define DOF4_URDF_J3_Y 0.0f
#define DOF4_URDF_J3_Z 0.0062281f

#define DOF4_URDF_L_J3_ROLL (-3.1358f)
#define DOF4_URDF_L_J3_PITCH 0.0f
#define DOF4_URDF_L_J3_YAW 0.0f
#define DOF4_URDF_L_J3_AXIS_X 0.0f
#define DOF4_URDF_L_J3_AXIS_Y (-0.0057731f)
#define DOF4_URDF_L_J3_AXIS_Z (-0.99998f)

#define DOF4_URDF_R_J3_ROLL 0.0f
#define DOF4_URDF_R_J3_PITCH 0.0f
#define DOF4_URDF_R_J3_YAW 3.1416f
#define DOF4_URDF_R_J3_AXIS_X 0.0f
#define DOF4_URDF_R_J3_AXIS_Y 0.0f
#define DOF4_URDF_R_J3_AXIS_Z 1.0f

#define DOF4_URDF_L_J4_X (-0.25443f)
#define DOF4_URDF_L_J4_Y (-0.094072f)
#define DOF4_URDF_L_J4_Z 0.0045432f
#define DOF4_URDF_L_J4_ROLL 0.0055764f
#define DOF4_URDF_L_J4_PITCH (-0.0014942f)
#define DOF4_URDF_L_J4_YAW (-2.8798f)
#define DOF4_URDF_L_J4_AXIS_X 0.0f
#define DOF4_URDF_L_J4_AXIS_Y 0.0f
#define DOF4_URDF_L_J4_AXIS_Z (-1.0f)

#define DOF4_URDF_R_J4_X 0.25443f
#define DOF4_URDF_R_J4_Y (-0.094096f)
#define DOF4_URDF_R_J4_Z (-0.004f)
#define DOF4_URDF_R_J4_ROLL 0.0f
#define DOF4_URDF_R_J4_PITCH 0.0f
#define DOF4_URDF_R_J4_YAW (-0.52358f)
#define DOF4_URDF_R_J4_AXIS_X 0.0f
#define DOF4_URDF_R_J4_AXIS_Y 0.0f
#define DOF4_URDF_R_J4_AXIS_Z 1.0f

typedef struct {
    float r[3][3];
    float p[3];
} Dof4_Transform;

extern Dof4_Arm g_dof4_arm_left;
extern Dof4_Arm g_dof4_arm_right;

/** @brief 全局世界坐标系原点偏移实例。 */
Dof4_WorldOffset g_dof4_world_offset = {0.0f, 0.0f, 0.0f};

/** @brief 双臂启动标志位：false=等待启动指令，true=控制循环放行。 */
bool g_dof4_arm_started = false;

static Dof4_CartesianPlanner g_planner_left;
static Dof4_CartesianPlanner g_planner_right;

/* ════════════════════════════════════════════════════════════════
 * 碰撞规避状态机
 * ════════════════════════════════════════════════════════════════ */

/** @brief 碰撞规避子状态 */
typedef enum {
    DOF4_AVOID_NORMAL          = 0,  /**< 正常模式，双臂自由移动 */
    DOF4_AVOID_RIGHT_RETREAT   = 1,  /**< 右臂撤退到安全位 */
    DOF4_AVOID_LEFT_ADVANCE    = 2,  /**< 左臂向目标前进 */
    DOF4_AVOID_RIGHT_ADVANCE   = 3,  /**< 右臂向目标前进 */
} Dof4_AvoidState;

static Dof4_AvoidState   s_avoid_state;
static Dof4_Pose         s_saved_left_target;   /**< 碰撞触发时保存的左臂目标 */
static Dof4_Pose         s_saved_right_target;  /**< 碰撞触发时保存的右臂目标 */
static Dof4_Pose         s_retreat_pose_right;  /**< 动态计算的右臂安全位 */
static uint32_t          s_avoid_enter_tick;    /**< 进入当前规避子状态的 tick */

#ifdef DOF4_HOST_TEST
static uint32_t g_dof4_host_tick_ms;
#endif

static uint32_t dof4_get_tick_ms(void)
{
#ifndef DOF4_HOST_TEST
    return HAL_GetTick();
#else
    return g_dof4_host_tick_ms;
#endif
}

static void dof4_delay_ms(uint32_t delay_ms)
{
#ifndef DOF4_HOST_TEST
    osDelay(delay_ms);
#else
    g_dof4_host_tick_ms += delay_ms;
#endif
}

/**
 * @brief 将浮点值限制到指定范围。
 * @param value 输入值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @retval float 限幅后的值。
 */
static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/**
 * @brief 将 int32 值限制到 int16 范围。
 * @param value 输入值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @retval int16_t 限幅后的值。
 */
static int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return (int16_t)value;
}

static int32_t round_servo_steps(float steps_f)
{
    return (int32_t)((steps_f >= 0.0f) ? (steps_f + 0.5f) : (steps_f - 0.5f));
}

static int32_t joint_angle_to_servo_steps_raw(const Dof4_Arm *arm,
                                              uint8_t joint_index,
                                              float angle_rad)
{
    float servo_angle = (angle_rad - arm->cfg.servo_offset[joint_index]) *
                        (float)arm->cfg.servo_sign[joint_index];
    if (arm->cfg.servo_reverse[joint_index] != 0U) {
        servo_angle = -servo_angle;
    }

    const float steps_f = (float)arm->cfg.servo_zero[joint_index] +
                          servo_angle * DOF4_SERVO_POS_PER_RAD;
    return round_servo_steps(steps_f);
}

static Dof4_Status j1_angle_to_servo_equivalent(const Dof4_Arm *arm,
                                                float angle_rad,
                                                float *selected_angle_rad,
                                                int16_t *servo_pos)
{
    if (arm == NULL || selected_angle_rad == NULL || servo_pos == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (arm->cfg.servo_sign[0] == 0) {
        return DOF4_STATUS_BAD_CONFIG;
    }

    bool have_valid = false;
    float best_valid_delta = 0.0f;
    float best_valid_angle = angle_rad;
    int32_t best_valid_steps = 0;

    bool have_fallback = false;
    int32_t best_fallback_overflow = 0;
    float best_fallback_angle = angle_rad;
    int32_t best_fallback_steps = 0;

    for (int32_t k = -2; k <= 2; ++k) {
        const float candidate = angle_rad + (float)k * 2.0f * M_PI_F;
        const int32_t steps = joint_angle_to_servo_steps_raw(arm, 0U, candidate);

        if (steps >= arm->cfg.servo_min[0] && steps <= arm->cfg.servo_max[0]) {
            const float delta = fabsf(candidate - angle_rad);
            if (!have_valid || delta < best_valid_delta) {
                have_valid = true;
                best_valid_delta = delta;
                best_valid_angle = candidate;
                best_valid_steps = steps;
            }
            continue;
        }

        const int32_t overflow = (steps < arm->cfg.servo_min[0])
                                 ? ((int32_t)arm->cfg.servo_min[0] - steps)
                                 : (steps - (int32_t)arm->cfg.servo_max[0]);
        if (!have_fallback || overflow < best_fallback_overflow) {
            have_fallback = true;
            best_fallback_overflow = overflow;
            best_fallback_angle = candidate;
            best_fallback_steps = steps;
        }
    }

    if (have_valid) {
        *selected_angle_rad = best_valid_angle;
        *servo_pos = (int16_t)best_valid_steps;
        return DOF4_STATUS_OK;
    }

    *selected_angle_rad = best_fallback_angle;
    *servo_pos = clamp_i16(best_fallback_steps,
                           arm->cfg.servo_min[0],
                           arm->cfg.servo_max[0]);
    return DOF4_STATUS_SERVO_LIMIT;
}

/**
 * @brief 将实际/URDF 关节角夹紧到 URDF 限位内。
 * @param arm 机械臂实例。
 * @param joint_index 关节索引。
 * @param angle_rad 输入角度，单位 rad。
 * @retval float 限位后的角度，单位 rad。
 */
static float clamp_joint_angle_rad(const Dof4_Arm *arm, uint8_t joint_index, float angle_rad)
{
    return clamp_float(angle_rad,
                       arm->cfg.joint_min[joint_index],
                       arm->cfg.joint_max[joint_index]);
}

/**
 * @brief 计算机械臂基座位置。
 * @param arm 机械臂实例。
 * @param base 输出基座位置。
 * @retval none 无。
 */
static void get_effective_base(const Dof4_Arm *arm, float base[3])
{
    base[0] = arm->cfg.base[0] + arm->cfg.base_offset[0];
    base[1] = arm->cfg.base[1] + arm->cfg.base_offset[1];
    base[2] = arm->cfg.base[2] + arm->cfg.base_offset[2];
}

/**
 * @brief 检查位姿是否位于工作空间内。
 * @param arm 机械臂实例。
 * @param pose 待检查位姿。
 * @retval Dof4_Status 状态码。
 */
static Dof4_Status check_workspace(const Dof4_Arm *arm, const Dof4_Pose *pose)
{
    if (arm == NULL || pose == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    /* workspace 边界随 base_offset 同步偏移，保证物理 TCP 位置一致 */
    const float eff_ws_min_x = arm->cfg.ws_min[0] + arm->cfg.base_offset[0];
    const float eff_ws_max_x = arm->cfg.ws_max[0] + arm->cfg.base_offset[0];
    const float eff_ws_min_y = arm->cfg.ws_min[1] + arm->cfg.base_offset[1];
    const float eff_ws_max_y = arm->cfg.ws_max[1] + arm->cfg.base_offset[1];
    const float eff_ws_min_z = arm->cfg.ws_min[2] + arm->cfg.base_offset[2];
    const float eff_ws_max_z = arm->cfg.ws_max[2] + arm->cfg.base_offset[2];

    if (pose->x < eff_ws_min_x || pose->x > eff_ws_max_x ||
        pose->y < eff_ws_min_y || pose->y > eff_ws_max_y ||
        pose->z < eff_ws_min_z || pose->z > eff_ws_max_z) {
        return DOF4_STATUS_OUT_OF_WORKSPACE;
    }
    return DOF4_STATUS_OK;
}

/**
 * @brief 生成一个肘型 IK 候选解（平面 3R 几何解析法，对标 {P} 平面坐标系）。
 *
 * @details
 * ## 几何推导
 *
 * 已知目标位姿 `(x, y, z, pitch)`（世界坐标系），求关节角 `(q1, q2, q3, q4)`。
 *
 * ```
 * Step 1 — 确定工作平面和 J1：
 *   dx = target_x - base_x,  dy = target_y - base_y
 *   planar_dist = sqrt(dx² + dy²)
 *   q1 = atan2(dy, dx)                               ← 水平旋转角
 *   r  = planar_dist - shoulder_r                    ← 肩部→目标 径向距离
 *   z  = target_z - base_z - shoulder_z              ← 肩部→目标 高度差
 *
 * Step 2 — 解耦末端工具（LT），得到腕部位置：
 *   phi     = target_pitch - pitch_offset            ← 目标 TCP 俯仰解析角
 *   wrist_r = r  - LT * cos(phi)                     ← 腕部径向（去工具）
 *   wrist_z = z  - LT * sin(phi)                     ← 腕部高度（去工具）
 *
 * Step 3 — 2R 余弦定理求 theta3：
 *   cos(theta3) = (wrist_r² + wrist_z² - L2² - L3²) / (2 * L2 * L3)
 *   theta3 = atan2(elbow_sign * sqrt(1-cos²), cos)   ← ±肘型
 *
 * Step 4 — 几何关系求 theta2：
 *   theta2 = atan2(wrist_z, wrist_r)
 *          - atan2(L3*sin(theta3), L2 + L3*cos(theta3))
 *
 * Step 5 — 腕部补偿：
 *   theta4 = phi - theta2 - theta3
 *
 * Step 6 — 解析角 → URDF 关节角（q = theta，同号）：
 *   q2 = theta2,  q3 = theta3,  q4 = theta4
 * ```
 *
 * @param arm        机械臂实例。
 * @param target     目标 TCP 位姿（世界坐标系）。
 * @param elbow_sign 肘型符号：+1 = 肘上，-1 = 肘下。
 * @param joints     输出关节角 q[0..3]（单位 rad）。
 * @retval Dof4_Status 状态码。
 */
static Dof4_Status solve_ik_candidate(const Dof4_Arm *arm,
                                      const Dof4_Pose *target,
                                      float elbow_sign,
                                      Dof4_JointState *joints)
{
    float base[3];
    get_effective_base(arm, base);

    /* Step 1: J1 水平旋转角 + 径向/高度
     *
     * FK 中 TCP = 连杆末端 + tcp_offset，因此 IK 需先从 target 中扣除
     * tcp_offset 再分解，保证 IK → FK 闭环：FK(IK(target)) ≈ target。
     *
     * 同时，target 为世界/雷达坐标系坐标，需先减去 world_offset
     * 转换到机械臂内部几何坐标系后再求解：
     *   臂坐标 = 世界坐标 - world_offset
     */
    const float arm_x = target->x - g_dof4_world_offset.x;
    const float arm_y = target->y - g_dof4_world_offset.y;
    const float arm_z = target->z - g_dof4_world_offset.z;
    const float dx = arm_x - base[0] - arm->cfg.tcp_offset[0];
    const float dy = arm_y - base[1] - arm->cfg.tcp_offset[1];
    const float planar_dist = sqrtf(dx * dx + dy * dy);
    const float q1_raw = (planar_dist < DOF4_AXIS_SINGULAR_EPS_M)
                         ? clamp_joint_angle_rad(arm, 0U, arm->joint_actual.q[0])
                         : atan2f(dy, dx);
    /* 将 q1 归化到关节限位内：atan2 返回 [-π,π]，
     * 目标在基座后方时可能需要 ±2π 偏移才能落入 [joint_min, joint_max]。 */
    float q1 = q1_raw;
    if (q1 > arm->cfg.joint_max[0]) {
        q1 -= 2.0f * M_PI_F;
    } else if (q1 < arm->cfg.joint_min[0]) {
        q1 += 2.0f * M_PI_F;
    }
    const float r  = planar_dist - arm->cfg.shoulder_r;   /* 径向（去 shoulder） */
    const float z  = arm_z - base[2] - arm->cfg.shoulder_z - arm->cfg.tcp_offset[2]; /* 高度（去 shoulder + tcp） */

    /* Step 2: 解耦末端工具长度 LT，得到腕部位置 */
    const float phi  = target->pitch - arm->cfg.pitch_offset;  /* 目标俯仰解析角 */
    const float tool = arm->cfg.link_len[2];                    /* LT */
    const float wrist_r = r - tool * cosf(phi);  /* 腕部径向 */
    const float wrist_z = z - tool * sinf(phi);  /* 腕部高度 */

    /* Step 3: 余弦定理求 elbow angle theta3 */
    const float l2 = arm->cfg.link_len[0];       /* L2 上臂 */
    const float l3 = arm->cfg.link_len[1];       /* L3 前臂 */
    const float wrist_d2 = wrist_r * wrist_r + wrist_z * wrist_z;
    const float denom = 2.0f * l2 * l3;
    if (denom < DOF4_GEOM_EPS) {
        return DOF4_STATUS_BAD_CONFIG;
    }

    float cos_q3 = (wrist_d2 - l2 * l2 - l3 * l3) / denom;
    if (cos_q3 > 1.0f + DOF4_COS_EPS || cos_q3 < -1.0f - DOF4_COS_EPS) {
        return DOF4_STATUS_IK_UNREACHABLE;        /* 目标超出机械臂可达范围 */
    }
    cos_q3 = clamp_float(cos_q3, -1.0f, 1.0f);

    const float sin_q3_abs = sqrtf(clamp_float(1.0f - cos_q3 * cos_q3, 0.0f, 1.0f));
    const float theta3 = atan2f(elbow_sign * sin_q3_abs, cos_q3);  /* ±肘型 */

    /* Step 4: shoulder angle theta2（几何关系） */
    const float theta2 = atan2f(wrist_z, wrist_r)
                       - atan2f(l3 * sinf(theta3), l2 + l3 * cosf(theta3));

    /* Step 5: wrist angle theta4（补偿） */
    const float theta4 = phi - theta2 - theta3;

    /* Step 6: 解析角 → URDF 关节角（q = theta，同号不翻转）
     *
     * 不在此处检查关节限位——IK 返回纯数学解，限位由下游
     * latch_joint_target() → clamp_joint_angle_rad() 强制钳位。 */
    joints->q[0] = q1;
    joints->q[1] = theta2;
    joints->q[2] = theta3;
    joints->q[3] = theta4;

    return DOF4_STATUS_OK;
}

/**
 * @brief 根据目标位姿规划或采样单臂笛卡尔轨迹。
 * @param arm 机械臂实例。
 * @param planner 单臂轨迹规划器。
 * @param now_ms 当前时间戳。
 * @param sample 输出采样位姿。
 * @retval Dof4_Status 状态码。
 */

static Dof4_Status sample_arm_target(Dof4_Arm *arm,
                                     Dof4_CartesianPlanner *planner,
                                     uint32_t now_ms,
                                     Dof4_Pose *sample)
{
    if (arm == NULL || planner == NULL || sample == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (!arm->target_valid) {
        *sample = arm->current_pose;
        return DOF4_STATUS_OK;
    }

    if (Dof4_cartesian_target_changed(planner, &arm->target_pose)) {
        Dof4_Pose plan_start = arm->current_pose;

        if (planner->running) {
            Dof4_Pose old_sample;
            Dof4_Status sample_st = Dof4_cartesian_planner_sample(planner, now_ms, &old_sample);
            if (sample_st == DOF4_STATUS_OK) {
                plan_start = old_sample;
            } else if (planner->has_last_sample) {
                plan_start = planner->last_sample_pose;
            }
        } else if (planner->has_last_sample) {
            plan_start = planner->last_sample_pose;
        }

        const float duration = Dof4_cartesian_compute_duration(&plan_start,
                                                               &arm->target_pose,
                                                               arm->cfg.cart_vel_mps,
                                                               arm->cfg.pitch_vel_rps);
        const float *v0 = (planner->running && planner->has_last_vel)
                          ? planner->last_sample_vel : NULL;
        const float *vf = arm->target_is_via ? arm->target_via_vel : NULL;
        Dof4_Status st = Dof4_cartesian_planner_plan(planner,
                                                     &plan_start,
                                                     v0,
                                                     &arm->target_pose,
                                                     vf,
                                                     now_ms,
                                                     duration);
        if (st != DOF4_STATUS_OK) {
            return st;
        }
    }

    return Dof4_cartesian_planner_sample(planner, now_ms, sample);
}

/**
 * @brief 将 IK 输出写入目标关节和舵机目标缓存。
 * @param arm 机械臂实例。
 * @param joints 目标关节角。
 * @retval Dof4_Status 状态码。
 */
static Dof4_Status latch_joint_target(Dof4_Arm *arm,
                                      const Dof4_JointState *joints,
                                      Dof4_ControlMode control_mode)
{
    if (arm == NULL || joints == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

    Dof4_JointState limited_joints = *joints;
    uint8_t clip_reason = DOF4_CLIP_REASON_NONE;
    uint8_t clip_joint_mask = 0U;

    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        if (arm->cfg.servo_sign[i] == 0) {
            return DOF4_STATUS_BAD_CONFIG;
        }

        if (i != 0U) {
            limited_joints.q[i] = clamp_joint_angle_rad(arm, i, joints->q[i]);
            if (limited_joints.q[i] != joints->q[i]) {
                clip_reason |= DOF4_CLIP_REASON_JOINT_LIMIT;
                clip_joint_mask |= (uint8_t)(1U << i);
            }
        }
    }

    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        if (i == 0U) {
            float selected_angle = limited_joints.q[i];
            int16_t servo_pos = arm->target_servo_pos[i];
            Dof4_Status st = j1_angle_to_servo_equivalent(arm,
                                                          limited_joints.q[i],
                                                          &selected_angle,
                                                          &servo_pos);
            limited_joints.q[i] = selected_angle;
            arm->target_servo_pos[i] = servo_pos;
            if (st == DOF4_STATUS_SERVO_LIMIT) {
                (void)Dof4_servo_to_angle(arm, i, servo_pos, &limited_joints.q[i]);
                clip_reason |= DOF4_CLIP_REASON_SERVO_LIMIT;
                clip_joint_mask |= (uint8_t)(1U << i);
            } else if (st != DOF4_STATUS_OK) {
                return st;
            }
            continue;
        }

        const int32_t rounded = joint_angle_to_servo_steps_raw(arm, i, limited_joints.q[i]);
        if (rounded < arm->cfg.servo_min[i] || rounded > arm->cfg.servo_max[i]) {
            arm->target_servo_pos[i] = clamp_i16(rounded,
                                                 arm->cfg.servo_min[i],
                                                 arm->cfg.servo_max[i]);
            (void)Dof4_servo_to_angle(arm,
                                      i,
                                      arm->target_servo_pos[i],
                                      &limited_joints.q[i]);
            clip_reason |= DOF4_CLIP_REASON_SERVO_LIMIT;
            clip_joint_mask |= (uint8_t)(1U << i);
        } else {
            arm->target_servo_pos[i] = (int16_t)rounded;
        }
    }

    arm->joint_target = limited_joints;

    const bool clip_changed = (clip_reason != arm->active_clip_reason) ||
                              (clip_joint_mask != arm->active_clip_joint_mask);
    arm->active_clip_reason = clip_reason;
    arm->active_clip_joint_mask = clip_joint_mask;

    if (clip_reason != DOF4_CLIP_REASON_NONE && clip_changed) {
        Dof4_Arm requested_arm = *arm;
        Dof4_Arm limited_arm = *arm;

        arm->clip_diagnostic.requested_joints = *joints;
        arm->clip_diagnostic.limited_joints = limited_joints;
        (void)Dof4_arm_forward_kinematics(&requested_arm,
                                          joints,
                                          &arm->clip_diagnostic.requested_pose);
        (void)Dof4_arm_forward_kinematics(&limited_arm,
                                          &limited_joints,
                                          &arm->clip_diagnostic.limited_pose);
        for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
            arm->clip_diagnostic.target_servo_pos[i] = arm->target_servo_pos[i];
        }
        arm->clip_diagnostic.control_mode = control_mode;
        arm->clip_diagnostic.reason = clip_reason;
        arm->clip_diagnostic.joint_mask = clip_joint_mask;
        arm->clip_event_counter++;
        arm->clip_diagnostic.event_id = arm->clip_event_counter;
        arm->clip_diagnostic.pending = true;
    }

    return DOF4_STATUS_OK;
}

/**
 * @brief 将角度归一化到 [-pi, pi]。
 * @param angle_rad 输入角度，单位 rad。
 * @retval float 归一化角度。
 */
float Dof4_normalize_angle(float angle_rad)
{
    float a = fmodf(angle_rad, 2.0f * M_PI_F);
    if (a > M_PI_F) {
        a -= 2.0f * M_PI_F;
    } else if (a < -M_PI_F) {
        a += 2.0f * M_PI_F;
    }
    return a;
}

/**
 * @brief 根据 URDF 默认值生成机械臂配置。
 * @param arm_id 机械臂 ID。
 * @retval Dof4_ArmConfig 默认配置。
 */
Dof4_ArmConfig Dof4_arm_default_config(Dof4_ArmId arm_id)
{
    Dof4_ArmConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.arm_id = arm_id;

    // 基于 URDF 参数初始化左右臂配置
    if (arm_id == DOF4_ARM_RIGHT) {
        cfg.base[0] = DOF4_URDF_R_BASE_X;
        cfg.base[1] = DOF4_URDF_R_BASE_Y;
        cfg.base[2] = DOF4_URDF_R_BASE_Z;
        cfg.link_len[2] = DOF4_URDF_R_TOOL_LEN;
        cfg.servo_id[0] = 5U;
        cfg.servo_id[1] = 6U;
        cfg.servo_id[2] = 7U;
        cfg.servo_id[3] = 8U;
        cfg.joint_min[0] = DOF4_R_J1_MIN;
        cfg.joint_max[0] = DOF4_R_J1_MAX;
        cfg.ws_min[0] = -0.30f;     //x
        cfg.ws_max[0] = 0.85f;
        cfg.ws_min[1] = -0.85f;       //y
        cfg.ws_max[1] = 0.6f;
    } else {
        cfg.arm_id = DOF4_ARM_LEFT;
        cfg.base[0] = DOF4_URDF_L_BASE_X;
        cfg.base[1] = DOF4_URDF_L_BASE_Y;
        cfg.base[2] = DOF4_URDF_L_BASE_Z;
        cfg.link_len[2] = DOF4_URDF_L_TOOL_LEN;
        cfg.servo_id[0] = 1U;
        cfg.servo_id[1] = 2U;
        cfg.servo_id[2] = 3U;
        cfg.servo_id[3] = 4U;
        cfg.joint_min[0] = DOF4_L_J1_MIN;
        cfg.joint_max[0] = DOF4_L_J1_MAX;
        cfg.ws_min[0] = -0.30f;
        cfg.ws_max[0] = 0.85f;
        cfg.ws_min[1] = -0.6f;
        cfg.ws_max[1] = 0.85f;
    }

    cfg.shoulder_r = DOF4_URDF_SHOULDER_R;
    cfg.shoulder_z = DOF4_URDF_SHOULDER_Z;
    cfg.link_len[0] = DOF4_URDF_LINK2_LEN;
    cfg.link_len[1] = DOF4_URDF_LINK3_LEN;
    cfg.ws_min[2] = -0.6f;           // TCP Z 方向工作空间下限（相对于基座）
    cfg.ws_max[2] = 0.6f;            // TCP Z 方向工作空间上限（相对于基座）
    cfg.cart_vel_mps = DOF4_DEFAULT_CART_VEL_MPS;         // 笛卡尔空间规划速度，单位 m/s
    cfg.pitch_vel_rps = DOF4_DEFAULT_PITCH_VEL_RPS;         // 俯仰角规划速度，单位 rad/s
    cfg.servo_speed = 1200U;
    cfg.servo_acc = 20U;

    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        cfg.servo_min[i] = DOF4_SERVO_MIN_POS;
        cfg.servo_max[i] = DOF4_SERVO_MAX_POS;
        cfg.servo_reverse[i] = 0U;
    }

    if (cfg.arm_id == DOF4_ARM_RIGHT) {
        cfg.servo_zero[0] = R_J1_ZERO_POS;
        cfg.servo_zero[1] = R_J2_ZERO_POS;
        cfg.servo_zero[2] = R_J3_ZERO_POS;
        cfg.servo_zero[3] = R_J4_ZERO_POS;
        cfg.servo_offset[0] = R_J1_ZERO_BIAS_RAD;
        cfg.servo_offset[1] = R_J2_ZERO_BIAS_RAD;
        cfg.servo_offset[2] = R_J3_ZERO_BIAS_RAD;
        cfg.servo_offset[3] = R_J4_ZERO_BIAS_RAD;
        cfg.servo_sign[0] = R_J1_SERVO_SIGN;
        cfg.servo_sign[1] = R_J2_SERVO_SIGN;
        cfg.servo_sign[2] = R_J3_SERVO_SIGN;
        cfg.servo_sign[3] = R_J4_SERVO_SIGN;
    } else {
        cfg.servo_zero[0] = L_J1_ZERO_POS;
        cfg.servo_zero[1] = L_J2_ZERO_POS;
        cfg.servo_zero[2] = L_J3_ZERO_POS;
        cfg.servo_zero[3] = L_J4_ZERO_POS;
        cfg.servo_offset[0] = L_J1_ZERO_BIAS_RAD;
        cfg.servo_offset[1] = L_J2_ZERO_BIAS_RAD;
        cfg.servo_offset[2] = L_J3_ZERO_BIAS_RAD;
        cfg.servo_offset[3] = L_J4_ZERO_BIAS_RAD;
        cfg.servo_sign[0] = L_J1_SERVO_SIGN;
        cfg.servo_sign[1] = L_J2_SERVO_SIGN;
        cfg.servo_sign[2] = L_J3_SERVO_SIGN;
        cfg.servo_sign[3] = L_J4_SERVO_SIGN;
    }

    /*
     * J2/J4 原配置描述的是舵机相对中位的机械行程。运行时统一保存为
     * URDF 绝对关节角，避免锁存路径和 Dof4_angle_to_servo() 语义不一致。
     */
    cfg.joint_min[1] = cfg.servo_offset[1] + DOF4_J2_SERVO_REL_MIN;
    cfg.joint_max[1] = cfg.servo_offset[1] + DOF4_J2_SERVO_REL_MAX;
    cfg.joint_min[2] = cfg.servo_offset[2] + DOF4_J3_SERVO_REL_MIN;
    cfg.joint_max[2] = DOF4_J3_ABS_MAX;
    cfg.joint_min[3] = cfg.servo_offset[3] + DOF4_J4_SERVO_REL_MIN;
    cfg.joint_max[3] = cfg.servo_offset[3] + DOF4_J4_SERVO_REL_MAX;
    return cfg;

}
 
/**
 * @brief 初始化单臂运行实例。
 * @param arm 机械臂实例。
 * @param config 配置对象。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_config_init(Dof4_Arm *arm, const Dof4_ArmConfig *config)
{
    if (arm == NULL || config == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (config->link_len[0] < DOF4_GEOM_EPS ||
        config->link_len[1] < DOF4_GEOM_EPS ||
        config->link_len[2] < 0.0f) {
        return DOF4_STATUS_BAD_CONFIG;
    }
    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        if (config->servo_sign[i] == 0 ||
            config->servo_min[i] > config->servo_max[i] ||
            config->joint_min[i] > config->joint_max[i]) {
            return DOF4_STATUS_BAD_CONFIG;
        }
    }

    memset(arm, 0, sizeof(*arm));
    arm->cfg = *config;
    arm->state = DOF4_ARM_STATE_INIT;
    arm->last_status = DOF4_STATUS_OK;

    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        arm->servo_pos[i] = arm->cfg.servo_zero[i];
        arm->target_servo_pos[i] = arm->cfg.servo_zero[i];
        (void)Dof4_servo_to_angle(arm,
                                  i,
                                  arm->servo_pos[i],
                                  &arm->joint_actual.q[i]);
        arm->joint_target.q[i] = arm->joint_actual.q[i];
    }



    (void)Dof4_arm_forward_kinematics(arm, &arm->joint_actual, &arm->current_pose);
    arm->target_pose = arm->current_pose;
    arm->target_valid = false;
    arm->control_mode = DOF4_CONTROL_MODE_POSE;
    arm->state = DOF4_ARM_STATE_IDLE;
    return DOF4_STATUS_OK;
}

/**
 * @brief 初始化双臂实例和舵机总线基础设置。
 * @param arm_left 左臂实例。
 * @param arm_right 右臂实例。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_dual_arm_init(Dof4_Arm *arm_left, Dof4_Arm *arm_right)
{
    if (arm_left == NULL || arm_right == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

#ifndef DOF4_HOST_TEST
    SCS_SetUART(&huart1);
    SCS_SetHalfDuplex(0);
    setEnd(0);
#endif

    Dof4_ArmConfig left_cfg  = Dof4_arm_default_config(DOF4_ARM_LEFT);
    Dof4_ArmConfig right_cfg = Dof4_arm_default_config(DOF4_ARM_RIGHT);
    Dof4_Status st = Dof4_arm_config_init(arm_left, &left_cfg);
    if (st != DOF4_STATUS_OK) {
        return st;
    }
    st = Dof4_arm_config_init(arm_right, &right_cfg);
    if (st != DOF4_STATUS_OK) {
        return st;
    }

    (void)Dof4_cartesian_planner_init(&g_planner_left);
    (void)Dof4_cartesian_planner_init(&g_planner_right);
    return DOF4_STATUS_OK;
}


/**
 * @brief 设置基座附加偏移。
 * @param arm 机械臂实例。
 * @param dx X 偏移，单位 m。
 * @param dy Y 偏移，单位 m。
 * @param dz Z 偏移，单位 m。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_base_offset(Dof4_Arm *arm, float dx, float dy, float dz)
{
    if (arm == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    arm->cfg.base_offset[0] = dx;
    arm->cfg.base_offset[1] = dy;
    arm->cfg.base_offset[2] = dz;
    return Dof4_arm_forward_kinematics(arm, &arm->joint_actual, &arm->current_pose);
}


/**
 * @brief 设置 TCP 附加偏移。
 * @param arm 机械臂实例。
 * @param dx X 偏移，单位 m。
 * @param dy Y 偏移，单位 m。
 * @param dz Z 偏移，单位 m。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_tcp_offset(Dof4_Arm *arm, float dx, float dy, float dz)
{
    if (arm == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    arm->cfg.tcp_offset[0] = dx;
    arm->cfg.tcp_offset[1] = dy;
    arm->cfg.tcp_offset[2] = dz;
    return DOF4_STATUS_OK;
}

/**
 * @brief 设置末端 pitch 零位偏置。
 * @param arm 机械臂实例。
 * @param pitch_offset pitch 偏置，单位 rad。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_pitch_offset(Dof4_Arm *arm, float pitch_offset)
{
    if (arm == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    arm->cfg.pitch_offset = pitch_offset;
    return DOF4_STATUS_OK;
}

/**
 * @brief 设置单个关节的舵机零位和角度偏置。
 *
 * @details
 * 换算公式：
 *   joint_angle_rad = (servo_pos - zero_pos) / POS_PER_RAD / servo_sign + offset_rad
 *
 * 即：将舵机步进换算为关节角后，再加上一个固定的角度偏置（安装/标定误差）。
 * **参数 `offset_rad` 必须为弧度制，调用方若持有度值需自行转换。**
 *
 * @param arm         机械臂实例。
 * @param joint_index 关节索引 0~3。
 * @param zero_pos    舵机零位步进值（通常 = 2048）。
 * @param offset_rad  角度偏置，单位 **弧度**（非度！）。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_servo_offset(Dof4_Arm *arm,
                                      uint8_t joint_index,
                                      int16_t zero_pos,
                                      float offset_rad)
{
    if (arm == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (joint_index >= DOF4_JOINT_COUNT) {
        return DOF4_STATUS_BAD_CONFIG;
    }
    arm->cfg.servo_zero[joint_index] = zero_pos;
    arm->cfg.servo_offset[joint_index] = offset_rad;   /* 直接存储弧度值，不做单位转换 */
    return DOF4_STATUS_OK;
}

/**
 * @brief 执行几何正运动学并更新关节世界坐标（对标 base_link 世界坐标系）。
 *
 * @details
 * ## 坐标系定义
 *
 * - `{B}` = URDF 的 `base_link` 世界坐标系：
 *   X 向前，Y 向左，Z 向上（右手系）。
 * - `{P}` = 由 J1 `RotZ(q1)` 确定的**垂直工作平面**：
 *   `X_P` 径向向外（水平），`Y_P` 竖直向上（= `{B}` 的 +Z）。
 *
 * ## 简化几何模型（运行时使用，与 IK 共用同一套参数）
 *
 * 将 URDF 中的空间偏置（shoulder_r, shoulder_z）和连杆长度（L2, L3, LT）
 * 近似为平面 3R 链，避免 4×4 矩阵乘法，适合 F407 200 Hz 实时控制：
 *
 * ```
 * 解析角：theta2 = q2, theta3 = q3, theta4 = q4
 *
 * r_J2   = shoulder_r
 * z_J2   = shoulder_z
 * r_J3   = r_J2  + L2 * cos(theta2)          ← 肘部径向
 * z_J3   = z_J2  + L2 * sin(theta2)          ← 肘部高度
 * r_J4   = r_J3  + L3 * cos(theta2+theta3)   ← 腕部径向
 * z_J4   = z_J3  + L3 * sin(theta2+theta3)   ← 腕部高度
 * r_TCP  = r_J4  + LT * cos(theta2+theta3+theta4)
 * z_TCP  = z_J4  + LT * sin(theta2+theta3+theta4)
 *
 * x_TCP  = base_x + cos(q1) * r_TCP + tcp_offset_x   ← 投影回 {B}
 * y_TCP  = base_y + sin(q1) * r_TCP + tcp_offset_y
 * z_TCP  = base_z +           z_TCP + tcp_offset_z
 * pitch  = (theta2+theta3+theta4) + pitch_offset
 *        = -(q2+q3+q4) + pitch_offset
 * ```
 *
 * @note  下方 `#if 0` 块保留了完整的 URDF 4×4 矩阵链 FK（含左右臂 pitch
 *        分支修正），用于离线验证简化模型的近似精度。F407 实时控制时不使用。
 *
 * @param arm    机械臂实例。
 * @param joints 输入关节角 q[0..3]（对标 URDF joint angle），单位 rad。
 * @param pose   输出 TCP 位姿，世界坐标系，单位 m / rad。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_forward_kinematics(Dof4_Arm *arm,
                                        const Dof4_JointState *joints,
                                        Dof4_Pose *pose)
{
    if (arm == NULL || joints == NULL || pose == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

    /* ════════════════════════════════════════════════════════════
     * 简化几何 FK（运行时路径）
     *
     * 将 J2/J3/J4 近似为同一垂直平面内的 3R 链。
     * 与 IK（solve_ik_candidate）使用同一套 shoulder_r / shoulder_z /
     * link_len[0..2] 参数，保证 IK → FK 闭环一致性。
     * ════════════════════════════════════════════════════════════ */
    {
        float base[3];
        get_effective_base(arm, base);  /* base = cfg.base + cfg.base_offset */

        /* ── 关节角 → 平面解析角 ──
         * theta = q（解析角 = 关节角），不再取反。
         * 实物臂向上抬 → q 变化 → sin(theta) 符号正确 → z 同步变化。
         * J1 绕 +Z 旋转，q1 正方向与 URDF 一致，无需处理。
         */
        const float q1     = joints->q[0];          /* J1 直接对应 +Z 旋转 */
        const float theta2 = joints->q[1];          /* J2 俯仰解析角（= q2） */
        const float theta3 = joints->q[2];          /* J3 肘俯仰解析角（= q3） */
        const float theta4 = joints->q[3];          /* J4 腕俯仰解析角（= q4） */
        const float c1 = cosf(q1);
        const float s1 = sinf(q1);

        /* ── 在 {P} 平面内累加径向 r 和高度 z ── */
        float r = 0.0f;
        float z = 0.0f;

        /* J1 基座旋转中心（世界坐标） */
        arm->joint_world[0][0] = base[0];
        arm->joint_world[0][1] = base[1];
        arm->joint_world[0][2] = base[2];

        /* J2 肩关节：J1 原点 + shoulder 水平/垂直偏置 */
        r += arm->cfg.shoulder_r;       /* 水平偏置 ≈ 0.02716 m */
        z += arm->cfg.shoulder_z;       /* 垂直偏置 ≈ 0.02232 m */
        arm->joint_world[1][0] = base[0] + c1 * r;
        arm->joint_world[1][1] = base[1] + s1 * r;
        arm->joint_world[1][2] = base[2] + z;

        /* J3 肘关节：沿上臂 L2 延伸（≈ 0.35548 m） */
        r += arm->cfg.link_len[0] * cosf(theta2);
        z += arm->cfg.link_len[0] * sinf(theta2);
        arm->joint_world[2][0] = base[0] + c1 * r;
        arm->joint_world[2][1] = base[1] + s1 * r;
        arm->joint_world[2][2] = base[2] + z;

        /* J4 腕关节：沿前臂 L3 延伸（≈ 0.27130 m） */
        const float theta23 = theta2 + theta3;
        r += arm->cfg.link_len[1] * cosf(theta23);
        z += arm->cfg.link_len[1] * sinf(theta23);
        arm->joint_world[3][0] = base[0] + c1 * r;
        arm->joint_world[3][1] = base[1] + s1 * r;
        arm->joint_world[3][2] = base[2] + z;

        /* TCP 末端：沿工具 LT 延伸（左 ≈ 0.03600 m，右 ≈ 0.03620 m） */
        const float theta234 = theta23 + theta4;
        r += arm->cfg.link_len[2] * cosf(theta234);
        z += arm->cfg.link_len[2] * sinf(theta234);

        /* ── 投影回 {B} 世界坐标系 ── */
        pose->x = base[0] + c1 * r + arm->cfg.tcp_offset[0];
        pose->y = base[1] + s1 * r + arm->cfg.tcp_offset[1];
        pose->z = base[2] +      z + arm->cfg.tcp_offset[2];
        /*
         * 叠加全局世界坐标系偏移，将臂内几何坐标映射到雷达/世界坐标系：
         *   世界坐标 = 机械臂几何坐标 + world_offset
         */
        pose->x += g_dof4_world_offset.x;
        pose->y += g_dof4_world_offset.y;
        pose->z += g_dof4_world_offset.z;
        /*
         * pitch = 三个俯仰解析角之和 + 标定偏置
         *       = theta2 + theta3 + theta4 + pitch_offset
         *       = (q2 + q3 + q4) + pitch_offset
         *
         * 简化模型中左右臂使用同一公式（平面近似下俯仰均为解析角和）。
         * 完整 URDF 模型中右臂需修正（见下方 #if 0 参考实现）。
         */
        pose->pitch = theta234 + arm->cfg.pitch_offset;

        /* 记录 TCP 世界坐标（用于碰撞检测胶囊体端点） */
        arm->joint_world[4][0] = pose->x;
        arm->joint_world[4][1] = pose->y;
        arm->joint_world[4][2] = pose->z;
        arm->current_pose = *pose;
        return DOF4_STATUS_OK;
    }
}

/**
 * @brief 执行几何解析逆运动学。
 *
 * 调用方通过 @p elbow_sign 显式指定肘型：
 * - `+1.0f` = 肘上解（elbow up）
 * - `-1.0f` = 肘下解（elbow down）
 *
 * @param arm        机械臂实例。
 * @param target     目标位姿。
 * @param elbow_sign 肘型符号，+1=肘上，-1=肘下。
 * @param joints     输出关节角。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_inverse_kinematics(Dof4_Arm *arm,
                                        const Dof4_Pose *target,
                                        float elbow_sign,
                                        Dof4_JointState *joints)
{
    if (arm == NULL || target == NULL || joints == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

    /* workspace 边界定义在臂坐标系中，需将世界坐标转为臂坐标后检查 */
    Dof4_Pose arm_target = {target->x - g_dof4_world_offset.x,
                            target->y - g_dof4_world_offset.y,
                            target->z - g_dof4_world_offset.z,
                            target->pitch};
    Dof4_Status st = check_workspace(arm, &arm_target);
    if (st != DOF4_STATUS_OK) {
        return st;
    }

    return solve_ik_candidate(arm, target, elbow_sign, joints);
}

/**
 * @brief 将舵机步进值转换为真实反馈关节角，不做 URDF 关节限位钳制。
 * @param arm 机械臂实例。
 * @param joint_index 关节索引。
 * @param servo_pos 舵机步进值。
 * @param angle_rad 输出关节角，单位 rad。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_servo_to_angle(const Dof4_Arm *arm,
                                uint8_t joint_index,
                                int16_t servo_pos,
                                float *angle_rad)
{
    if (arm == NULL || angle_rad == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (joint_index >= DOF4_JOINT_COUNT) {
        return DOF4_STATUS_BAD_CONFIG;
    }

    if (arm->cfg.servo_sign[joint_index] == 0) {
        return DOF4_STATUS_BAD_CONFIG;
    }

    const int16_t limited_pos = clamp_i16((int32_t)servo_pos,
                                          arm->cfg.servo_min[joint_index],
                                          arm->cfg.servo_max[joint_index]);
    float servo_angle = ((float)(limited_pos - arm->cfg.servo_zero[joint_index]) /
                         DOF4_SERVO_POS_PER_RAD);
    if (arm->cfg.servo_reverse[joint_index] != 0U) {
        servo_angle = -servo_angle;
    }

    *angle_rad = (servo_angle / (float)arm->cfg.servo_sign[joint_index]) +
                 arm->cfg.servo_offset[joint_index];
    return DOF4_STATUS_OK;
}

/**
 * @brief 将目标关节角限制到 URDF 安全范围后转换为舵机步进值。
 * @param arm 机械臂实例。
 * @param joint_index 关节索引。
 * @param angle_rad 关节角，单位 rad。
 * @param servo_pos 输出舵机步进值。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_angle_to_servo(const Dof4_Arm *arm,
                                uint8_t joint_index,
                                float angle_rad,
                                int16_t *servo_pos)
{
    if (arm == NULL || servo_pos == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (joint_index >= DOF4_JOINT_COUNT) {
        return DOF4_STATUS_BAD_CONFIG;
    }

    if (arm->cfg.servo_sign[joint_index] == 0) {
        return DOF4_STATUS_BAD_CONFIG;
    }

    if (joint_index == 0U) {
        float selected_angle = angle_rad;
        return j1_angle_to_servo_equivalent(arm,
                                            angle_rad,
                                            &selected_angle,
                                            servo_pos);
    }

    const float limited_angle = clamp_joint_angle_rad(arm, joint_index, angle_rad);
    float servo_angle = (limited_angle - arm->cfg.servo_offset[joint_index]) *
                        (float)arm->cfg.servo_sign[joint_index];
    if (arm->cfg.servo_reverse[joint_index] != 0U) {
        servo_angle = -servo_angle;
    }

    const float steps_f = (float)arm->cfg.servo_zero[joint_index] +
                          servo_angle * DOF4_SERVO_POS_PER_RAD;
    const int32_t rounded = (int32_t)((steps_f >= 0.0f) ? (steps_f + 0.5f) : (steps_f - 0.5f));
    if (rounded < arm->cfg.servo_min[joint_index] ||
        rounded > arm->cfg.servo_max[joint_index]) {
        *servo_pos = clamp_i16(rounded,
                               arm->cfg.servo_min[joint_index],
                               arm->cfg.servo_max[joint_index]);
        return DOF4_STATUS_SERVO_LIMIT;
    }

    *servo_pos = (int16_t)rounded;
    return DOF4_STATUS_OK;
}

/**
 * @brief 设置用户目标位姿。
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
                                float target_pitch)
{
    if (arm == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

    /* workspace 边界定义在臂坐标系中，需将世界坐标转为臂坐标后检查 */
    Dof4_Pose pose = {target_x, target_y, target_z, target_pitch};
    Dof4_Pose arm_pose = {target_x - g_dof4_world_offset.x,
                          target_y - g_dof4_world_offset.y,
                          target_z - g_dof4_world_offset.z,
                          target_pitch};
    Dof4_Status st = check_workspace(arm, &arm_pose);
    if (st != DOF4_STATUS_OK) {
        arm->last_status = st;
        return st;
    }

    arm->target_pose = pose;
    arm->target_valid = true;
    arm->target_is_via = false;
    arm->control_mode = DOF4_CONTROL_MODE_POSE;
    arm->last_status = DOF4_STATUS_OK;
    return DOF4_STATUS_OK;
}

/**
 * @brief 设置单臂途经点目标位姿（非零终端速度，平滑通过不停止）。
 * @param arm 机械臂实例。
 * @param target_x 目标 X，单位 m。
 * @param target_y 目标 Y，单位 m。
 * @param target_z 目标 Z，单位 m。
 * @param target_pitch 目标 pitch，单位 rad。
 * @param via_vel 通过速度 (x,y,z,pitch)，NULL=退化为零速停止。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_target_via(Dof4_Arm *arm,
                                    float target_x,
                                    float target_y,
                                    float target_z,
                                    float target_pitch,
                                    const float via_vel[4])
{
    if (arm == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

    /* workspace 检查（与 set_target 相同） */
    Dof4_Pose arm_pose = {target_x - g_dof4_world_offset.x,
                          target_y - g_dof4_world_offset.y,
                          target_z - g_dof4_world_offset.z,
                          target_pitch};
    Dof4_Status st = check_workspace(arm, &arm_pose);
    if (st != DOF4_STATUS_OK) {
        arm->last_status = st;
        return st;
    }

    arm->target_pose.x = target_x;
    arm->target_pose.y = target_y;
    arm->target_pose.z = target_z;
    arm->target_pose.pitch = target_pitch;
    arm->target_valid = true;
    arm->target_is_via = true;
    arm->control_mode = DOF4_CONTROL_MODE_POSE;

    if (via_vel != NULL) {
        arm->target_via_vel[0] = via_vel[0];
        arm->target_via_vel[1] = via_vel[1];
        arm->target_via_vel[2] = via_vel[2];
        arm->target_via_vel[3] = via_vel[3];
    } else {
        arm->target_via_vel[0] = 0.0f;
        arm->target_via_vel[1] = 0.0f;
        arm->target_via_vel[2] = 0.0f;
        arm->target_via_vel[3] = 0.0f;
    }

    arm->last_status = DOF4_STATUS_OK;
    return DOF4_STATUS_OK;
}

/**
 * @brief 设置单臂关节目标并切换到关节控制模式。
 * @param arm 机械臂实例。
 * @param joints 目标关节角，单位 rad。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_set_joint_target(Dof4_Arm *arm,
                                      const Dof4_JointState *joints)
{
    Dof4_Status st = latch_joint_target(arm, joints, DOF4_CONTROL_MODE_JOINT);
    if (st != DOF4_STATUS_OK) {
        if (arm != NULL) {
            arm->last_status = st;
        }
        return st;
    }

    arm->target_valid = false;
    arm->control_mode = DOF4_CONTROL_MODE_JOINT;
    arm->last_status = DOF4_STATUS_OK;
    return DOF4_STATUS_OK;
}

/**
 * @brief 读取单臂舵机反馈并更新 FK。
 * @param arm 机械臂实例。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_read_servo_pos(Dof4_Arm *arm)
{
    if (arm == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

#ifndef DOF4_HOST_TEST
    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        const int pos = ReadPos((int)arm->cfg.servo_id[i]);
        if (pos < 0) {
            if (arm->comm_fail_count < 255U) {
                arm->comm_fail_count++;
            }
            if (arm->comm_fail_count >= DOF4_COMM_FAIL_THRESHOLD) {
                arm->state = DOF4_ARM_STATE_ERROR;
            }
            return DOF4_STATUS_COMM_FAIL;
        }
        arm->servo_pos[i] = (int16_t)pos;
    }
#endif

    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        (void)Dof4_servo_to_angle(arm, i, arm->servo_pos[i], &arm->joint_actual.q[i]);
    }
    arm->comm_fail_count = 0U;
    return Dof4_arm_forward_kinematics(arm, &arm->joint_actual, &arm->current_pose);
}

/**
 * @brief 读取左右双臂舵机反馈。
 *
 * @note 当前左臂舵机未接入硬件，跳过其读取以避免通信失败阻塞右臂。
 *       左臂 joint_actual / current_pose 保持初始化零位不变。
 *       恢复左臂硬件后，取消注释左臂读取行即可。
 *
 * @param arm_left  左臂实例。
 * @param arm_right 右臂实例。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_batch_read_all_servo(Dof4_Arm *arm_left, Dof4_Arm *arm_right)
{
    /* 读取左臂舵机反馈 */
    Dof4_Status st = Dof4_arm_read_servo_pos(arm_left);
    if (st != DOF4_STATUS_OK) {
        return st;
    }

    /* 读取右臂舵机反馈 */
    return Dof4_arm_read_servo_pos(arm_right);
}

/**
 * @brief 写入左右双臂舵机目标。
 *
 * @param arm_left  左臂实例。
 * @param arm_right 右臂实例。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_batch_write_all_servo(const Dof4_Arm *arm_left,
                                       const Dof4_Arm *arm_right)
{
    if (arm_left == NULL || arm_right == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

#ifndef DOF4_HOST_TEST
    /* 双臂 8 路舵机：左臂 ID = 1,2,3,4；右臂 ID = 5,6,7,8 */
    #define DOF4_BATCH_SERVO_COUNT (DOF4_JOINT_COUNT * 2U)
    uint8_t  id[DOF4_BATCH_SERVO_COUNT];
    int16_t  pos[DOF4_BATCH_SERVO_COUNT];
    uint16_t spd[DOF4_BATCH_SERVO_COUNT];
    uint8_t  acc[DOF4_BATCH_SERVO_COUNT];

    /* 左臂 4 路 */
    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        id[i]  = arm_left->cfg.servo_id[i];
        pos[i] = arm_left->target_servo_pos[i];
        spd[i] = arm_left->cfg.servo_speed;
        acc[i] = arm_left->cfg.servo_acc;
    }

    /* 右臂 4 路 */
    for (uint8_t i = 0; i < DOF4_JOINT_COUNT; ++i) {
        id[DOF4_JOINT_COUNT + i]  = arm_right->cfg.servo_id[i];
        pos[DOF4_JOINT_COUNT + i] = arm_right->target_servo_pos[i];
        spd[DOF4_JOINT_COUNT + i] = arm_right->cfg.servo_speed;
        acc[DOF4_JOINT_COUNT + i] = arm_right->cfg.servo_acc;
    }

    SyncWritePosEx(id, DOF4_BATCH_SERVO_COUNT, pos, spd, acc);
#endif 
    return DOF4_STATUS_OK;
}

/* ════════════════════════════════════════════════════════════════
 * 碰撞规避辅助函数 — 前向声明（定义见文件末尾）
 * ════════════════════════════════════════════════════════════════ */

static void dof4_compute_retreat_pose(const Dof4_CollisionDetail *detail,
                                      const Dof4_Pose *right_current,
                                      Dof4_Pose *retreat);
static bool dof4_is_pose_reached(const Dof4_Arm *arm,
                                 const Dof4_Pose *target,
                                 float tol_m);

/**
 * @brief 双臂一步式控制循环——每帧执行一次完整的感知→决策→执行流水线。
 *
 * @details
 * ## 调用约定
 * 由外部 RTOS 任务以固定频率（200 Hz）周期性调用。每次调用完成一次
 * "读反馈 → 轨迹采样 → IK → 碰撞预检 → 锁存目标 → 舵机下发 → 状态更新"
 * 的完整闭环。
 *
 * ## 执行顺序（按数据流）
 *
 * ```
 * ① batch_read_all_servo   ← 读舵机实际位置 → FK 更新 current_pose
 *          ↓
 * ② sample_arm_target      ← 用户 target → 梯形速度轨迹 → 输出本帧采样点
 *          ↓
 * ③ inverse_kinematics     ← 笛卡尔采样点 → 几何解析 → 关节角 q[0..3]
 *          ↓
 * ④ collision_check        ← 用 IK 结果跑 FK 副本 → 胶囊体最短距离 → 碰撞?
 *          ↓ (安全)
 * ⑤ latch_joint_target     ← 关节角限位 → angle_to_servo → target_servo_pos[]
 *          ↓
 * ⑥ batch_write_all_servo  ← SyncWritePosEx 一次帧下发双臂 8 路舵机
 *          ↓
 * ⑦ state = IDLE / MOVING  ← 更新臂状态供上层监控
 * ```
 *
 * ## 错误处理策略
 *
 * - ① 通信失败 → 立即返回，不执行任何控制（保持原位安全）
 * - ②/③/⑤ 失败 → 记录 `last_status` 后返回，同一帧后续步骤不执行
 * - ④ 碰撞风险 → 返回 `DOF4_STATUS_COLLISION_RISK`，不执行 latch/write
 * - ⑥ 当前被注释用于离线调试，解除注释后恢复完整闭环
 *
 * ## 数据流关键变量
 *
 * | 步骤 | 输入 | 输出 |
 * |------|------|------|
 * | ① | 舵机总线 | `joint_actual.q[]`, `current_pose` |
 * | ② | `target_pose`(用户设), `current_pose` | `sample_left/right` (笛卡尔) |
 * | ③ | `sample_left/right` | `joints_left/right` (关节角) |
 * | ④ | `joints_left/right` + 臂副本 | 通过/拦截 |
 * | ⑤ | `joints_left/right` | `joint_target`, `target_servo_pos[]` |
 * | ⑥ | `target_servo_pos[]` | 舵机硬件 |
 *
 * @param arm_left  左臂实例。
 * @param arm_right 右臂实例。
 * @param now_ms    当前系统时间戳（`HAL_GetTick()`），用于轨迹规划计时。
 * @retval Dof4_Status 本帧执行结果，`DOF4_STATUS_OK` 表示完整闭环成功。
 */
Dof4_Status Dof4_dual_arm_control_loop(Dof4_Arm *arm_left,
                                       Dof4_Arm *arm_right,
                                       uint32_t now_ms)
{
    if (arm_left == NULL || arm_right == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

    /* ─── ① 读反馈 + FK ───────────────────────────────────────────
     * 逐个读取双臂 8 路舵机位置 → servo_to_angle → FK 更新 current_pose。
     * 若任一舵机通信失败，整帧放弃——不基于过时数据做控制决策。
     */
    Dof4_Status st = Dof4_batch_read_all_servo(arm_left, arm_right);
    if (st != DOF4_STATUS_OK) {
        return st;
    }

    /* ─── ② 轨迹采样 ───────────────────────────────────────────────
     * 将用户通过 Dof4_arm_set_target() 设置的目标位姿，经梯形速度规划后
     * 按当前时间戳采样，得到本帧应到达的笛卡尔位姿。
     *
     * - 若 target 刚变化 → 启动新轨迹段 (cart_vel_mps / pitch_vel_rps)
     * - 若轨迹未完成     → 沿已有轨迹按时间插值
     * - 若从未设目标     → 返回 current_pose（停在原地）
     */
    const bool left_pose_mode = (arm_left->control_mode != DOF4_CONTROL_MODE_JOINT);
    const bool right_pose_mode = (arm_right->control_mode != DOF4_CONTROL_MODE_JOINT);

    Dof4_Pose sample_left;
    Dof4_Pose sample_right;
    if (left_pose_mode) {
        st = sample_arm_target(arm_left, &g_planner_left, now_ms, &sample_left);
        if (st != DOF4_STATUS_OK) {
            arm_left->last_status = st;
            return st;
        }
    } else {
        (void)Dof4_cartesian_planner_init(&g_planner_left);
        arm_left->target_valid = false;
    }
    if (right_pose_mode) {
        st = sample_arm_target(arm_right, &g_planner_right, now_ms, &sample_right);
        if (st != DOF4_STATUS_OK) {
            arm_right->last_status = st;
            return st;
        }
    } else {
        (void)Dof4_cartesian_planner_init(&g_planner_right);
        arm_right->target_valid = false;
    }

    /* ─── ③ 逆运动学 ───────────────────────────────────────────────
     * 笛卡尔采样点 (x,y,z,pitch) → 平面 3R 几何解析 → 关节角 (q1,q2,q3,q4)。
     * 左臂肘型 +1.0f（肘上解），右臂肘型 -1.0f（肘下解），避免镜像双臂肘部碰撞。
     *
     * ⚠ IK 失败时不退出循环：invalidate target → 下帧从 current_pose 重规划，
     *   臂保持原位不动，避免因单帧不可达而永久卡死。
     */
    Dof4_JointState joints_left;
    Dof4_JointState joints_right;
    if (left_pose_mode) {
        st = Dof4_arm_inverse_kinematics(arm_left, &sample_left, -1.0f, &joints_left);
        if (st != DOF4_STATUS_OK) {
            /* IK 失败 → 软着陆：不移动臂，迫使下帧从当前位姿重新规划轨迹 */
            arm_left->last_status = st;
            arm_left->target_valid = false;
            arm_left->state = DOF4_ARM_STATE_IDLE;
            arm_right->last_status = DOF4_STATUS_OK;
            return DOF4_STATUS_OK;
        }
    }
    if (right_pose_mode) {
        st = Dof4_arm_inverse_kinematics(arm_right, &sample_right, -1.0f, &joints_right);
        if (st != DOF4_STATUS_OK) {
            /* IK 失败 → 软着陆：不移动臂，迫使下帧从当前位姿重新规划轨迹 */
            arm_right->last_status = st;
            arm_right->target_valid = false;
            arm_right->state = DOF4_ARM_STATE_IDLE;
            arm_left->last_status  = DOF4_STATUS_OK;
            return DOF4_STATUS_OK;
        }
    }

    /* ─── ④ 碰撞预检测 + 规避状态机 ───────────────────────────────
     * 用 IK 输出关节角做预测 FK → 胶囊体碰撞检查。
     * 若碰撞且状态为 NORMAL → 保存目标，计算右臂撤退位姿，进入 RIGHT_RETREAT。
     * 若已处于规避状态 → 按阶段推进。
     * 安全时 → 正常 latch → write → 状态更新。
     */
    // {
    //     Dof4_Arm predicted_left  = *arm_left;   /* 浅拷贝 */
    //     Dof4_Arm predicted_right = *arm_right;
    //     Dof4_Pose tmp_pose;
    //     (void)Dof4_arm_forward_kinematics(&predicted_left,  &joints_left,  &tmp_pose);
    //     (void)Dof4_arm_forward_kinematics(&predicted_right, &joints_right, &tmp_pose);

    //     Dof4_CollisionDetail detail;
    //     memset(&detail, 0, sizeof(detail));
    //     st = Dof4_collision_check_capsule(&predicted_left, &predicted_right, &detail);

    //     if (st == DOF4_STATUS_COLLISION_RISK && !detail.is_safe) {
    //         /* ── 检测到碰撞风险 ── */
    //         if (s_avoid_state == DOF4_AVOID_NORMAL) {
    //             /* 首次碰撞：保存双方原始目标，计算右臂安全位 */
    //             s_saved_left_target  = arm_left->target_pose;
    //             s_saved_right_target = arm_right->target_pose;
    //             dof4_compute_retreat_pose(&detail,
    //                                       &arm_right->current_pose,
    //                                       &s_retreat_pose_right);
    //             (void)Dof4_clamp_to_workspace(arm_right, &s_retreat_pose_right);
    //             s_avoid_state = DOF4_AVOID_RIGHT_RETREAT;
    //             s_avoid_enter_tick = now_ms;
    //         }
    //     }

    //     /* ── 根据规避状态覆写双臂目标 ── */
    //     bool avoid_stall_frame = false;  /* 本帧是否需要跳过 latch/write */
    //     switch (s_avoid_state) {
    //     case DOF4_AVOID_NORMAL:
    //         break;  /* 正常流程，不做覆写 */

    //     case DOF4_AVOID_RIGHT_RETREAT:
    //         /* 左臂停住，右臂撤到安全位 */
    //         arm_left->target_valid  = false;
    //         arm_right->target_pose  = s_retreat_pose_right;
    //         arm_right->target_valid = true;
    //         if (dof4_is_pose_reached(arm_right, &s_retreat_pose_right, DOF4_AVOID_REACH_TOL_M) ||
    //             ((now_ms - s_avoid_enter_tick) > DOF4_AVOID_TIMEOUT_MS)) {
    //             s_avoid_state = DOF4_AVOID_LEFT_ADVANCE;
    //             s_avoid_enter_tick = now_ms;
    //         }
    //         avoid_stall_frame = true;
    //         break;

    //     case DOF4_AVOID_LEFT_ADVANCE:
    //         /* 左臂前进，右臂停住 */
    //         arm_left->target_pose  = s_saved_left_target;
    //         arm_left->target_valid = true;
    //         arm_right->target_valid = false;
    //         if (dof4_is_pose_reached(arm_left, &s_saved_left_target, DOF4_AVOID_REACH_TOL_M) ||
    //             ((now_ms - s_avoid_enter_tick) > DOF4_AVOID_TIMEOUT_MS)) {
    //             s_avoid_state = DOF4_AVOID_RIGHT_ADVANCE;
    //             s_avoid_enter_tick = now_ms;
    //         }
    //         avoid_stall_frame = true;
    //         break;

    //     case DOF4_AVOID_RIGHT_ADVANCE:
    //         /* 右臂前进到原始目标，左臂停住 */
    //         arm_left->target_valid  = false;
    //         arm_right->target_pose  = s_saved_right_target;
    //         arm_right->target_valid = true;
    //         if (dof4_is_pose_reached(arm_right, &s_saved_right_target, DOF4_AVOID_REACH_TOL_M) ||
    //             ((now_ms - s_avoid_enter_tick) > DOF4_AVOID_TIMEOUT_MS)) {
    //             s_avoid_state = DOF4_AVOID_NORMAL;
    //         }
    //         avoid_stall_frame = true;
    //         break;
    //     }

    //     if (avoid_stall_frame) {
    //         /* 规避激活中：本帧重新采样轨迹但不 latch/write，等下一帧 */
    //         arm_left->state  = DOF4_ARM_STATE_MOVING;
    //         arm_right->state = DOF4_ARM_STATE_MOVING;
    //         arm_left->last_status  = DOF4_STATUS_OK;
    //         arm_right->last_status = DOF4_STATUS_OK;
    //         return DOF4_STATUS_OK;
    //     }
    // }

    /* ─── ⑤ 锁存关节目标 ─────────────────────────────────────────── */
    if (left_pose_mode) {
        st = latch_joint_target(arm_left, &joints_left, DOF4_CONTROL_MODE_POSE);
        if (st != DOF4_STATUS_OK) {
            arm_left->last_status = st;
            return st;
        }
    }
    if (right_pose_mode) {
        st = latch_joint_target(arm_right, &joints_right, DOF4_CONTROL_MODE_POSE);
        if (st != DOF4_STATUS_OK) {
            arm_right->last_status = st;
            return st;
        }
    }

    /* ─── ⑥ 舵机下发 ─────────────────────────────────────────────── */
    st = Dof4_batch_write_all_servo(arm_left, arm_right);
    if (st != DOF4_STATUS_OK) {
        return st;
    }

    /* ─── ⑦ 状态更新 ─────────────────────────────────────────────── */
    arm_left->state   = left_pose_mode
                        ? (g_planner_left.running ? DOF4_ARM_STATE_MOVING : DOF4_ARM_STATE_IDLE)
                        : DOF4_ARM_STATE_MOVING;
    arm_right->state  = right_pose_mode
                        ? (g_planner_right.running ? DOF4_ARM_STATE_MOVING : DOF4_ARM_STATE_IDLE)
                        : DOF4_ARM_STATE_MOVING;
    arm_left->last_status  = DOF4_STATUS_OK;
    arm_right->last_status = DOF4_STATUS_OK;
    return DOF4_STATUS_OK;

}

/* ════════════════════════════════════════════════════════════════
 * 碰撞规避辅助函数
 * ════════════════════════════════════════════════════════════════ */
/**
 * @brief 基于碰撞详情计算右臂安全撤退位姿。
 *
 * 利用碰撞检测返回的 closest_a / closest_b 得到最近距离向量方向，
 * 将右臂当前位姿沿该方向推开 DOF4_AVOID_PUSH_M，并附加 Z 抬高。
 *
 * @param detail          碰撞详情（含 closest_a / closest_b）。
 * @param right_current   右臂当前 FK 位姿。
 * @param retreat         输出安全撤退位姿。
 */
static void dof4_compute_retreat_pose(const Dof4_CollisionDetail *detail,
                                      const Dof4_Pose *right_current,
                                      Dof4_Pose *retreat)
{
    float dir_x = detail->closest_b[0] - detail->closest_a[0];
    float dir_y = detail->closest_b[1] - detail->closest_a[1];
    float dir_z = detail->closest_b[2] - detail->closest_a[2];
    const float len = sqrtf(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);

    if (len > DOF4_GEOM_EPS) {
        dir_x /= len;
        dir_y /= len;
        dir_z /= len;
    } else {
        /* 退化：沿默认方向（右臂向右后方推开） */
        dir_x = 0.0f;
        dir_y = -1.0f;
        dir_z = 0.3f;
    }

    retreat->x     = right_current->x + dir_x * DOF4_AVOID_PUSH_M;
    retreat->y     = right_current->y + dir_y * DOF4_AVOID_PUSH_M;
    retreat->z     = right_current->z + dir_z * DOF4_AVOID_PUSH_M + DOF4_AVOID_Z_LIFT_M;
    retreat->pitch = right_current->pitch;
}

/**
 * @brief 检查单臂 FK 位姿是否已到达目标（位置 + pitch 均在容差内）。
 * @param arm    机械臂实例。
 * @param target 目标位姿。
 * @param tol_m  位置容差，单位 m。
 * @retval true  已到位。
 * @retval false 未到位。
 */
static bool dof4_is_pose_reached(const Dof4_Arm *arm,
                                 const Dof4_Pose *target,
                                 float tol_m)
{
    const float dx = arm->current_pose.x - target->x;
    const float dy = arm->current_pose.y - target->y;
    const float dz = arm->current_pose.z - target->z;
    const float pos_err = sqrtf(dx * dx + dy * dy + dz * dz);
    const float pitch_err = fabsf(Dof4_normalize_angle(arm->current_pose.pitch - target->pitch));
    return (pos_err <= tol_m) && (pitch_err <= tol_m);
}


/**
 * @brief 将位姿钳位到机械臂工作空间内。
 * @param arm 机械臂实例。
 * @param pose 输入输出位姿。
 * @retval Dof4_Status 状态码。
 */
/* Startup pose gate for the dual 4DOF arm with per-arm targets. */
Dof4_Status Dof4_dual_arm_startup_pose(Dof4_Arm *arm_left,
                                       Dof4_Arm *arm_right,
                                       const Dof4_Pose *target_left,
                                       const Dof4_Pose *target_right,
                                       uint32_t timeout_ms,
                                       float pos_tol_m,
                                       float pitch_tol_rad)
{
    if (arm_left == NULL || arm_right == NULL ||
        target_left == NULL || target_right == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (pos_tol_m <= 0.0f || pitch_tol_rad <= 0.0f) {
        return DOF4_STATUS_BAD_CONFIG;
    }

    Dof4_Status st = Dof4_arm_set_target(arm_left,
                                         target_left->x,
                                         target_left->y,
                                         target_left->z,
                                         target_left->pitch);
    if (st != DOF4_STATUS_OK) {
        arm_left->last_status = st;
        return st;
    }
    st = Dof4_arm_set_target(arm_right,
                             target_right->x,
                             target_right->y,
                             target_right->z,
                             target_right->pitch);
    if (st != DOF4_STATUS_OK) {
        arm_right->last_status = st;
        return st;
    }

    const uint32_t start_ms = dof4_get_tick_ms();
    Dof4_Status last_error = DOF4_STATUS_OK;

    while ((uint32_t)(dof4_get_tick_ms() - start_ms) < timeout_ms) {
        const uint32_t now_ms = dof4_get_tick_ms();
        st = Dof4_dual_arm_control_loop(arm_left, arm_right, now_ms);

        if (st != DOF4_STATUS_OK) {
            last_error = st;
            arm_left->last_status  = st;
            arm_right->last_status = st;
        } else {
            if (arm_left->last_status != DOF4_STATUS_OK) {
                last_error = arm_left->last_status;
            }
            if (arm_right->last_status != DOF4_STATUS_OK) {
                last_error = arm_right->last_status;
            }

            /* 左臂到位检查 */
            const float dx_l = arm_left->current_pose.x - target_left->x;
            const float dy_l = arm_left->current_pose.y - target_left->y;
            const float dz_l = arm_left->current_pose.z - target_left->z;
            const float pos_err_l = sqrtf(dx_l * dx_l + dy_l * dy_l + dz_l * dz_l);
            const float pitch_err_l = fabsf(Dof4_normalize_angle(arm_left->current_pose.pitch -
                                                                 target_left->pitch));
            bool left_ok = (pos_err_l <= pos_tol_m) && (pitch_err_l <= pitch_tol_rad);

            /* 右臂到位检查 */
            const float dx_r = arm_right->current_pose.x - target_right->x;
            const float dy_r = arm_right->current_pose.y - target_right->y;
            const float dz_r = arm_right->current_pose.z - target_right->z;
            const float pos_err_r = sqrtf(dx_r * dx_r + dy_r * dy_r + dz_r * dz_r);
            const float pitch_err_r = fabsf(Dof4_normalize_angle(arm_right->current_pose.pitch -
                                                                 target_right->pitch));
            bool right_ok = (pos_err_r <= pos_tol_m) && (pitch_err_r <= pitch_tol_rad);

            if (left_ok && right_ok) {
                arm_left->last_status  = DOF4_STATUS_OK;
                arm_right->last_status = DOF4_STATUS_OK;
                return DOF4_STATUS_OK;
            }
        }

        dof4_delay_ms(DOF4_STARTUP_CONTROL_PERIOD_MS);
    }

    st = (last_error != DOF4_STATUS_OK) ? last_error : DOF4_STATUS_NOT_READY;
    arm_left->last_status  = st;
    arm_right->last_status = st;
    return st;
}

/* Clamp a pose to the configured workspace (bounds shift with base_offset). */
Dof4_Status Dof4_clamp_to_workspace(const Dof4_Arm *arm, Dof4_Pose *pose)
{
    if (arm == NULL || pose == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    pose->x = clamp_float(pose->x,
                          arm->cfg.ws_min[0] + arm->cfg.base_offset[0],
                          arm->cfg.ws_max[0] + arm->cfg.base_offset[0]);
    pose->y = clamp_float(pose->y,
                          arm->cfg.ws_min[1] + arm->cfg.base_offset[1],
                          arm->cfg.ws_max[1] + arm->cfg.base_offset[1]);
    pose->z = clamp_float(pose->z,
                          arm->cfg.ws_min[2] + arm->cfg.base_offset[2],
                          arm->cfg.ws_max[2] + arm->cfg.base_offset[2]);
    return DOF4_STATUS_OK;
}

/**
 * @brief 获取用于胶囊体检测的连杆线段端点。
 * @param arm 机械臂实例。
 * @param endpoints 输出端点数组。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_arm_get_link_endpoints(const Dof4_Arm *arm,
                                        float endpoints[DOF4_LINK_SEGMENT_COUNT][2][3])
{
    if (arm == NULL || endpoints == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    for (uint8_t i = 0; i < DOF4_LINK_SEGMENT_COUNT; ++i) {
        endpoints[i][0][0] = arm->joint_world[i][0];
        endpoints[i][0][1] = arm->joint_world[i][1];
        endpoints[i][0][2] = arm->joint_world[i][2];
        endpoints[i][1][0] = arm->joint_world[i + 1U][0];
        endpoints[i][1][1] = arm->joint_world[i + 1U][1];
        endpoints[i][1][2] = arm->joint_world[i + 1U][2];
    }
    return DOF4_STATUS_OK;
}

/**
 * @brief 根据 ID 返回左右臂实例。
 * @param arm_id 机械臂 ID。
 * @param left 左臂实例。
 * @param right 右臂实例。
 * @retval Dof4_Arm* 对应实例，非法 ID 返回 NULL。
 */
Dof4_Arm *Dof4_arm_get_by_id(Dof4_ArmId arm_id, Dof4_Arm *left, Dof4_Arm *right)
{
    if (arm_id == DOF4_ARM_LEFT) {
        return left;
    }
    if (arm_id == DOF4_ARM_RIGHT) {
        return right;
    }
    return NULL;
}

/**
 * @brief 设置全局世界坐标系原点偏移。
 * @param dx X 偏移，单位 m。
 * @param dy Y 偏移，单位 m。
 * @param dz Z 偏移，单位 m。
 * @retval Dof4_Status 总是返回 DOF4_STATUS_OK。
 */
Dof4_Status Dof4_set_world_offset(float dx, float dy, float dz)
{
    g_dof4_world_offset.x = dx;
    g_dof4_world_offset.y = dy;
    g_dof4_world_offset.z = dz;
    return DOF4_STATUS_OK;
}

/**
 * @brief 读取全局世界坐标系原点偏移。
 * @param dx 输出 X 偏移，单位 m（可为 NULL）。
 * @param dy 输出 Y 偏移，单位 m（可为 NULL）。
 * @param dz 输出 Z 偏移，单位 m（可为 NULL）。
 * @retval Dof4_Status 总是返回 DOF4_STATUS_OK。
 */
Dof4_Status Dof4_get_world_offset(float *dx, float *dy, float *dz)
{
    if (dx != NULL) {
        *dx = g_dof4_world_offset.x;
    }
    if (dy != NULL) {
        *dy = g_dof4_world_offset.y;
    }
    if (dz != NULL) {
        *dz = g_dof4_world_offset.z;
    }
    return DOF4_STATUS_OK;
}

/**
 * @brief 设置双臂启动标志位，放行主控制循环。
 *
 * 调用后 g_dof4_arm_started 置为 true，arm_control_task 的主循环
 * 在下一帧检测到后开始执行正常的控制流水线。置位不可逆。
 */
void Dof4_double_arm_start(void)
{
    g_dof4_arm_started = true;
}


void Dof4_double_arm_Desable(void)
{
    relay_control(RELAY_LEFT_ARM,  SUCTION_OFF);
    relay_control(RELAY_RIGHT_ARM, SUCTION_OFF);
    relay_control(RELAY_LEFT_BACK,  SUCTION_OFF);
    relay_control(RELAY_RIGHT_BACK, SUCTION_OFF);

    /* 右臂 4 路 */
    for (uint8_t i = 0; i < 8; ++i) {
        EnableTorque((int)(i + 1), false);  /* ID 从 1 开始 */
    }
}

void Dof4_double_arm_Enable(void)
{
    relay_control(RELAY_LEFT_ARM,  SUCTION_ON);
    relay_control(RELAY_RIGHT_ARM, SUCTION_ON);
    relay_control(RELAY_LEFT_BACK,  SUCTION_ON);
    relay_control(RELAY_RIGHT_BACK, SUCTION_ON);

    /* 右臂 4 路 */
    for (uint8_t i = 0; i < 8; ++i) {
        EnableTorque((int)(i + 1), true);  /* ID 从 1 开始 */
    }
}
