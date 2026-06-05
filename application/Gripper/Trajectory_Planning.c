/**
 * @file Trajectory_Planning.c
 * @brief 笛卡尔五次多项式轨迹规划实现。
 */

#include "Trajectory_Planning.h"

#include <math.h>
#include <string.h>

/**
 * @brief 将浮点数限制到指定范围。
 * @param value 输入值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @retval float 限幅后的值。
 */
static float traj_clamp_float(float value, float min_value, float max_value)
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
 * @brief 用 Horner 形式计算五次多项式位置。
 * @param coeffs 多项式系数。
 * @param t 时间，单位 s。
 * @retval float 位置。
 */
static float quintic_eval_pos(const float coeffs[6], float t)
{
    return coeffs[0] + t * (coeffs[1] +
           t * (coeffs[2] +
           t * (coeffs[3] +
           t * (coeffs[4] +
           t *  coeffs[5]))));
}

/**
 * @brief 用 Horner 形式计算五次多项式速度。
 * @param coeffs 多项式系数。
 * @param t 时间，单位 s。
 * @retval float 速度。
 */
static float quintic_eval_vel(const float coeffs[6], float t)
{
    return coeffs[1] +
           t * (2.0f * coeffs[2] +
           t * (3.0f * coeffs[3] +
           t * (4.0f * coeffs[4] +
           t *  5.0f * coeffs[5])));
}

/**
 * @brief 用 Horner 形式计算五次多项式加速度。
 * @param coeffs 多项式系数。
 * @param t 时间，单位 s。
 * @retval float 加速度。
 */
static float quintic_eval_acc(const float coeffs[6], float t)
{
    return 2.0f * coeffs[2] +
           t * (6.0f * coeffs[3] +
           t * (12.0f * coeffs[4] +
           t *  20.0f * coeffs[5]));
}

/**
 * @brief 初始化单轴五次多项式轨迹段。
 * @param seg 轨迹段。
 * @param p0 起点位置。
 * @param v0 起点速度。
 * @param a0 起点加速度。
 * @param pf 终点位置。
 * @param vf 终点速度。
 * @param af 终点加速度。
 * @param duration 轨迹时长，单位 s。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Traj_quintic_init(Traj_QuinticSegment *seg,
                              float p0,
                              float v0,
                              float a0,
                              float pf,
                              float vf,
                              float af,
                              float duration)
{
    if (seg == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (duration < TRAJ_TIME_EPSILON) {
        return DOF4_STATUS_BAD_CONFIG;
    }

    memset(seg, 0, sizeof(*seg));
    seg->p0 = p0;
    seg->v0 = v0;
    seg->a0 = a0;
    seg->pf = pf;
    seg->vf = vf;
    seg->af = af;
    seg->duration = duration;
    seg->valid = true;

    const float T = duration;
    const float T2 = T * T;
    const float T3 = T2 * T;
    const float T4 = T3 * T;
    const float T5 = T4 * T;
    const float dp = pf - p0;

    seg->coeffs[0] = p0;
    seg->coeffs[1] = v0;
    seg->coeffs[2] = 0.5f * a0;
    seg->coeffs[3] = (20.0f * dp - (8.0f * vf + 12.0f * v0) * T
                     - (3.0f * a0 - af) * T2) / (2.0f * T3);
    seg->coeffs[4] = (30.0f * (p0 - pf) + (14.0f * vf + 16.0f * v0) * T
                     + (3.0f * a0 - 2.0f * af) * T2) / (2.0f * T4);
    seg->coeffs[5] = (12.0f * dp - (6.0f * vf + 6.0f * v0) * T
                     - (a0 - af) * T2) / (2.0f * T5);

    return DOF4_STATUS_OK;
}

/**
 * @brief 采样单轴五次多项式轨迹段。
 * @param seg 轨迹段。
 * @param elapsed_s 已运行时间，单位 s。
 * @param out_pos 输出位置，可为 NULL。
 * @param out_vel 输出速度，可为 NULL。
 * @param out_acc 输出加速度，可为 NULL。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Traj_quintic_sample(const Traj_QuinticSegment *seg,
                                float elapsed_s,
                                float *out_pos,
                                float *out_vel,
                                float *out_acc)
{
    if (seg == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (!seg->valid) {
        return DOF4_STATUS_NOT_READY;
    }

    float t = traj_clamp_float(elapsed_s, 0.0f, seg->duration);
    if (out_pos != NULL) {
        *out_pos = quintic_eval_pos(seg->coeffs, t);
    }
    if (out_vel != NULL) {
        *out_vel = quintic_eval_vel(seg->coeffs, t);
    }
    if (out_acc != NULL) {
        *out_acc = quintic_eval_acc(seg->coeffs, t);
    }

    if (elapsed_s >= seg->duration) {
        if (out_pos != NULL) {
            *out_pos = seg->pf;
        }
        if (out_vel != NULL) {
            *out_vel = seg->vf;
        }
        if (out_acc != NULL) {
            *out_acc = seg->af;
        }
    }

    return DOF4_STATUS_OK;
}

/**
 * @brief 初始化笛卡尔五次轨迹规划器。
 * @param planner 规划器。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_cartesian_planner_init(Dof4_CartesianPlanner *planner)
{
    if (planner == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

    memset(planner, 0, sizeof(*planner));
    return DOF4_STATUS_OK;
}

/**
 * @brief 规划 x/y/z/pitch 四通道五次轨迹。
 * @param planner 规划器。
 * @param start 起点位姿。
 * @param target 终点位姿。
 * @param start_time_ms 起始时间戳，单位 ms。
 * @param duration_s 轨迹时长，单位 s。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_cartesian_planner_plan(Dof4_CartesianPlanner *planner,
                                        const Dof4_Pose *start,
                                        const float v0[4],
                                        const Dof4_Pose *target,
                                        uint32_t start_time_ms,
                                        float duration_s)
{
    if (planner == NULL || start == NULL || target == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (duration_s < TRAJ_TIME_EPSILON) {
        return DOF4_STATUS_BAD_CONFIG;
    }

    planner->start_pose = *start;
    planner->target_pose = *target;
    planner->start_time_ms = start_time_ms;

    /* v0=NULL 时退化为零初速 */
    const float vx = (v0 != NULL) ? v0[0] : 0.0f;
    const float vy = (v0 != NULL) ? v0[1] : 0.0f;
    const float vz = (v0 != NULL) ? v0[2] : 0.0f;
    const float vp = (v0 != NULL) ? v0[3] : 0.0f;

    Dof4_Status st;
    st = Traj_quintic_init(&planner->axis[0], start->x, vx, 0.0f,
                           target->x, 0.0f, 0.0f, duration_s);
    if (st != DOF4_STATUS_OK) {
        return st;
    }
    st = Traj_quintic_init(&planner->axis[1], start->y, vy, 0.0f,
                           target->y, 0.0f, 0.0f, duration_s);
    if (st != DOF4_STATUS_OK) {
        return st;
    }
    st = Traj_quintic_init(&planner->axis[2], start->z, vz, 0.0f,
                           target->z, 0.0f, 0.0f, duration_s);
    if (st != DOF4_STATUS_OK) {
        return st;
    }
    st = Traj_quintic_init(&planner->axis[3], start->pitch, vp, 0.0f,
                           target->pitch, 0.0f, 0.0f, duration_s);
    if (st != DOF4_STATUS_OK) {
        return st;
    }

    planner->valid = true;
    planner->running = true;
    return DOF4_STATUS_OK;
}

/**
 * @brief 采样笛卡尔轨迹。
 * @param planner 规划器。
 * @param now_ms 当前时间戳，单位 ms。
 * @param pose 输出位姿。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_cartesian_planner_sample(Dof4_CartesianPlanner *planner,
                                          uint32_t now_ms,
                                          Dof4_Pose *pose)
{
    if (planner == NULL || pose == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }
    if (!planner->valid) {
        return DOF4_STATUS_NOT_READY;
    }

    const uint32_t delta_ms = (uint32_t)(now_ms - planner->start_time_ms);
    const float elapsed_s = (float)delta_ms * 0.001f;
    Dof4_Status st;

    st = Traj_quintic_sample(&planner->axis[0], elapsed_s, &pose->x, &planner->last_sample_vel[0], NULL);
    if (st != DOF4_STATUS_OK) {
        return st;
    }
    st = Traj_quintic_sample(&planner->axis[1], elapsed_s, &pose->y, &planner->last_sample_vel[1], NULL);
    if (st != DOF4_STATUS_OK) {
        return st;
    }
    st = Traj_quintic_sample(&planner->axis[2], elapsed_s, &pose->z, &planner->last_sample_vel[2], NULL);
    if (st != DOF4_STATUS_OK) {
        return st;
    }
    st = Traj_quintic_sample(&planner->axis[3], elapsed_s, &pose->pitch, &planner->last_sample_vel[3], NULL);
    if (st != DOF4_STATUS_OK) {
        return st;
    }

    if (elapsed_s >= planner->axis[0].duration) {
        planner->running = false;
        *pose = planner->target_pose;
    }
    planner->last_sample_pose = *pose;
    planner->has_last_sample = true;
    planner->has_last_vel = true;
    return DOF4_STATUS_OK;
}

/**
 * @brief 根据笛卡尔距离和 pitch 变化计算轨迹时长。
 * @param start 起点位姿。
 * @param target 终点位姿。
 * @param cart_vel_mps 笛卡尔速度上限，单位 m/s。
 * @param pitch_vel_rps pitch 速度上限，单位 rad/s。
 * @retval float 轨迹时长，单位 s。
 */
float Dof4_cartesian_compute_duration(const Dof4_Pose *start,
                                      const Dof4_Pose *target,
                                      float cart_vel_mps,
                                      float pitch_vel_rps)
{
    if (start == NULL || target == NULL) {
        return DOF4_TRAJ_MIN_DURATION_S;
    }

    const float dx = target->x - start->x;
    const float dy = target->y - start->y;
    const float dz = target->z - start->z;
    const float dist = sqrtf(dx * dx + dy * dy + dz * dz);
    const float dpitch = fabsf(target->pitch - start->pitch);
    const float safe_cart_vel = (cart_vel_mps > 1.0e-6f) ? cart_vel_mps : DOF4_DEFAULT_CART_VEL_MPS;
    const float safe_pitch_vel = (pitch_vel_rps > 1.0e-6f) ? pitch_vel_rps : DOF4_DEFAULT_PITCH_VEL_RPS;
    float duration = dist / safe_cart_vel;
    const float pitch_duration = dpitch / safe_pitch_vel;

    if (pitch_duration > duration) {
        duration = pitch_duration;
    }
    return traj_clamp_float(duration, DOF4_TRAJ_MIN_DURATION_S, DOF4_TRAJ_MAX_DURATION_S);
}

/**
 * @brief 判断目标是否需要触发重规划。
 * @param planner 规划器。
 * @param target 新目标。
 * @retval true 需要重规划。
 * @retval false 可继续当前轨迹。
 */
bool Dof4_cartesian_target_changed(const Dof4_CartesianPlanner *planner,
                                   const Dof4_Pose *target)
{
    if (planner == NULL || target == NULL || !planner->valid) {
        return true;
    }
    if (fabsf(target->x - planner->target_pose.x) > DOF4_REPLAN_POS_EPS_M) {
        return true;
    }
    if (fabsf(target->y - planner->target_pose.y) > DOF4_REPLAN_POS_EPS_M) {
        return true;
    }
    if (fabsf(target->z - planner->target_pose.z) > DOF4_REPLAN_POS_EPS_M) {
        return true;
    }
    if (fabsf(target->pitch - planner->target_pose.pitch) > DOF4_REPLAN_PITCH_EPS_RAD) {
        return true;
    }
    return false;
}
