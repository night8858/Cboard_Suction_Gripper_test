/**
 * @file Trajectory_Planning.h
 * @brief 笛卡尔五次多项式轨迹规划接口。
 */

#ifndef TRAJECTORY_PLANNING_H
#define TRAJECTORY_PLANNING_H

#include <stdbool.h>
#include <stdint.h>
#include "Dof4_Arm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 五次多项式时间下限。 */
#define TRAJ_TIME_EPSILON 1.0e-6f

/**
 * @brief 单通道五次多项式段。
 */
typedef struct {
    float p0;            /**< 起点位置。 */
    float v0;            /**< 起点速度。 */
    float a0;            /**< 起点加速度。 */
    float pf;            /**< 终点位置。 */
    float vf;            /**< 终点速度。 */
    float af;            /**< 终点加速度。 */
    float duration;      /**< 段时长，单位 s。 */
    float coeffs[6];     /**< 多项式系数 a0~a5。 */
    bool valid;          /**< 段是否有效。 */
} Traj_QuinticSegment;

/**
 * @brief 笛卡尔 x/y/z/pitch 五次规划器。
 */
typedef struct {
    Traj_QuinticSegment axis[4]; /**< x、y、z、pitch 四通道。 */
    Dof4_Pose start_pose;        /**< 起点位姿。 */
    Dof4_Pose target_pose;       /**< 终点位姿。 */
    uint32_t start_time_ms;      /**< 起始时间戳，单位 ms。 */
    bool valid;                  /**< 是否已规划。 */
    bool running;                /**< 是否正在运行。 */
} Dof4_CartesianPlanner;

/**
 * @brief 初始化单通道五次多项式段。
 * @param seg 轨迹段。
 * @param p0 起点位置。
 * @param v0 起点速度。
 * @param a0 起点加速度。
 * @param pf 终点位置。
 * @param vf 终点速度。
 * @param af 终点加速度。
 * @param duration 段时长，单位 s。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Traj_quintic_init(Traj_QuinticSegment *seg,
                              float p0,
                              float v0,
                              float a0,
                              float pf,
                              float vf,
                              float af,
                              float duration);

/**
 * @brief 采样单通道五次多项式。
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
                                float *out_acc);

/**
 * @brief 初始化笛卡尔轨迹规划器。
 * @param planner 规划器。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_cartesian_planner_init(Dof4_CartesianPlanner *planner);

/**
 * @brief 规划 x/y/z/pitch 笛卡尔五次轨迹。
 * @param planner 规划器。
 * @param start 起点位姿。
 * @param target 终点位姿。
 * @param start_time_ms 起始时间戳，单位 ms。
 * @param duration_s 轨迹时长，单位 s。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_cartesian_planner_plan(Dof4_CartesianPlanner *planner,
                                        const Dof4_Pose *start,
                                        const Dof4_Pose *target,
                                        uint32_t start_time_ms,
                                        float duration_s);

/**
 * @brief 采样笛卡尔轨迹。
 * @param planner 规划器。
 * @param now_ms 当前时间戳，单位 ms。
 * @param pose 输出位姿。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_cartesian_planner_sample(Dof4_CartesianPlanner *planner,
                                          uint32_t now_ms,
                                          Dof4_Pose *pose);

/**
 * @brief 根据位姿差和速度上限计算轨迹时长。
 * @param start 起点位姿。
 * @param target 终点位姿。
 * @param cart_vel_mps 笛卡尔速度上限，单位 m/s。
 * @param pitch_vel_rps pitch 速度上限，单位 rad/s。
 * @retval float 轨迹时长，单位 s。
 */
float Dof4_cartesian_compute_duration(const Dof4_Pose *start,
                                      const Dof4_Pose *target,
                                      float cart_vel_mps,
                                      float pitch_vel_rps);

/**
 * @brief 判断目标是否相对上一规划发生显著变化。
 * @param planner 规划器。
 * @param target 新目标。
 * @retval bool true 表示需要重规划。
 */
bool Dof4_cartesian_target_changed(const Dof4_CartesianPlanner *planner,
                                   const Dof4_Pose *target);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_PLANNING_H */
