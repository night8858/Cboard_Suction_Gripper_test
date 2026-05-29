/**
 * @file Dof4_Collision.h
 * @brief 双臂胶囊体碰撞检测接口。
 */

#ifndef DOF4_COLLISION_H
#define DOF4_COLLISION_H

#include <stdbool.h>
#include <stdint.h>
#include "Dof4_Arm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 默认胶囊体安全距离，单位 m。 */
#define DOF4_COL_DEFAULT_SAFE_DISTANCE_M 0.050f

/** @brief 默认 link1 胶囊体半径，单位 m。 */
#define DOF4_COL_RADIUS_LINK1_M 0.035f

/** @brief 默认 link2 胶囊体半径，单位 m。 */
#define DOF4_COL_RADIUS_LINK2_M 0.040f

/** @brief 默认 link3 胶囊体半径，单位 m。 */
#define DOF4_COL_RADIUS_LINK3_M 0.035f

/** @brief 默认 link4 胶囊体半径，单位 m。 */
#define DOF4_COL_RADIUS_LINK4_M 0.030f

/**
 * @brief 碰撞检测详细结果。
 */
typedef struct {
    bool is_safe;             /**< true 表示安全。 */
    float min_distance;       /**< 最近表面距离，单位 m。 */
    uint8_t link_a;           /**< 左臂最近连杆索引。 */
    uint8_t link_b;           /**< 右臂最近连杆索引。 */
    float closest_a[3];       /**< 左臂最近点。 */
    float closest_b[3];       /**< 右臂最近点。 */
} Dof4_CollisionDetail;

/**
 * @brief 计算两点距离。
 * @param a 点 A。
 * @param b 点 B。
 * @retval float 距离，单位 m。
 */
float Dof4_col_distance_point3(const float a[3], const float b[3]);

/**
 * @brief 计算点在线段上的投影参数。
 * @param point 点坐标。
 * @param seg_A 线段起点。
 * @param seg_B 线段终点。
 * @retval float 投影参数，范围 [0, 1]。
 */
float Dof4_col_point_to_segment_param(const float point[3],
                                      const float seg_A[3],
                                      const float seg_B[3]);

/**
 * @brief 计算两条三维线段最短距离。
 * @param A 第一条线段起点。
 * @param B 第一条线段终点。
 * @param C 第二条线段起点。
 * @param D 第二条线段终点。
 * @param closest1 输出第一条线段最近点，可为 NULL。
 * @param closest2 输出第二条线段最近点，可为 NULL。
 * @retval float 线段最短距离，单位 m。
 */
float Dof4_col_segment_distance(const float A[3],
                                const float B[3],
                                const float C[3],
                                const float D[3],
                                float closest1[3],
                                float closest2[3]);

/**
 * @brief 执行双臂胶囊体碰撞检测。
 * @param arm_left 左臂实例。
 * @param arm_right 右臂实例。
 * @param detail 输出详细结果，可为 NULL。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_collision_check_capsule(const Dof4_Arm *arm_left,
                                         const Dof4_Arm *arm_right,
                                         Dof4_CollisionDetail *detail);

/**
 * @brief 双臂碰撞检测统一入口。
 * @param arm_left 左臂实例。
 * @param arm_right 右臂实例。
 * @param detail 输出详细结果，可为 NULL。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_collision_check(const Dof4_Arm *arm_left,
                                 const Dof4_Arm *arm_right,
                                 Dof4_CollisionDetail *detail);

#ifdef __cplusplus
}
#endif

#endif /* DOF4_COLLISION_H */
