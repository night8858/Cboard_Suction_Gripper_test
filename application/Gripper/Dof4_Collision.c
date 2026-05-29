/**
 * @file Dof4_Collision.c
 * @brief 双臂胶囊体碰撞检测实现。
 */

#include "Dof4_Collision.h"

#include <float.h>
#include <math.h>
#include <string.h>

/** @brief 线段退化判定阈值。 */
#define DOF4_COL_EPS 1.0e-8f

/**
 * @brief 三维向量点积。
 * @param a 向量 a。
 * @param b 向量 b。
 * @retval float 点积。
 */
static float vec3_dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * @brief 三维向量相减。
 * @param a 被减向量。
 * @param b 减向量。
 * @param out 输出 a-b。
 * @retval none 无。
 */
static void vec3_sub(const float a[3], const float b[3], float out[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

/**
 * @brief 三维向量长度平方。
 * @param v 输入向量。
 * @retval float 长度平方。
 */
static float vec3_len_sq(const float v[3])
{
    return vec3_dot(v, v);
}

/**
 * @brief 将浮点值限制到 [0, 1]。
 * @param value 输入值。
 * @retval float 限幅后的值。
 */
static float clamp01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

/**
 * @brief 获取指定连杆胶囊体半径。
 * @param link_index 连杆索引。
 * @retval float 半径，单位 m。
 */
static float get_link_radius(uint8_t link_index)
{
    static const float radii[DOF4_LINK_SEGMENT_COUNT] = {
        DOF4_COL_RADIUS_LINK1_M,
        DOF4_COL_RADIUS_LINK2_M,
        DOF4_COL_RADIUS_LINK3_M,
        DOF4_COL_RADIUS_LINK4_M
    };

    if (link_index >= DOF4_LINK_SEGMENT_COUNT) {
        return DOF4_COL_RADIUS_LINK4_M;
    }
    return radii[link_index];
}

/**
 * @brief 计算两个三维点的欧氏距离。
 * @param a 点 A。
 * @param b 点 B。
 * @retval float 距离，单位 m。
 */
float Dof4_col_distance_point3(const float a[3], const float b[3])
{
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 计算点在线段上的最近投影参数。
 * @param point 待投影点。
 * @param seg_A 线段起点。
 * @param seg_B 线段终点。
 * @retval float 投影参数，范围 [0, 1]。
 */
float Dof4_col_point_to_segment_param(const float point[3],
                                      const float seg_A[3],
                                      const float seg_B[3])
{
    float ab[3];
    float ap[3];
    vec3_sub(seg_B, seg_A, ab);
    vec3_sub(point, seg_A, ap);

    const float ab_len_sq = vec3_len_sq(ab);
    if (ab_len_sq < DOF4_COL_EPS) {
        return 0.0f;
    }

    return clamp01(vec3_dot(ap, ab) / ab_len_sq);
}

/**
 * @brief 计算两条三维线段之间的最短距离。
 * @param A 第一条线段起点。
 * @param B 第一条线段终点。
 * @param C 第二条线段起点。
 * @param D 第二条线段终点。
 * @param closest1 输出第一条线段最近点，可为 NULL。
 * @param closest2 输出第二条线段最近点，可为 NULL。
 * @retval float 最短距离，单位 m。
 */
float Dof4_col_segment_distance(const float A[3],
                                const float B[3],
                                const float C[3],
                                const float D[3],
                                float closest1[3],
                                float closest2[3])
{
    float u[3];
    float v[3];
    float w[3];
    vec3_sub(B, A, u);
    vec3_sub(D, C, v);
    vec3_sub(A, C, w);

    const float a = vec3_dot(u, u);
    const float b = vec3_dot(u, v);
    const float c = vec3_dot(v, v);
    const float d = vec3_dot(u, w);
    const float e = vec3_dot(v, w);
    const float denom = a * c - b * b;

    float s;
    float t;

    if (a < DOF4_COL_EPS && c < DOF4_COL_EPS) {
        s = 0.0f;
        t = 0.0f;
    } else if (a < DOF4_COL_EPS) {
        s = 0.0f;
        t = clamp01(e / c);
    } else if (c < DOF4_COL_EPS) {
        t = 0.0f;
        s = clamp01(-d / a);
    } else if (fabsf(denom) < DOF4_COL_EPS) {
        s = 0.0f;
        t = clamp01(e / c);
    } else {
        s = clamp01((b * e - c * d) / denom);
        t = clamp01((a * e - b * d) / denom);

        /* 钳位 s 后重新投影 t，可避免最近点落到线段外。 */
        t = clamp01((b * s + e) / c);
        /* 钳位 t 后重新投影 s，使两条有限线段边界情况稳定。 */
        s = clamp01((b * t - d) / a);
    }

    float p[3];
    float q[3];
    p[0] = A[0] + s * u[0];
    p[1] = A[1] + s * u[1];
    p[2] = A[2] + s * u[2];
    q[0] = C[0] + t * v[0];
    q[1] = C[1] + t * v[1];
    q[2] = C[2] + t * v[2];

    if (closest1 != NULL) {
        memcpy(closest1, p, sizeof(p));
    }
    if (closest2 != NULL) {
        memcpy(closest2, q, sizeof(q));
    }
    return Dof4_col_distance_point3(p, q);
}

/**
 * @brief 使用连杆胶囊体模型检测双臂碰撞风险。
 * @param arm_left 左臂实例。
 * @param arm_right 右臂实例。
 * @param detail 输出最近距离细节，可为 NULL。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_collision_check_capsule(const Dof4_Arm *arm_left,
                                         const Dof4_Arm *arm_right,
                                         Dof4_CollisionDetail *detail)
{
    if (arm_left == NULL || arm_right == NULL) {
        return DOF4_STATUS_NULL_PARAM;
    }

    float endpoints_l[DOF4_LINK_SEGMENT_COUNT][2][3];
    float endpoints_r[DOF4_LINK_SEGMENT_COUNT][2][3];
    Dof4_Status st = Dof4_arm_get_link_endpoints(arm_left, endpoints_l);
    if (st != DOF4_STATUS_OK) {
        return st;
    }
    st = Dof4_arm_get_link_endpoints(arm_right, endpoints_r);
    if (st != DOF4_STATUS_OK) {
        return st;
    }

    Dof4_CollisionDetail local;
    memset(&local, 0, sizeof(local));
    local.is_safe = true;
    local.min_distance = FLT_MAX;

    for (uint8_t i = 0; i < DOF4_LINK_SEGMENT_COUNT; ++i) {
        for (uint8_t j = 0; j < DOF4_LINK_SEGMENT_COUNT; ++j) {
            float pa[3];
            float pb[3];
            const float center_dist = Dof4_col_segment_distance(endpoints_l[i][0],
                                                                endpoints_l[i][1],
                                                                endpoints_r[j][0],
                                                                endpoints_r[j][1],
                                                                pa,
                                                                pb);
            const float surface_dist = center_dist - get_link_radius(i) - get_link_radius(j);
            if (surface_dist < local.min_distance) {
                local.min_distance = surface_dist;
                local.link_a = i;
                local.link_b = j;
                memcpy(local.closest_a, pa, sizeof(pa));
                memcpy(local.closest_b, pb, sizeof(pb));
            }
        }
    }

    if (local.min_distance < DOF4_COL_DEFAULT_SAFE_DISTANCE_M) {
        local.is_safe = false;
    }
    if (detail != NULL) {
        *detail = local;
    }
    return local.is_safe ? DOF4_STATUS_OK : DOF4_STATUS_COLLISION_RISK;
}

/**
 * @brief 双臂碰撞检测统一入口。
 * @param arm_left 左臂实例。
 * @param arm_right 右臂实例。
 * @param detail 输出最近距离细节，可为 NULL。
 * @retval Dof4_Status 状态码。
 */
Dof4_Status Dof4_collision_check(const Dof4_Arm *arm_left,
                                 const Dof4_Arm *arm_right,
                                 Dof4_CollisionDetail *detail)
{
    return Dof4_collision_check_capsule(arm_left, arm_right, detail);
}
