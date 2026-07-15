/**
 * @file Dof4_Arm_Calibration.h
 * @brief 4DOF 机械臂现场标定和动作调参集中入口。
 */

#ifndef DOF4_ARM_CALIBRATION_H
#define DOF4_ARM_CALIBRATION_H

#include "Dof4_Arm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DOF4_CALIB_ARM_COUNT DOF4_ARM_COUNT
#define DOF4_CALIB_PC_PRE_POSE_MAX 5U
#define DOF4_CALIB_PC_POST_POSE_MAX 4U
#define DOF4_CALIB_PC_PRE_JOINT_MAX 6U
#define DOF4_CALIB_PC_POST_JOINT_MAX 4U

typedef struct {
    float x;
    float y;
    float z;
} Dof4_CalibrationOffset3;

typedef struct {
    uint8_t servo_id[DOF4_JOINT_COUNT];
    float base[3];
    float base_offset[3];
    float shoulder_r;
    float shoulder_z;
    float link_len[3];
    float tcp_offset[3];
    float pitch_offset;
    float joint_min[DOF4_JOINT_COUNT];
    float joint_max[DOF4_JOINT_COUNT];
    int16_t servo_min[DOF4_JOINT_COUNT];
    int16_t servo_max[DOF4_JOINT_COUNT];
    int16_t servo_zero[DOF4_JOINT_COUNT];
    float servo_offset[DOF4_JOINT_COUNT];
    int8_t servo_sign[DOF4_JOINT_COUNT];
    uint8_t servo_reverse[DOF4_JOINT_COUNT];
    uint16_t servo_speed;
    uint8_t servo_acc;
    float ws_min[3];
    float ws_max[3];
    float cart_vel_mps;
    float pitch_vel_rps;
} Dof4_SingleArmCalibration;

typedef struct {
    Dof4_SingleArmCalibration arm[DOF4_CALIB_ARM_COUNT];
    Dof4_CalibrationOffset3 world_offset;
    Dof4_CalibrationOffset3 target_bias[DOF4_CALIB_ARM_COUNT];
} Dof4ArmCalibration;

typedef struct {
    Dof4_Pose entry_offset;
    Dof4_Pose exit_offset; /**< 当前动态撤离仅使用 y；x/z/pitch 保留。 */
    float target_pitch;
    float vertical_clearance_m;
} Dof4PcActionDynamicTemplate;

typedef struct {
    float q[DOF4_JOINT_COUNT];
} Dof4PcActionJointPoint;

typedef struct {
    Dof4PcActionJointPoint pre[DOF4_CALIB_PC_PRE_JOINT_MAX];
    Dof4PcActionJointPoint post[DOF4_CALIB_PC_POST_JOINT_MAX];
    uint8_t pre_count;
    uint8_t post_count;
} Dof4PcActionJointPath;

typedef struct {
    Dof4PcActionDynamicTemplate pick[DOF4_CALIB_ARM_COUNT];
    Dof4PcActionDynamicTemplate dual_pick[DOF4_CALIB_ARM_COUNT];
    Dof4PcActionDynamicTemplate place[DOF4_CALIB_ARM_COUNT];
    Dof4PcActionJointPath put_back[DOF4_CALIB_ARM_COUNT];
    Dof4PcActionJointPath dual_put_back[DOF4_CALIB_ARM_COUNT];
    Dof4PcActionJointPath get_back[DOF4_CALIB_ARM_COUNT];
    uint32_t move_timeout_ms;
    float pose_pos_tol_m;
    float pose_pitch_tol_rad;
    float path_point_max_adjust_m; /**< 动态路径中间点最大 XYZ 邻近可达修正量。 */
    float joint_tol_rad;
    uint16_t back_servo_speed;
    uint16_t back_fast_servo_speed;
    uint16_t back_final_servo_speed;
    uint8_t back_fast_servo_acc;
    uint8_t back_final_servo_acc;
    float back_final_joint_tol_rad;
    uint8_t back_final_stable_frames;
    uint8_t dynamic_hold_pre_index;
    uint32_t delay_dynamic_pick_hold_ms;
    uint32_t delay_dynamic_place_release_ms;
    uint32_t delay_dynamic_target_settle_ms;
    uint32_t delay_back_pre_release_ms;
    uint32_t delay_back_post_release_ms;
    uint32_t delay_back_get_arm_hold_ms;
    uint32_t delay_back_source_release_ms;
    uint32_t delay_idle_hold_ms;
    uint32_t delay_dynamic_hover_hold_ms;
} Dof4PcActionCalibration;

typedef struct {
    Dof4_Pose idle_offset[DOF4_CALIB_ARM_COUNT];
    Dof4_Pose default_back_avoid[DOF4_CALIB_ARM_COUNT];
    Dof4_Pose current_back_avoid[DOF4_CALIB_ARM_COUNT];
    uint32_t move_timeout_ms;
    uint32_t suction_timeout_ms;
    uint32_t place_hold_ms;
    uint32_t back_release_hold_ms;
    uint32_t place_pre_release_back_ms;
    uint32_t place_post_release_back_ms;
    uint32_t place_post_release_ext_ms;
    uint32_t release_timeout_ms;
    uint32_t hold_ms;
    uint32_t waypoint_hold_ms;
    float default_blend_dist_m;
    float default_via_speed_factor;
    float chain_final_blend_dist_m;
    float reach_pos_tol_m;
    float reach_pitch_tol_rad;
    float joint_reach_tol_rad;
    float joint_blend_tol_rad;
} Dof4ActionCalibration;

const Dof4ArmCalibration *Dof4_calibration_get_arm(void);
const Dof4PcActionCalibration *Dof4_calibration_get_pc_action(void);
const Dof4ActionCalibration *Dof4_calibration_get_action(void);

void Dof4_calibration_apply_runtime_offsets(Dof4_Arm *left, Dof4_Arm *right);

#ifdef __cplusplus
}
#endif

#endif /* DOF4_ARM_CALIBRATION_H */
