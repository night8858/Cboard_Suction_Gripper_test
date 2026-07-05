#ifndef HOST_STUB_DOF4_ARM_H
#define HOST_STUB_DOF4_ARM_H

#include <stdbool.h>
#include <stdint.h>

#define DOF4_JOINT_COUNT 4U
#define DOF4_CLIP_REASON_JOINT_LIMIT 0x01U
#define DOF4_CLIP_REASON_SERVO_LIMIT 0x02U
#define DOF4_CLIP_REASON_PC_ACTION_REJECT 0x04U

typedef enum {
    DOF4_STATUS_OK = 0,
    DOF4_STATUS_IK_UNREACHABLE = 4,
} Dof4_Status;

typedef enum {
    DOF4_ARM_LEFT = 0,
    DOF4_ARM_RIGHT = 1,
} Dof4_ArmId;

typedef enum {
    DOF4_CONTROL_MODE_POSE = 0,
    DOF4_CONTROL_MODE_JOINT = 1,
} Dof4_ControlMode;

typedef struct {
    float x;
    float y;
    float z;
    float pitch;
} Dof4_Pose;

typedef struct {
    float q[DOF4_JOINT_COUNT];
} Dof4_JointState;

typedef struct {
    bool pending;
    uint32_t event_id;
    Dof4_ControlMode control_mode;
    uint8_t reason;
    uint8_t joint_mask;
    Dof4_Pose requested_pose;
    Dof4_JointState requested_joints;
    Dof4_JointState limited_joints;
    Dof4_Pose limited_pose;
    int16_t target_servo_pos[DOF4_JOINT_COUNT];
} Dof4_ClipDiagnostic;

typedef struct {
    float joint_min[DOF4_JOINT_COUNT];
    float joint_max[DOF4_JOINT_COUNT];
    float base[3];
    float base_offset[3];
    float cart_vel_mps;
    float pitch_vel_rps;
    uint16_t servo_speed;
} Dof4_ArmConfig;

typedef struct {
    Dof4_ArmConfig cfg;
    Dof4_JointState joint_actual;
    Dof4_JointState joint_target;
    Dof4_Pose current_pose;
    Dof4_Pose target_pose;
    int16_t target_servo_pos[DOF4_JOINT_COUNT];
    uint32_t clip_event_counter;
    Dof4_ClipDiagnostic clip_diagnostic;
} Dof4_Arm;

extern Dof4_Arm g_dof4_arm_left;
extern Dof4_Arm g_dof4_arm_right;
extern bool g_dof4_arm_started;

Dof4_Status Dof4_arm_set_target(Dof4_Arm *arm, float x, float y, float z, float pitch);
Dof4_Status Dof4_arm_set_target_via(Dof4_Arm *arm,
                                    float x,
                                    float y,
                                    float z,
                                    float pitch,
                                    const float via_vel[4]);
Dof4_Status Dof4_arm_set_joint_target(Dof4_Arm *arm, const Dof4_JointState *joints);
Dof4_Status Dof4_arm_inverse_kinematics(const Dof4_Arm *arm,
                                        const Dof4_Pose *target,
                                        float elbow_sign,
                                        Dof4_JointState *joints);
Dof4_Status Dof4_angle_to_servo(const Dof4_Arm *arm,
                                uint8_t joint_index,
                                float angle_rad,
                                int16_t *servo_pos);
Dof4_Status Dof4_servo_to_angle(const Dof4_Arm *arm,
                                uint8_t joint_index,
                                int16_t servo_pos,
                                float *angle_rad);
Dof4_Status Dof4_clamp_to_workspace(const Dof4_Arm *arm, Dof4_Pose *pose);
Dof4_Status Dof4_set_world_offset(float dx, float dy, float dz);
float Dof4_normalize_angle(float angle_rad);
void Dof4_double_arm_start(void);

#endif
