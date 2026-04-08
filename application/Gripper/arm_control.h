#ifndef __ARM_CONTROL_H__
#define __ARM_CONTROL_H__

#include "DM_motor.h"
#include "main.h"


typedef struct {
  float terminal_YAW;
  float terminal_PITCH;
  float terminal_ROLL;
  float terminal_X;
  float terminal_Y;
  float terminal_Z;

} Arm_terminal;

typedef struct {
  float Joint1_angle;
  float Joint2_angle;
  float Joint3_angle;
  float Joint4_angle;
  float Joint5_angle;
  float Joint6_angle;

} Arm_motor_angle;

typedef struct {

  float R_11;
  float R_12;
  float R_13;
  float R_21;
  float R_22;
  float R_23;
  float R_31;
  float R_32;
  float R_33;

} rotation_matrix;

/////////////////////////////////////////////////////////////////////////////////
void arm_task_init(void);
// 任务规划层
void arm_task_planning(void);
// 运动学逆解层
void arm_IK_calc(void);
// 轨迹规划层
void arm_trajectory_planning(void);
// 关节控制层
void arm_joint_controller(void);
// 位置反馈层
void arm_pos_feedback(void);
/////////////////////////////////////////////////////////////////////////////////
void gripper_loop(void) ;

void arm_2_start_point_simple(void);

void arm_config_init(void);
void arm_enable(void);
void arm_motor_controlSend(void);
void arm_pid_calc(void);
Arm_terminal forward_kinematics(float joint1_angle, float joint2_angle,
                                float joint3_angle, float joint4_angle,
                                float joint5_angle, float joint6_angle);
void arm_target_pos_set(s_DMmotor_data_t *target_motor, float target_position);
void gripper_motor_pos_set(Arm_motor_angle *target_motor_angle);
void joint_motor_pos_update(void);
rotation_matrix rotation_matrix_calc(Arm_terminal terminal_position);

float limit(float value, float min, float max);

void angle_set1(void);
void angle_set2(void);


#endif /* __ARM_CONTROL_H__ */
