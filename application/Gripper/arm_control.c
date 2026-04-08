#include "main.h"
#include "math.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "DM_motor.h"
#include "PID.h"
#include "arm_control.h"
#include "variables.h"
#include <stdint.h>

extern s_DMmotor_data_t DMmotor_4340[3];
extern s_DMmotor_data_t DMmotor_4310[3];

Arm_motor_angle real_motor_angle;
Arm_motor_angle target_motor_angle;

Arm_terminal real_arm_terminal;
Arm_terminal target_arm_terminal;
rotation_matrix R_euler_angle;
/////////////////////////////////////////////////////////////////////////////////

// start angle -0.002,2.292,-0.054,4.027,-0.444,0.000
//-2.794 2.292,-0.054,1.235,-0.444,0.000

void gripper_loop(void) {
  arm_pos_feedback();
  arm_joint_controller();
}

// 机械臂任务初始化
void arm_task_init(void) {
  arm_config_init();
  arm_enable();
}

// 任务规划层
void arm_task_planning(void) {
  // 机械臂任务规划逻辑
  // ...
}

// 运动学逆解层
void arm_IK_calc(void) {
  // 根据末端目标位姿计算各关节的目标位置
  // ...
}

// 轨迹规划层
void arm_trajectory_planning(void) {
  // 计算机械臂各个关节的目标位置

  // ...
}

// 关节控制层
void arm_joint_controller(void) {
  gripper_motor_pos_set(&target_motor_angle);
  arm_pid_calc();          // 计算PID
  arm_motor_controlSend(); // 发送控制指令
}

// 位置反馈层
void arm_pos_feedback(void) {
  joint_motor_pos_update(); // 更新各关节的实际位置
  real_arm_terminal = forward_kinematics(
      real_motor_angle.Joint1_angle, real_motor_angle.Joint2_angle,
      real_motor_angle.Joint3_angle, real_motor_angle.Joint4_angle,
      real_motor_angle.Joint5_angle, real_motor_angle.Joint6_angle);
}

///////////////////////////////////////////////////////////////////////////////

// 机械臂初始化配置
void arm_config_init(void) {
  // 初始化机械臂各个关节的电机
  DM_motor_config(&DMmotor_4340[0], 0x01, &hcan1, DM4340, PID_control);
  DM_motor_config(&DMmotor_4340[1], 0x02, &hcan1, DM4340, PID_control);
  DM_motor_config(&DMmotor_4340[2], 0x03, &hcan1, DM4340, PID_control);
  DM_motor_config(&DMmotor_4310[0], 0x04, &hcan1, DM4310, PID_control);
  DM_motor_config(&DMmotor_4310[1], 0x05, &hcan1, DM4310, PID_control);
  DM_motor_config(&DMmotor_4310[2], 0x06, &hcan1, DM4310, PID_control);
  // 初始化机械臂各个关节电机的PID参数MIT
  DM_motor_PID_init(&DMmotor_4340[0], 17.8, 0.0, 10.4, 1.0, 27, PID_POS);
  DM_motor_PID_init(&DMmotor_4340[1], 21.8, 0.0, 10.4, 1.0, 27, PID_POS);
  DM_motor_PID_init(&DMmotor_4340[2], 42.5, 0.0, 18.4, 1.0, 27, PID_POS);
  DM_motor_PID_init(&DMmotor_4310[0], 6.2, 0.0, 4.3, 1.0, 9, PID_POS);
  DM_motor_PID_init(&DMmotor_4310[1], 6.2, 0.0, 4.3, 1.0, 9, PID_POS);
  DM_motor_PID_init(&DMmotor_4310[2], 6.2, 0.0, 4.3, 1.0, 9, PID_POS);

  DM_motor_PID_init(&DMmotor_4340[0], 0.42, 0.1, 0.5, 1.8, 9, PID_SPD);
  DM_motor_PID_init(&DMmotor_4340[1], 0.42, 0.1, 0.5, 1.8, 9, PID_SPD);
  DM_motor_PID_init(&DMmotor_4340[2], 0.6, 0.1, 0.5, 1.8, 9, PID_SPD);
  DM_motor_PID_init(&DMmotor_4310[0], 0.4, 0.1, 0.5, 1.0, 3, PID_SPD);
  DM_motor_PID_init(&DMmotor_4310[1], 0.4, 0.1, 0.5, 1.0, 3, PID_SPD);
  DM_motor_PID_init(&DMmotor_4310[2], 0.4, 0.1, 0.5, 1.0, 3, PID_SPD);
}

void arm_enable(void) {
  for (int i = 0; i < 3; i++) {
    DM_motor_Disable(&DMmotor_4340[i]);
    osDelay(200);
    DM_motor_Disable(&DMmotor_4310[i]);
    osDelay(200);
  }
  // 使能机械臂各个关节的电机
  for (int i = 0; i < 3; i++) {
    DM_motor_Enable(&DMmotor_4340[i]);
    osDelay(200);
    DM_motor_Enable(&DMmotor_4340[i]);
    osDelay(50);
  }
  for (int i = 0; i < 3; i++) {
    DM_motor_Enable(&DMmotor_4310[i]);
    osDelay(200);
    DM_motor_Enable(&DMmotor_4310[i]);
    osDelay(50);
  }
}
// 设置机械臂各个关节的目标位置

void angle_set1(void) {

  target_motor_angle.Joint1_angle = 0.0f;
  target_motor_angle.Joint2_angle = 0.0f;
  target_motor_angle.Joint3_angle = -(0.0f);
  target_motor_angle.Joint4_angle = 0.0f;
  target_motor_angle.Joint5_angle = 0.0f;
  target_motor_angle.Joint6_angle = 0.0f;
}

void angle_set2(void) {
  target_motor_angle.Joint1_angle = 0.0f;
  target_motor_angle.Joint2_angle = -0.5f;
  target_motor_angle.Joint3_angle = -(-0.5f);
  target_motor_angle.Joint4_angle = 0.0f;
  target_motor_angle.Joint5_angle = 0.0f;
  target_motor_angle.Joint6_angle = 0.0f;
}
void gripper_motor_pos_set(Arm_motor_angle *target_motor_angle) {
  // target_motor_angle->Joint1_angle = 0.0f;
  // target_motor_angle->Joint2_angle = -0.5f;
  // target_motor_angle->Joint3_angle = -(-0.5f);
  // target_motor_angle->Joint4_angle = 0.0f;
  // target_motor_angle->Joint5_angle = 0.0f;
  // target_motor_angle->Joint6_angle = 0.0f;

  target_motor_angle->Joint1_angle =
      limit(target_motor_angle->Joint1_angle, -2.618f, 2.618f); // 150度
  target_motor_angle->Joint2_angle =
      limit(target_motor_angle->Joint2_angle, -3.14f, 0.0f);
  target_motor_angle->Joint3_angle =
      limit(target_motor_angle->Joint3_angle, 0.0f, 3.14159f);
  target_motor_angle->Joint4_angle =
      limit(target_motor_angle->Joint4_angle, -2.792f, 2.792f);
  target_motor_angle->Joint5_angle =
      limit(target_motor_angle->Joint5_angle, -2.400f, 2.400f);
  target_motor_angle->Joint6_angle =
      limit(target_motor_angle->Joint6_angle, -2.618f, 2.618f);

  arm_target_pos_set(&DMmotor_4340[0],
                     target_motor_angle->Joint1_angle - 2.794f);
  arm_target_pos_set(&DMmotor_4340[1],
                     target_motor_angle->Joint2_angle + 2.297f);
  arm_target_pos_set(&DMmotor_4340[2],
                     target_motor_angle->Joint3_angle - 0.057f);
  arm_target_pos_set(&DMmotor_4310[0],
                     target_motor_angle->Joint4_angle + 1.235f);
  arm_target_pos_set(&DMmotor_4310[1],
                     target_motor_angle->Joint5_angle + 1.572f);
  arm_target_pos_set(&DMmotor_4310[2],
                     target_motor_angle->Joint6_angle + 2.792f);
}

// 设置单个机械臂关节电机的目标位置
void arm_target_pos_set(s_DMmotor_data_t *target_motor, float target_position) {
  target_motor->target_position = target_position;
}

// 机械臂PID计算函数
void arm_pid_calc(void) {
  // for (int i = 0; i < 3; i++) {
  //   motor_double_loop_PID(
  //       &DMmotor_4340[i].position_pid, &DMmotor_4340[i].speed_pid,
  //       DMmotor_4340[i].esc_back_position, DMmotor_4340[i].target_position,
  //       DMmotor_4340[i].esc_back_speed);
  // }
  for (int i = 0; i < 3; i++) {
    /***********DM4340电机pid计算***********/
    DMmotor_4310[i].position_pid.NowError =
        DMmotor_4310[i].target_position -
        DMmotor_4310[i].esc_back_position; // bat_control->pitch_pos_filter.out
    PID_AbsoluteMode(&DMmotor_4310[i].position_pid);
    DMmotor_4310[i].speed_pid.NowError =
        DMmotor_4310[i].position_pid.PIDout - DMmotor_4310[i].esc_back_speed;
    PID_AbsoluteMode(&DMmotor_4310[i].speed_pid);
    DMmotor_4310[i].target_out_current = DMmotor_4310[i].speed_pid.PIDout;

    DMmotor_4340[i].position_pid.NowError =
        DMmotor_4340[i].target_position -
        DMmotor_4340[i].esc_back_position; // bat_control->pitch_pos_filter.out
    PID_AbsoluteMode(&DMmotor_4340[i].position_pid);
    DMmotor_4340[i].speed_pid.NowError =
        DMmotor_4340[i].position_pid.PIDout - DMmotor_4340[i].esc_back_speed;
    PID_AbsoluteMode(&DMmotor_4340[i].speed_pid);
    DMmotor_4340[i].target_out_current = DMmotor_4340[i].speed_pid.PIDout;
    // 测试用，将所有电机电流设置为0
    DMmotor_4340[i].target_out_current = 0;
    DMmotor_4310[i].target_out_current = 0;
  }
}

// 机械臂电机控制发送函数
void arm_motor_controlSend(void) {

  for (int i = 0; i < 3; i++) {
    DM_motor_control(&DMmotor_4340[i]);
    osDelay(1);
    DM_motor_control(&DMmotor_4310[i]);
    osDelay(1);
  }
}

// 更新机械臂各个关节的实际角度值
void joint_motor_pos_update(void) {
  // 加上的数字是机械臂的初始角度偏移量
  real_motor_angle.Joint1_angle = DMmotor_4340[0].esc_back_position + 2.794f;
  real_motor_angle.Joint2_angle = (DMmotor_4340[1].esc_back_position - 2.297f);
  real_motor_angle.Joint3_angle = -(DMmotor_4340[2].esc_back_position + 0.057f);
  real_motor_angle.Joint4_angle = DMmotor_4310[0].esc_back_position - 1.235f;
  real_motor_angle.Joint5_angle = DMmotor_4310[1].esc_back_position - 1.572f;
  real_motor_angle.Joint6_angle = DMmotor_4310[2].esc_back_position - 2.792f;
}

/**
 * @brief  机械臂正运动学计算函数
 * @note   根据各关节角度计算机械臂末端的位姿（位置和姿态）
 *         本函数目前是一个框架，需要根据实际的机械臂连杆参数完成具体实现
 * @param  joint1_angle: 关节1的角度（通常是基座旋转），单位：弧度
 * @param  joint2_angle: 关节2的角度（通常是大臂俯仰），单位：弧度
 * @param  joint3_angle: 关节3的角度（通常是小臂俯仰），单位：弧度
 * @param  joint4_angle: 关节4的角度（通常是腕部旋转），单位：弧度
 * @param  joint5_angle: 关节5的角度（通常是腕部俯仰），单位：弧度
 * @param  joint6_angle: 关节6的角度（通常是末端旋转），单位：弧度
 * @retval Arm_terminal: 包含末端位置(X,Y,Z)和姿态(YAW,PITCH,ROLL)的结构体
 */
Arm_terminal forward_kinematics(float joint1_angle, float joint2_angle,
                                float joint3_angle, float joint4_angle,
                                float joint5_angle, float joint6_angle) {

  /*
   * 1. 定义机械臂的DH参数或连杆参数
   * 2. 构建各关节的变换矩阵
   * 3. 计算末端相对于基坐标系的总变换矩阵
   * 4. 从总变换矩阵中提取位置和姿态信息
   */
  /* 定义末端位姿结构体变量 */
  Arm_terminal terminal_position;

  // 处理初始offest
  double joint1_angle_offest = joint1_angle + 3.14159f;
  double joint3_angle_offest = joint3_angle + 1.571f;
  double joint4_angle_offest = joint4_angle + 3.14159f;
  double joint5_angle_offest = joint5_angle;

  terminal_position.terminal_X = 0.0f;
  terminal_position.terminal_Y = 0.0f;
  terminal_position.terminal_Z = 0.0f;
  terminal_position.terminal_YAW = 0.0f;
  terminal_position.terminal_PITCH = 0.0f;
  terminal_position.terminal_ROLL = 0.0f;

  double terminal_X = (11 * cos(joint1_angle_offest) * cos(joint2_angle)) / 50 +
                      (11 * cos(joint1_angle_offest) * cos(joint2_angle) *
                       cos(joint3_angle_offest)) /
                          200 -
                      (2969 * cos(joint1_angle_offest) * cos(joint2_angle) *
                       sin(joint3_angle_offest)) /
                          10000 +
                      (2969 * cos(joint1_angle_offest) *
                       cos(joint3_angle_offest) * sin(joint2_angle)) /
                          10000 +
                      (11 * cos(joint1_angle_offest) * sin(joint2_angle) *
                       sin(joint3_angle_offest)) /
                          200;

  double terminal_Y = (11 * cos(joint2_angle) * sin(joint1_angle_offest)) / 50 +
                      (11 * cos(joint2_angle) * cos(joint3_angle_offest) *
                       sin(joint1_angle_offest)) /
                          200 -
                      (2969 * cos(joint2_angle) * sin(joint1_angle_offest) *
                       sin(joint3_angle_offest)) /
                          10000 +
                      (2969 * cos(joint3_angle_offest) *
                       sin(joint1_angle_offest) * sin(joint2_angle)) /
                          10000 +
                      (11 * sin(joint1_angle_offest) * sin(joint2_angle) *
                       sin(joint3_angle_offest)) /
                          200;

  double terminal_Z =
      (11 * sin(joint2_angle)) / 50 +
      (11 * cos(joint2_angle) * sin(joint3_angle_offest)) / 200 +
      (11 * cos(joint3_angle_offest) * sin(joint2_angle)) / 200 -
      (2969 * sin(joint2_angle) * sin(joint3_angle_offest)) / 10000 +
      (2969 * cos(joint2_angle) * cos(joint3_angle_offest)) / 10000 + 0.12;

  double R_11 =
      -cos(joint6_angle) *
          (sin(joint5_angle_offest) *
               (cos(joint1_angle_offest) * cos(joint2_angle) *
                    sin(joint3_angle_offest) +
                cos(joint1_angle_offest) * cos(joint3_angle_offest) *
                    sin(joint2_angle)) +
           cos(joint5_angle_offest) *
               (sin(joint1_angle_offest) * sin(joint4_angle_offest) -
                cos(joint4_angle_offest) *
                    (cos(joint1_angle_offest) * cos(joint2_angle) *
                         cos(joint3_angle_offest) -
                     cos(joint1_angle_offest) * sin(joint2_angle) *
                         sin(joint3_angle_offest)))) -
      sin(joint6_angle) * (cos(joint4_angle_offest) * sin(joint1_angle_offest) +
                           sin(joint4_angle_offest) *
                               (cos(joint1_angle_offest) * cos(joint2_angle) *
                                    cos(joint3_angle_offest) -
                                cos(joint1_angle_offest) * sin(joint2_angle) *
                                    sin(joint3_angle_offest)));

  double R_12 =
      sin(joint6_angle) *
          (sin(joint5_angle_offest) *
               (cos(joint1_angle_offest) * cos(joint2_angle) *
                    sin(joint3_angle_offest) +
                cos(joint1_angle_offest) * cos(joint3_angle_offest) *
                    sin(joint2_angle)) +
           cos(joint5_angle_offest) *
               (sin(joint1_angle_offest) * sin(joint4_angle_offest) -
                cos(joint4_angle_offest) *
                    (cos(joint1_angle_offest) * cos(joint2_angle) *
                         cos(joint3_angle_offest) -
                     cos(joint1_angle_offest) * sin(joint2_angle) *
                         sin(joint3_angle_offest)))) -
      cos(joint6_angle) * (cos(joint4_angle_offest) * sin(joint1_angle_offest) +
                           sin(joint4_angle_offest) *
                               (cos(joint1_angle_offest) * cos(joint2_angle) *
                                    cos(joint3_angle_offest) -
                                cos(joint1_angle_offest) * sin(joint2_angle) *
                                    sin(joint3_angle_offest)));

  double R_13 = sin(joint5_angle_offest) *
                    (sin(joint1_angle_offest) * sin(joint4_angle_offest) -
                     cos(joint4_angle_offest) *
                         (cos(joint1_angle_offest) * cos(joint2_angle) *
                              cos(joint3_angle_offest) -
                          cos(joint1_angle_offest) * sin(joint2_angle) *
                              sin(joint3_angle_offest))) -
                cos(joint5_angle_offest) *
                    (cos(joint1_angle_offest) * cos(joint2_angle) *
                         sin(joint3_angle_offest) +
                     cos(joint1_angle_offest) * cos(joint3_angle_offest) *
                         sin(joint2_angle));
  double R_21 = -sin(joint6_angle) *
                    (sin(joint4_angle_offest) *
                         (cos(joint2_angle) * cos(joint3_angle_offest) *
                              sin(joint1_angle_offest) -
                          sin(joint1_angle_offest) * sin(joint2_angle) *
                              sin(joint3_angle_offest)) -
                     cos(joint1_angle_offest) * cos(joint4_angle_offest)) -
                cos(joint6_angle) *
                    (sin(joint5_angle_offest) *
                         (cos(joint2_angle) * sin(joint1_angle_offest) *
                              sin(joint3_angle_offest) +
                          cos(joint3_angle_offest) * sin(joint1_angle_offest) *
                              sin(joint2_angle)) -
                     cos(joint5_angle_offest) *
                         (cos(joint1_angle_offest) * sin(joint4_angle_offest) +
                          cos(joint4_angle_offest) *
                              (cos(joint2_angle) * cos(joint3_angle_offest) *
                                   sin(joint1_angle_offest) -
                               sin(joint1_angle_offest) * sin(joint2_angle) *
                                   sin(joint3_angle_offest))));
  double R_22 =
      sin(joint6_angle) *
          (sin(joint5_angle_offest) *
               (cos(joint2_angle) * sin(joint1_angle_offest) *
                    sin(joint3_angle_offest) +
                cos(joint3_angle_offest) * sin(joint1_angle_offest) *
                    sin(joint2_angle)) -
           cos(joint5_angle_offest) *
               (cos(joint1_angle_offest) * sin(joint4_angle_offest) +
                cos(joint4_angle_offest) *
                    (cos(joint2_angle) * cos(joint3_angle_offest) *
                         sin(joint1_angle_offest) -
                     sin(joint1_angle_offest) * sin(joint2_angle) *
                         sin(joint3_angle_offest)))) -
      cos(joint6_angle) * (sin(joint4_angle_offest) *
                               (cos(joint2_angle) * cos(joint3_angle_offest) *
                                    sin(joint1_angle_offest) -
                                sin(joint1_angle_offest) * sin(joint2_angle) *
                                    sin(joint3_angle_offest)) -
                           cos(joint1_angle_offest) * cos(joint4_angle_offest));
  double R_23 = -sin(joint5_angle_offest) *
                    (cos(joint1_angle_offest) * sin(joint4_angle_offest) +
                     cos(joint4_angle_offest) *
                         (cos(joint2_angle) * cos(joint3_angle_offest) *
                              sin(joint1_angle_offest) -
                          sin(joint1_angle_offest) * sin(joint2_angle) *
                              sin(joint3_angle_offest))) -
                cos(joint5_angle_offest) *
                    (cos(joint2_angle) * sin(joint1_angle_offest) *
                         sin(joint3_angle_offest) +
                     cos(joint3_angle_offest) * sin(joint1_angle_offest) *
                         sin(joint2_angle));

  double R_31 = -cos(joint6_angle) *
                    (sin(joint5_angle_offest) *
                         (sin(joint2_angle) * sin(joint3_angle_offest) -
                          cos(joint2_angle) * cos(joint3_angle_offest)) -
                     cos(joint4_angle_offest) * cos(joint5_angle_offest) *
                         (cos(joint2_angle) * sin(joint3_angle_offest) +
                          cos(joint3_angle_offest) * sin(joint2_angle))) -
                sin(joint4_angle_offest) * sin(joint6_angle) *
                    (cos(joint2_angle) * sin(joint3_angle_offest) +
                     cos(joint3_angle_offest) * sin(joint2_angle));
  double R_32 =
      sin(joint6_angle) * (sin(joint5_angle_offest) *
                               (sin(joint2_angle) * sin(joint3_angle_offest) -
                                cos(joint2_angle) * cos(joint3_angle_offest)) -
                           cos(joint4_angle_offest) * cos(joint5_angle_offest) *
                               (cos(joint2_angle) * sin(joint3_angle_offest) +
                                cos(joint3_angle_offest) * sin(joint2_angle))) -
      cos(joint6_angle) * sin(joint4_angle_offest) *
          (cos(joint2_angle) * sin(joint3_angle_offest) +
           cos(joint3_angle_offest) * sin(joint2_angle));
  double R_33 = -cos(joint5_angle_offest) *
                    (sin(joint2_angle) * sin(joint3_angle_offest) -
                     cos(joint2_angle) * cos(joint3_angle_offest)) -
                cos(joint4_angle_offest) * sin(joint5_angle_offest) *
                    (cos(joint2_angle) * sin(joint3_angle_offest) +
                     cos(joint3_angle_offest) * sin(joint2_angle));

  double terminal_YAW, terminal_PITCH, terminal_ROLL;

                      
  if(fabs(R_13 ) > 0.99f)
  {
    
    terminal_PITCH = atan(1);
   if(R_13 > 0.0f){
      terminal_ROLL = atan2(R_32, R_22);
    } else {
      terminal_ROLL = -atan2(R_21, R_31);
    }
    terminal_YAW = 0.0f;
  }else {
    // 处理非万向锁问题
    terminal_YAW = -atan2(R_12, R_11);
    terminal_ROLL = -atan2(R_23, R_33);
    terminal_PITCH = atan(R_13 * cos(terminal_YAW) / R_11);
  }

  // if (fabs(fabs(R_31) - 1.0) < 0.002) {
  //   // 万向锁情况：pitch接近 ±90°
  //   terminal_YAW = 0.0; // 可以任意选择，通常设为0

  //   if (R_31 < 0) {
  //     // pitch = 90°
  //     terminal_PITCH = M_PI / 2.0;
  //     terminal_ROLL = atan2(R_12, R_22);
  //   } else {
  //     // pitch = -90°
  //     terminal_PITCH = -M_PI / 2.0;
  //     terminal_ROLL = -atan2(R_12, R_22);
  //   }
  // } else {
  //   // 非万向锁情况
  //   terminal_YAW = atan2(R_21, R_11);  // ψ (yaw)
  //   terminal_PITCH = -asin(R_31);      // θ (pitch)，注意负号！
  //   terminal_ROLL = atan2(R_32, R_33); // φ (roll)
  // }

  terminal_position.terminal_X = terminal_X;
  terminal_position.terminal_Y = terminal_Y;
  terminal_position.terminal_Z = terminal_Z;
  terminal_position.terminal_YAW = terminal_YAW;
  terminal_position.terminal_PITCH = terminal_PITCH;
  terminal_position.terminal_ROLL = terminal_ROLL;

  return terminal_position;
}

rotation_matrix rotation_matrix_calc(Arm_terminal terminal_position) {
  rotation_matrix R;

  R.R_11 = cos(terminal_position.terminal_YAW) *
           cos(terminal_position.terminal_PITCH);
  R.R_12 = cos(terminal_position.terminal_YAW) *
               sin(terminal_position.terminal_PITCH) *
               sin(terminal_position.terminal_ROLL) -
           sin(terminal_position.terminal_YAW) *
               cos(terminal_position.terminal_ROLL);
  R.R_13 = cos(terminal_position.terminal_YAW) *
               sin(terminal_position.terminal_PITCH) *
               cos(terminal_position.terminal_ROLL) +
           sin(terminal_position.terminal_YAW) *
               sin(terminal_position.terminal_ROLL);

  R.R_21 = sin(terminal_position.terminal_YAW) *
           cos(terminal_position.terminal_PITCH);
  R.R_22 = sin(terminal_position.terminal_YAW) *
               sin(terminal_position.terminal_PITCH) *
               sin(terminal_position.terminal_ROLL) +
           cos(terminal_position.terminal_YAW) *
               cos(terminal_position.terminal_ROLL);
  R.R_23 = sin(terminal_position.terminal_YAW) *
               sin(terminal_position.terminal_PITCH) *
               cos(terminal_position.terminal_ROLL) -
           cos(terminal_position.terminal_YAW) *
               sin(terminal_position.terminal_ROLL);

  R.R_31 = -sin(terminal_position.terminal_PITCH);
  R.R_32 = cos(terminal_position.terminal_PITCH) *
           sin(terminal_position.terminal_ROLL);
  R.R_33 = cos(terminal_position.terminal_PITCH) *
           cos(terminal_position.terminal_ROLL);

  return R;
}
// Yaw (ψ): 绕Z轴旋转
// Pitch (θ): 绕Y轴旋转
// Roll (φ): 绕X轴旋转

// R = [cosψ*cosθ,  cosψ*sinθ*sinφ - sinψ*cosφ,  cosψ*sinθ*cosφ + sinψ*sinφ;
//      sinψ*cosθ,  sinψ*sinθ*sinφ + cosψ*cosφ,  sinψ*sinθ*cosφ - cosψ*sinφ;
//      -sinθ,      cosθ*sinφ,                   cosθ*cosφ]

// R = [cos(terminal_YAW)*cos(terminal_PITCH),
// cos(terminal_YAW)*sin(terminal_PITCH)*sin(terminal_ROLL) -
// sin(terminal_YAW)*cos(terminal_ROLL),
// cos(terminal_YAW)*sin(terminal_PITCH)*cos(terminal_ROLL) +
// sin(terminal_YAW)*sin(terminal_ROLL);

//      sin(terminal_YAW)*cos(terminal_PITCH),
//      sin(terminal_YAW)*sin(terminal_PITCH)*sin(terminal_ROLL) +
//      cos(terminal_YAW)*cos(terminal_ROLL),
//      sin(terminal_YAW)*sin(terminal_PITCH)*cos(terminal_ROLL) -
//      cos(terminal_YAW)*sin(terminal_ROLL);

// -sin(terminal_PITCH),
//      cos(terminal_PITCH)*sin(terminal_ROLL),
//      cos(terminal_PITCH)*cos(terminal_ROLL)]

/**
 * @brief  机械臂逆运动学计算函数
 * @note   根据末端的位姿（位置和姿态）计算各关节角度
 *         本函数目前是一个框架，需要根据实际的机械臂连杆参数完成具体实现
 * @param  target_terminal: 期望的末端位置(X,Y,Z)和姿态(YAW,PITCH,ROLL)的结构体
 * @retval Arm_motor_angle: 包含各关节角度的结构体，单位：弧度
 */
Arm_motor_angle inverse_kinematics(Arm_terminal target_terminal) {
  Arm_motor_angle motor_angle;
  R_euler_angle = rotation_matrix_calc(target_terminal);

  motor_angle.Joint1_angle =
      atan2(target_terminal.terminal_Y - R_euler_angle.R_23,
            target_terminal.terminal_X - R_euler_angle.R_13);

  motor_angle.Joint2_angle = 0.0f;
  motor_angle.Joint3_angle = 0.0f;

  motor_angle.Joint5_angle =
      atan2(R_euler_angle.R_13, sqrt(R_euler_angle.R_11 * R_euler_angle.R_11 +
                                     R_euler_angle.R_12 * R_euler_angle.R_12));

  motor_angle.Joint4_angle =
      atan2(-R_euler_angle.R_23 / cos(motor_angle.Joint5_angle),
            -R_euler_angle.R_33 / cos(motor_angle.Joint5_angle));

  motor_angle.Joint6_angle =
      atan2(-R_euler_angle.R_12 / cos(motor_angle.Joint5_angle),
            R_euler_angle.R_11 / cos(motor_angle.Joint5_angle));

  return motor_angle;
}

// 线性位置插值
void linear_interpolate(const double start[3], const double end[3], double t,
                        double result[3]) {
  for (int i = 0; i < 3; i++) {
    result[i] = start[i] + t * (end[i] - start[i]);
  }
}

// void calculatr_error
float limit(float value, float min, float max) {
  if (value < min) {
    return min;
  } else if (value > max) {
    return max;
  } else {
    return value;
  }
}