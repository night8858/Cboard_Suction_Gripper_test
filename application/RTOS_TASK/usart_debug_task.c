#include "DJI_motor.h"
#include "main.h"

#include "cmsis_os.h"

#include "DM_motor.h"
#include "arm_control.h"
#include "bsp_usart.h"
#include "variables.h"
#include "DT7.h"

extern s_DMmotor_data_t DMmotor_4340[3];
extern s_DMmotor_data_t DMmotor_4310[3];
extern s_Dji_motor_data_t DJI_motor_3508;
//extern Arm_terminal real_arm_terminal;
extern Arm_motor_angle real_motor_angle;
extern Arm_motor_angle target_motor_angle;
extern RC_ctrl_t rc_ctrl;
extern Planar_Robot_Arm Arm_LF;       //左前
extern Planar_Robot_Arm Arm_RF;       //右前
extern Planar_Robot_Arm Arm_LB;       //左后
extern Planar_Robot_Arm Arm_RB;       //右后

void usartr_debug_task(void const *argument) 
{
  osDelay(1600);
  // usart_cmd_rx_init(&huart6);
  // (void)usart_cmd_rx_start();
  /* USER CODE BEGIN usart_debug_task */
  /* Infinite loop */

  for (;;) {
    // uart_dma_printf(
    //     &huart6, "%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\n",
    //     real_motor_angle.Joint1_angle, real_motor_angle.Joint2_angle,
    //     real_motor_angle.Joint3_angle, real_motor_angle.Joint4_angle,
    //     real_motor_angle.Joint5_angle, real_motor_angle.Joint6_angle);
    // osDelay(3);
    //  uart_dma_printf(
    //     &huart6, "%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\n",
    //         real_arm_terminal.terminal_X, real_arm_terminal.terminal_Y,
    //         real_arm_terminal.terminal_Z, real_arm_terminal.terminal_YAW,
    //         real_arm_terminal.terminal_PITCH,
    //         real_arm_terminal.terminal_ROLL);
    //  uart_dma_printf(
    //     &huart6, "%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\n",
    //         real_arm_terminal.terminal_X, real_arm_terminal.terminal_Y,
    //         real_arm_terminal.terminal_Z, real_arm_terminal.terminal_YAW,
    //         real_arm_terminal.terminal_PITCH,
    //         real_arm_terminal.terminal_ROLL);
    // uart_dma_printf(
    //     &huart6,
    //     "%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\n",
    //     real_motor_angle.Joint1_angle, real_motor_angle.Joint2_angle,
    //     real_motor_angle.Joint3_angle, real_motor_angle.Joint4_angle,
    //     real_motor_angle.Joint5_angle, real_motor_angle.Joint6_angle,
    //     real_arm_terminal.terminal_X, real_arm_terminal.terminal_Y,
    //     real_arm_terminal.terminal_Z, real_arm_terminal.terminal_YAW,
    //     real_arm_terminal.terminal_PITCH, real_arm_terminal.terminal_ROLL);

    // PID
    // uart_dma_printf(&huart6, "%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\n", 
    //     DMmotor_4340[2].target_position, DMmotor_4340[2].esc_back_position,
    //     DMmotor_4340[1].target_position, DMmotor_4340[1].esc_back_position,
    //     DMmotor_4340[0].target_position, DMmotor_4340[0].esc_back_position);
    // uart_dma_printf(
    //     &huart6,
    //     "%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\n",
    //     DMmotor_4340[0].esc_back_position, DMmotor_4340[1].esc_back_position,
    //     DMmotor_4340[1].esc_back_position, DMmotor_4310[0].esc_back_position,
    //     DMmotor_4310[1].esc_back_position, DMmotor_4310[2].esc_back_position,
    //     real_arm_terminal.terminal_X, real_arm_terminal.terminal_Y,
    //     real_arm_terminal.terminal_Z, real_arm_terminal.terminal_YAW,
    //     real_arm_terminal.terminal_PITCH, real_arm_terminal.terminal_ROLL);
    //   heartbeat_kick(HB_TASK_DEBUG, HAL_GetTick());
      // uart_dma_printf(&huart6, "%f,%f,%f,%f,%f,%f\n", 
      //                 Arm_RB.current_joint_angles[0], 
      //                 Arm_RB.current_joint_angles[1],
      //                 Arm_RB.end_effector_x,
      //                 Arm_RB.end_effector_y,
      //                 Arm_RB.target_joint_angles[0],
      //                 Arm_RB.target_joint_angles[1]);

      //测试末端位姿输出  
        // uart_dma_printf(&huart6, "%f,%f,%f,%f,%f,%f,%f,%f\n", 
        //                 Arm_LF.end_effector_x, 
        //                 Arm_LF.end_effector_y,
        //                 Arm_RF.end_effector_x,
        //                 Arm_RF.end_effector_y,
        //                 Arm_LB.end_effector_x,
        //                 Arm_LB.end_effector_y,
        //                 Arm_RB.end_effector_x,
        //                 Arm_RB.end_effector_y);

                // uart_dma_printf(&huart6, "%f,%f,%f,%f,%d,%d,%f,%f,%d,%d\n", 
                //         Arm_LF.end_effector_x, 
                //         Arm_LF.end_effector_y,
                //         Arm_LF.target_joint_angles[0],
                //         Arm_LF.target_joint_angles[1],
                //         Arm_LF.target_servo_positions[0],
                //         Arm_LF.target_servo_positions[1],
                //         Arm_LF.current_joint_angles[0],
                //         Arm_LF.current_joint_angles[1],
                //         Arm_LF.current_servo_positions[0],
                //         Arm_LF.current_servo_positions[1]);

                // uart_dma_printf(&huart6, "%f,%f,%f,%f,%d,%d,%f,%f,%d,%d\n", 
                //         Arm_LB.end_effector_x, 
                //         Arm_LB.end_effector_y,
                //         Arm_LB.target_joint_angles[0],
                //         Arm_LB.target_joint_angles[1],
                //         Arm_LB.target_servo_positions[0],
                //         Arm_LB.target_servo_positions[1],
                //         Arm_LB.current_joint_angles[0],
                //         Arm_LB.current_joint_angles[1],
                //         Arm_LB.current_servo_positions[0],
                //         Arm_LB.current_servo_positions[1]);

                uart_dma_printf(&huart6, "%f,%f,%f,%f,%d,%d,%f,%f,%d,%d\n", 
                        Arm_RF.end_effector_x, 
                        Arm_RF.end_effector_y,
                        Arm_RF.target_joint_angles[0],
                        Arm_RF.target_joint_angles[1],
                        Arm_RF.target_servo_positions[0],
                        Arm_RF.target_servo_positions[1],
                        Arm_RF.current_joint_angles[0],
                        Arm_RF.current_joint_angles[1],
                        Arm_RF.current_servo_positions[0],
                        Arm_RF.current_servo_positions[1]);
      osDelay(4);
  }
  /* USER CODE END usart_debug_task */
}