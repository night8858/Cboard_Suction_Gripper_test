#include "DM_motor.h"
#include "main.h"
#include "math.h"
#include <sys/_intsup.h>

static CAN_TxHeaderTypeDef CAN_DMstart_TxHeader;
static CAN_TxHeaderTypeDef CAN_motor_msg_TxHeader;

/**
 * @brief DM电机初始化配置函数
 * @param motor_data: 电机数据结构体指针，用于存储电机的所有配置信息和状态
 * @param id: 电机的CAN通信ID（主机ID）
 * @param hcan: CAN外设句柄指针，用于配置电机使用的CAN接口
 * @param motor_type:
 * 电机类型，可选择DM4310或DM4340等，定义在DM_motor_type枚举中
 * @param control_type:
 * 控制类型，可选择PID控制或MIT控制，定义在DM_motor_contorl_type枚举中
 * @retval None
 * @note 从机ID会自动设置为主机ID+0x10，初始控制模式设置为0（位置环）
 */
void DM_motor_config(s_DMmotor_data_t *motor_data, int id,
                     CAN_HandleTypeDef *hcan, int motor_type,
                     int control_type) {
  motor_data->state = 0;                   // 初始化电机状态为0（未使能）
  motor_data->hcan = *hcan;                // 复制CAN句柄配置
  motor_data->CAN_id = id;                 // 设置电机的主机ID
  motor_data->master_id = id + 0x10;       // 主机ID = 从机ID + 0x10
  motor_data->control_mode = 0;            // 初始控制模式设为0（位置环控制）
  motor_data->motor_type = motor_type;     // 设置电机类型
  motor_data->control_type = control_type; // 设置控制类型
}

void DM_motor_PID_init(s_DMmotor_data_t *motor_data, float kp, float ki,
                       float kd, float errlim, float max_output, int PID_Type) {
  if (PID_Type == PID_POS) {
    pid_abs_param_init(&motor_data->position_pid, kp, ki, kd, errlim,
                       max_output);
    return;
  } else if (PID_Type == PID_SPD) {
    pid_abs_param_init(&motor_data->speed_pid, kp, ki, kd, errlim, max_output);
    return;
  }
}
// 浮点数转整形,同时限制输入范围
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
  float span = x_max - x_min;
  if (x < x_min)
    x = x_min;
  else if (x > x_max)
    x = x_max;
  return (int)((x - x_min) * ((float)((1 << bits) / span)));
}

// 整形转浮点数
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void DM_motor_Enable(s_DMmotor_data_t *motor_data) {
  // uint32_t send_mail_box;
  uint8_t TxData[8];
  CAN_DMstart_TxHeader.StdId = motor_data->CAN_id;
  CAN_DMstart_TxHeader.IDE = CAN_ID_STD;
  CAN_DMstart_TxHeader.RTR = CAN_RTR_DATA;
  CAN_DMstart_TxHeader.DLC = 0x08;
  TxData[0] = 0xFF;
  TxData[1] = 0xFF;
  TxData[2] = 0xFF;
  TxData[3] = 0xFF;
  TxData[4] = 0xFF;
  TxData[5] = 0xFF;
  TxData[6] = 0xFF;
  TxData[7] = 0xFC;

  HAL_CAN_AddTxMessage(&motor_data->hcan, &CAN_DMstart_TxHeader, TxData,
                       (uint32_t *)CAN_TX_MAILBOX0);
}

void DM_motor_Disable(s_DMmotor_data_t *motor_data) {
  // uint32_t send_mail_box;
  uint8_t TxData[8];
  CAN_DMstart_TxHeader.StdId = motor_data->CAN_id;
  CAN_DMstart_TxHeader.IDE = CAN_ID_STD;
  CAN_DMstart_TxHeader.RTR = CAN_RTR_DATA;
  CAN_DMstart_TxHeader.DLC = 0x08;
  TxData[0] = 0xFF;
  TxData[1] = 0xFF;
  TxData[2] = 0xFF;
  TxData[3] = 0xFF;
  TxData[4] = 0xFF;
  TxData[5] = 0xFF;
  TxData[6] = 0xFF;
  TxData[7] = 0xFD;

  HAL_CAN_AddTxMessage(&motor_data->hcan, &CAN_DMstart_TxHeader, TxData,
                       (uint32_t *)CAN_TX_MAILBOX0);
}

void DM_motor_setZero(s_DMmotor_data_t *motor_data) {
  // uint32_t send_mail_box;
  uint8_t TxData[8];
  CAN_DMstart_TxHeader.StdId = motor_data->CAN_id;
  CAN_DMstart_TxHeader.IDE = CAN_ID_STD;
  CAN_DMstart_TxHeader.RTR = CAN_RTR_DATA;
  CAN_DMstart_TxHeader.DLC = 0x08;
  TxData[0] = 0xFF;
  TxData[1] = 0xFF;
  TxData[2] = 0xFF;
  TxData[3] = 0xFF;
  TxData[4] = 0xFF;
  TxData[5] = 0xFF;
  TxData[6] = 0xFF;
  TxData[7] = 0xFE;

  HAL_CAN_AddTxMessage(&motor_data->hcan, &CAN_DMstart_TxHeader, TxData,
                       (uint32_t *)CAN_TX_MAILBOX0);
}

void DM_motor_control(s_DMmotor_data_t *traget_motor) {
  if (traget_motor->motor_type == DM4310) {
    MD4310_motor_Control(traget_motor);
  } else if (traget_motor->motor_type == DM4340) {
    MD4340_motor_Control(traget_motor);
  }
}

void DM_MITvlue_set(s_DMmotor_data_t *traget_motor, float p, float v, float kp, float kd, float t) {
  traget_motor->f_p = p;
  traget_motor->f_v = v;
  traget_motor->f_kp = kp;
  traget_motor->f_kd = kd;
  traget_motor->f_t = t;
}

void MD4310_motor_Control(s_DMmotor_data_t *traget_motor) {
  uint8_t txData[8];
  if (traget_motor->motor_type != DM4310)
    return;

  CAN_motor_msg_TxHeader.StdId = traget_motor->CAN_id;
  CAN_motor_msg_TxHeader.IDE = CAN_ID_STD;
  CAN_motor_msg_TxHeader.RTR = CAN_RTR_DATA;
  CAN_motor_msg_TxHeader.DLC = 0x08;
  if (traget_motor->control_type == MIT_control) {

    traget_motor->p_int =
        float_to_uint(traget_motor->f_p, DM4310_P_MIN, DM4310_P_MAX, 16);
    traget_motor->v_int =
        float_to_uint(traget_motor->f_v, DM4310_V_MIN, DM4310_V_MAX, 12);
    traget_motor->kp_int =
        float_to_uint(traget_motor->f_kp, DM4310_KP_MIN, DM4310_KP_MAX, 12);
    traget_motor->kd_int =
        float_to_uint(traget_motor->f_kd, DM4310_KD_MIN, DM4310_KD_MAX, 12);
    traget_motor->t_int =
        float_to_uint(traget_motor->f_t, DM4310_T_MIN, DM4310_T_MAX, 12);

    txData[0] = traget_motor->p_int >> 8;
    txData[1] = traget_motor->p_int & 0xFF;
    txData[2] = traget_motor->v_int >> 4;
    txData[3] =
        ((traget_motor->v_int & 0xF) << 4) | (traget_motor->kp_int >> 8);
    txData[4] = traget_motor->kp_int & 0xFF;
    txData[5] = traget_motor->kd_int >> 4;
    txData[6] = ((traget_motor->kd_int) << 4) | (traget_motor->t_int >> 8);
    txData[7] = traget_motor->t_int & 0xff;
  } else if (traget_motor->control_type == PID_control) {

    traget_motor->target_position =
        fminf(fmaxf(DM4310_P_MIN, traget_motor->target_position), DM4310_P_MAX);
    traget_motor->f_p = traget_motor->target_position;
    traget_motor->f_v = traget_motor->target_speed;
    traget_motor->f_t = traget_motor->target_out_current;

    traget_motor->p_int =
        float_to_uint(traget_motor->f_p, DM4310_P_MIN, DM4310_P_MAX, 16);
    traget_motor->v_int =
        float_to_uint(traget_motor->f_v, DM4310_V_MIN, DM4310_V_MAX, 12);
    traget_motor->kp_int =
        float_to_uint(traget_motor->f_kp, DM4310_KP_MIN, DM4310_KP_MAX, 12);
    traget_motor->kd_int =
        float_to_uint(traget_motor->f_kd, DM4310_KD_MIN, DM4310_KD_MAX, 12);
    traget_motor->t_int =
        float_to_uint(traget_motor->f_t, DM4310_T_MIN, DM4310_T_MAX, 12);

    txData[0] = 0;
    txData[1] = 0;
    txData[2] = 0;
    txData[3] = 0;
    txData[4] = 0;
    txData[5] = 0;
    txData[6] = ((0 & 0xf) << 4) | (traget_motor->t_int >> 8);
    txData[7] = traget_motor->t_int & 0xff;
  }

  HAL_CAN_AddTxMessage(&traget_motor->hcan, &CAN_motor_msg_TxHeader, txData,
                       (uint32_t *)CAN_TX_MAILBOX0);
}

void MD4340_motor_Control(s_DMmotor_data_t *traget_motor) {
  uint8_t txData[8];
  if (traget_motor->motor_type != DM4340)
    return;

  CAN_motor_msg_TxHeader.StdId = traget_motor->CAN_id;
  CAN_motor_msg_TxHeader.IDE = CAN_ID_STD;
  CAN_motor_msg_TxHeader.RTR = CAN_RTR_DATA;
  CAN_motor_msg_TxHeader.DLC = 0x08;

  if (traget_motor->control_type == MIT_control) {

    traget_motor->p_int =
        float_to_uint(traget_motor->f_p, DM4340_P_MIN, DM4340_P_MAX, 16);
    traget_motor->v_int =
        float_to_uint(traget_motor->f_v, DM4340_V_MIN, DM4340_V_MAX, 12);
    traget_motor->kp_int =
        float_to_uint(traget_motor->f_kp, DM4340_KP_MIN, DM4340_KP_MAX, 12);
    traget_motor->kd_int =
        float_to_uint(traget_motor->f_kd, DM4340_KD_MIN, DM4340_KD_MAX, 12);
    traget_motor->t_int =
        float_to_uint(traget_motor->f_t, DM4340_T_MIN, DM4340_T_MAX, 12);

    txData[0] = traget_motor->p_int >> 8;
    txData[1] = traget_motor->p_int & 0xFF;
    txData[2] = traget_motor->v_int >> 4;
    txData[3] =
        ((traget_motor->v_int & 0xF) << 4) | (traget_motor->kp_int >> 8);
    txData[4] = traget_motor->kp_int & 0xFF;
    txData[5] = traget_motor->kd_int >> 4;
    txData[6] = ((traget_motor->kd_int) << 4) | (traget_motor->t_int >> 8);
    txData[7] = traget_motor->t_int & 0xff;
  } else if (traget_motor->control_type == PID_control) {

    traget_motor->target_position =
        fminf(fmaxf(DM4340_P_MIN, traget_motor->target_position), DM4340_P_MAX);
    traget_motor->f_p = traget_motor->target_position;
    traget_motor->f_v = traget_motor->target_speed;
    traget_motor->f_t = traget_motor->target_out_current;

    traget_motor->p_int =
        float_to_uint(traget_motor->f_p, DM4340_P_MIN, DM4340_P_MAX, 16);
    traget_motor->v_int =
        float_to_uint(traget_motor->f_v, DM4340_V_MIN, DM4340_V_MAX, 12);
    traget_motor->kp_int =
        float_to_uint(traget_motor->f_kp, DM4340_KP_MIN, DM4340_KP_MAX, 12);
    traget_motor->kd_int =
        float_to_uint(traget_motor->f_kd, DM4340_KD_MIN, DM4340_KD_MAX, 12);
    traget_motor->t_int =
        float_to_uint(traget_motor->f_t, DM4340_T_MIN, DM4340_T_MAX, 12);

    txData[0] = 0;
    txData[1] = 0;
    txData[2] = 0;
    txData[3] = 0;
    txData[4] = 0;
    txData[5] = 0;
    txData[6] = ((0 & 0xf) << 4) | (traget_motor->t_int >> 8);
    txData[7] = traget_motor->t_int & 0xff;
  }
  HAL_CAN_AddTxMessage(&traget_motor->hcan, &CAN_motor_msg_TxHeader, txData,
                       (uint32_t *)CAN_TX_MAILBOX0);
}

void DMmotor_data_recive(s_DMmotor_data_t *motor_data, uint8_t *RxDate) {
  // 处理接收到的数据
  int ID = RxDate[0] & 0xf;
  int p_int = (RxDate[1] << 8) | RxDate[2];
  int v_int = (RxDate[3] << 4) | (RxDate[4] >> 4);
  int i_int = ((RxDate[4] & 0xf) << 8) | (RxDate[5]);
  int T_int = RxDate[6];

  if (motor_data->motor_type == DM4340) {
    if (motor_data->CAN_id != ID)
      return;
    motor_data->state = (RxDate[0]) >> 4;
    motor_data->esc_back_position =
        uint_to_float(p_int, DM4340_P_MIN, DM4340_P_MAX, 16); // 电机位置
    motor_data->esc_back_speed =
        uint_to_float(v_int, DM4340_V_MIN, DM4340_V_MAX, 12);
    motor_data->esc_back_current = uint_to_float(
        i_int, DM4340_T_MIN, DM4340_T_MAX, 12); //	电机扭矩/电流
    motor_data->Tmos = (float)(T_int);
    motor_data->Tcoil = (float)(RxDate[7]);
  } else if (motor_data->motor_type == DM4310) {
    if (motor_data->CAN_id != ID)
      return;
    motor_data->state = (RxDate[0]) >> 4;
    motor_data->esc_back_position =
        uint_to_float(p_int, DM4310_P_MIN, DM4310_P_MAX, 16); // 电机位置
    motor_data->esc_back_speed =
        uint_to_float(v_int, DM4310_V_MIN, DM4310_V_MAX, 12);
    motor_data->esc_back_current = uint_to_float(
        i_int, DM4310_T_MIN, DM4310_T_MAX, 12); //	电机扭矩/电流
    motor_data->Tmos = (float)(T_int);
    motor_data->Tcoil = (float)(RxDate[7]);
  }
}