#include "bsp_can.h"
#include "main.h"

#include "DM_motor.h"
#include "DJI_motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern s_DMmotor_data_t DMmotor_4340[3];
extern s_DMmotor_data_t DMmotor_4310[3];
extern s_Dji_motor_data_t DJI_motor_3508;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header_can1, rx_header_can2;
    uint8_t rx_data_can1[8];
    uint8_t rx_data_can2[8];
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can1, rx_data_can1);

        switch (rx_header_can1.StdId)
        {
        
        case DM_4340_1 + 0x10:
        {
            DMmotor_data_recive(&DMmotor_4340[0], rx_data_can1);
            break;
        }
        case DM_4340_2 + 0x10:
        {
            DMmotor_data_recive(&DMmotor_4340[1], rx_data_can1);
            break;
        }
        case DM_4340_3 + 0x10:
        {
            DMmotor_data_recive(&DMmotor_4340[2], rx_data_can1);
            break;
        }
        case DM_4310_1 + 0x10:
        {
            DMmotor_data_recive(&DMmotor_4310[0], rx_data_can1);
            break;
        }
        case DM_4310_2 + 0x10:
        {
            DMmotor_data_recive(&DMmotor_4310[1], rx_data_can1);
            break;
        }
        case DM_4310_3 + 0x10:
        {
            DMmotor_data_recive(&DMmotor_4310[2], rx_data_can1);
            break;
        }
        case DJI_motor_3508_ID_1:
        {
            DJI_motor_3508_recevie(&DJI_motor_3508, rx_data_can1);
            break;
        }
        default:
        {
            break;
        }
        }
    }
    ////////CAN2接收数据处理//////////
    /*            */
    if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header_can2, rx_data_can2); // 从FIFO中接收消息至rx_header_can2
        switch (rx_header_can2.StdId)
        {

       
            // case DM8006_M1:
            // {
            //     DM8006_Date[0].id = (rx_data_can2[0]) & 0x0F;
            //     DM_CanReceive(&DM8006_Date[0], rx_data_can2);

            //     DM8006_Date[0].esc_back_position_last = DM8006_Date[0].esc_back_position;
            //     DM8006_Date[0].real_angle = (DM8006_Date[0].esc_back_position * 57.29577951308f)  + 35.0f;
            //     //这里加上35度偏移量，是加上机械方面的限制最靠下的倾角是35
            //     FPS.Pitch_DM8006++;
            //     break;
            // }

        default:
        {
            break;
        }
        }
    }
}



