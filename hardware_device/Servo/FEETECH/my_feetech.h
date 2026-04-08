#ifndef MY_FEETECH_H
#define MY_FEETECH_H

#include <stdbool.h>
#include <stdint.h>

#include "usart.h"

#define FTSTS_MAX_PARAM_LEN 16U

enum FTSTS_servo_address {
    current_position = 0x38,
    current_speed = 0x3A,
    current_load = 0x3C,
};

typedef struct {
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t parameter_len;
    uint8_t parameter[FTSTS_MAX_PARAM_LEN];
    uint8_t checksum;
    uint32_t timestamp_ms;
} ftsts_rx_frame_t;

typedef struct
{
    uint8_t id;
    float angle;
    float speed;
    int back_position;
    int back_speed;
    
}FTservo_data_t;

void FTSTS_dma_rx_init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef FTSTS_dma_rx_start(void);
void FTSTS_dma_rx_on_event(UART_HandleTypeDef *huart, uint16_t size);
void FTSTS_read_data(UART_HandleTypeDef *huart, uint8_t id, uint8_t addr, uint8_t data_len);
bool FTSTS_get_last_frame(uint8_t id, ftsts_rx_frame_t *out_frame);
int servo_get_position(UART_HandleTypeDef *huart, int id);

void FTSTS_setID(int id);
void FTSTS_clear_writelock(int id);
void FTSTS_setMAX_F(int id);
void FTSTS_servo_write_pos(UART_HandleTypeDef *huart, uint8_t id, float angle, float speed);
void hand_test(float angle);
void finger_test(float angle, uint8_t id);

#endif
