#include "my_feetech.h"

#include <stdint.h>
#include <string.h>

#include "cmsis_os.h"
#include "bsp_usart.h"

//指令逻辑
/*
帧头          ID 号   数据长度    指令             参数                         校验和
0XFF 0XFF     ID      Length      Instruction     Parameter1 ...Parameter N    Check Sum

  
Instruction: PING（查询）             0x01      0(长度)
             READ DATA（读）          0x02      2
             WRITE DATA（写）         0x03      N（N>=2，包含地址和数据）
             REGWRITE DATA(异步写)    0x04      N（N>=4，包含地址和数据）


Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N) 若
括号内的计算和超出255, 则取最低的一个字节，“~”表示取反。
*/ 

//应答帧逻辑
/*
帧头          ID 号   数据长度    状态             参数                         校验和
0XFF 0XFF     ID      Length      ERROR     Parameter1 ...Parameter N    Check Sum

返回的应答帧包含舵机的当前状态ERROR，若舵机当前工作状态不正常，会通过这个字节反映出来（各状态所代表意义详见手册内存控制表），若ERROR为0，则舵机无报错信息。
*/

#define FTSTS_DMA_RX_BUF_SIZE 64U
#define FTSTS_STREAM_BUF_SIZE 128U
#define FTSTS_READ_INST 0x02U
#define FTSTS_WRITE_INST 0x03U
#define FTSTS_DEFAULT_TIMEOUT_MS 20U

static UART_HandleTypeDef *s_ft_uart = &huart6;
static uint8_t s_dma_rx_buf[FTSTS_DMA_RX_BUF_SIZE];
static uint8_t s_stream_buf[FTSTS_STREAM_BUF_SIZE];
static uint16_t s_stream_len;
static ftsts_rx_frame_t s_last_frame[256];



//飞特和校验码计算函数
static uint8_t ftsts_checksum(const uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    uint8_t i;

    for (i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(~sum);
}

//读取舵机位置，返回值为-1表示读取失败，非负值表示舵机位置
static void ftsts_cache_frame(uint8_t id, const uint8_t *frame)
{
    ftsts_rx_frame_t *dst = &s_last_frame[id];
    uint8_t length = frame[3];
    uint8_t parameter_len = (length >= 2U) ? (uint8_t)(length - 2U) : 0U;

    if (parameter_len > FTSTS_MAX_PARAM_LEN) {
        parameter_len = FTSTS_MAX_PARAM_LEN;
    }

    dst->id = id;
    dst->length = length;
    dst->error = frame[4];
    dst->parameter_len = parameter_len;
    if (parameter_len > 0U) {
        memcpy(dst->parameter, &frame[5], parameter_len);
    }
    dst->checksum = frame[(uint16_t)length + 3U];
    dst->timestamp_ms = HAL_GetTick();
}

//解析接收数据流，提取完整帧并缓存最新的应答帧
static void ftsts_parse_stream(void)
{
    while (s_stream_len >= 6U) {
        uint16_t i;
        uint8_t id;
        uint8_t length;
        uint16_t frame_len;
        uint8_t checksum;

        for (i = 0; (i + 1U) < s_stream_len; i++) {
            if ((s_stream_buf[i] == 0xFFU) && (s_stream_buf[i + 1U] == 0xFFU)) {
                break;
            }
        }

        if ((i + 1U) >= s_stream_len) {
            s_stream_len = 0;
            return;
        }

        if (i > 0U) {
            memmove(s_stream_buf, &s_stream_buf[i], s_stream_len - i);
            s_stream_len = (uint16_t)(s_stream_len - i);
        }

        if (s_stream_len < 6U) {
            return;
        }

        id = s_stream_buf[2];
        length = s_stream_buf[3];
        if (length < 2U) {
            memmove(s_stream_buf, &s_stream_buf[1], s_stream_len - 1U);
            s_stream_len--;
            continue;
        }

        frame_len = (uint16_t)length + 4U;
        if (frame_len > sizeof(s_stream_buf)) {
            memmove(s_stream_buf, &s_stream_buf[1], s_stream_len - 1U);
            s_stream_len--;
            continue;
        }

        if (s_stream_len < frame_len) {
            return;
        }

        checksum = ftsts_checksum(&s_stream_buf[2], (uint8_t)(length + 1U));
        if (checksum == s_stream_buf[frame_len - 1U]) {
            ftsts_cache_frame(id, s_stream_buf);
            memmove(s_stream_buf, &s_stream_buf[frame_len], s_stream_len - frame_len);
            s_stream_len = (uint16_t)(s_stream_len - frame_len);
        } else {
            memmove(s_stream_buf, &s_stream_buf[1], s_stream_len - 1U);
            s_stream_len--;
        }
    }
}

// 初始化DMA接收
void FTSTS_dma_rx_init(UART_HandleTypeDef *huart)
{
    if (huart != NULL) {
        s_ft_uart = huart;
    }

    s_stream_len = 0U;
    memset(s_dma_rx_buf, 0, sizeof(s_dma_rx_buf));
    memset(s_stream_buf, 0, sizeof(s_stream_buf));
    memset(s_last_frame, 0, sizeof(s_last_frame));
}

// 启动DMA接收
HAL_StatusTypeDef FTSTS_dma_rx_start(void)
{
    HAL_StatusTypeDef ret;

    if (s_ft_uart == NULL) {
        return HAL_ERROR;
    }

    if (s_ft_uart->RxState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortReceive(s_ft_uart);
    }

    ret = HAL_UARTEx_ReceiveToIdle_DMA(s_ft_uart, s_dma_rx_buf, sizeof(s_dma_rx_buf));
    if ((ret == HAL_OK) && (s_ft_uart->hdmarx != NULL)) {
        __HAL_DMA_DISABLE_IT(s_ft_uart->hdmarx, DMA_IT_HT);
    }
    return ret;
}

// DMA + 空闲中断事件处理入口：可在任意回调中调用
void FTSTS_dma_rx_on_event(UART_HandleTypeDef *huart, uint16_t size)
{
    if ((huart == NULL) || (huart != s_ft_uart)) {
        return;
    }

    if ((size > 0U) && (size <= sizeof(s_dma_rx_buf))) {
        if ((s_stream_len + size) > sizeof(s_stream_buf)) {
            uint16_t remain = (uint16_t)(sizeof(s_stream_buf) - size);
            if ((remain > 0U) && (s_stream_len > remain)) {
                memmove(s_stream_buf, &s_stream_buf[s_stream_len - remain], remain);
                s_stream_len = remain;
            } else if (remain == 0U) {
                s_stream_len = 0U;
            }
        }

        memcpy(&s_stream_buf[s_stream_len], s_dma_rx_buf, size);
        s_stream_len = (uint16_t)(s_stream_len + size);
        ftsts_parse_stream();
    }

    (void)FTSTS_dma_rx_start();
}

// UART接收完成回调函数，解析接收到的数据流
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    usart_cmd_rx_on_event(huart, Size);
    FTSTS_dma_rx_on_event(huart, Size);
}

//读取舵机位置，返回值为-1表示读取失败，非负值表示舵机位置
void FTSTS_read_data(UART_HandleTypeDef *huart, uint8_t id, uint8_t addr, uint8_t data_len)
{
    uint8_t frame[8];
    UART_HandleTypeDef *uart = (huart != NULL) ? huart : s_ft_uart;

    if (uart == NULL) {
        return;
    }

    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = id;
    frame[3] = 0x04U;
    frame[4] = FTSTS_READ_INST;
    frame[5] = addr;
    frame[6] = data_len;
    frame[7] = ftsts_checksum(&frame[2], 5U);

    (void)HAL_UART_Transmit(uart, frame, sizeof(frame), 20U);
}

bool FTSTS_get_last_frame(uint8_t id, ftsts_rx_frame_t *out_frame)
{
    if ((out_frame == NULL) || (s_last_frame[id].timestamp_ms == 0U)) {
        return false;
    }

    *out_frame = s_last_frame[id];
    return true;
}


int servo_get_position(UART_HandleTypeDef *huart, int id)
{
    uint32_t start_ms = HAL_GetTick();
    ftsts_rx_frame_t frame;

    //发送读取位置指令
    FTSTS_read_data(huart, (uint8_t)id, (uint8_t)current_position, 2U);

    while ((HAL_GetTick() - start_ms) < FTSTS_DEFAULT_TIMEOUT_MS) {
        if (FTSTS_get_last_frame((uint8_t)id, &frame)) {
            if (frame.error != 0U) {
                return -(int)frame.error;
            }
            if (frame.parameter_len >= 2U) {
                return (int)((uint16_t)frame.parameter[0] | ((uint16_t)frame.parameter[1] << 8));
            }
        }
        osDelay(1);
    }

    return -1;
} 

void FTSTS_clear_writelock(int id)
{
    uint8_t frame[8];

    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = (uint8_t)id;
    frame[3] = 0x04U;
    frame[4] = FTSTS_WRITE_INST;
    frame[5] = 0x30U;
    frame[6] = 0x00U;
    frame[7] = ftsts_checksum(&frame[2], 5U);

    (void)HAL_UART_Transmit(s_ft_uart, frame, sizeof(frame), 20U);
}


// 设置舵机ID，参数id为新的舵机ID
void FTSTS_setID(int id)
{
    uint8_t frame[8];

    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = 0xFEU;
    frame[3] = 0x04U;
    frame[4] = FTSTS_WRITE_INST;
    frame[5] = 0x05U;
    frame[6] = (uint8_t)id;
    frame[7] = ftsts_checksum(&frame[2], 5U);

    (void)HAL_UART_Transmit(s_ft_uart, frame, sizeof(frame), 20U);
}

// 设置舵机最大力矩，参数id为舵机ID
void FTSTS_setMAX_F(int id)
{
    uint16_t max_f = 700U;
    uint8_t frame[9];

    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = (uint8_t)id;
    frame[3] = 0x05U;
    frame[4] = FTSTS_WRITE_INST;
    frame[5] = 0x10U;
    frame[6] = (uint8_t)(max_f & 0xFFU);
    frame[7] = (uint8_t)((max_f >> 8) & 0xFFU);
    frame[8] = ftsts_checksum(&frame[2], 6U);

    (void)HAL_UART_Transmit(s_ft_uart, frame, sizeof(frame), 20U);
}

// 设置舵机目标位置，参数id为舵机ID，angle为目标角度（单位：度），time为运动时间（单位：毫秒），speed为运动速度（单位：0.1度/秒）
void FTSTS_servo_write_pos(UART_HandleTypeDef *huart, uint8_t id, float angle, float speed)
{
    uint8_t tx_buffer[13];
    int16_t pos_value;
    int16_t speed_value;
    UART_HandleTypeDef *uart = (huart != NULL) ? huart : s_ft_uart;

    //int lenth = sizeof(tx_buffer) / sizeof(tx_buffer[0]) + 2;

    if (uart == NULL) {
        return;
    }

    tx_buffer[0] = 0xFFU;
    tx_buffer[1] = 0xFFU;
    tx_buffer[2] = id;
    tx_buffer[3] = 0x09U;
    tx_buffer[4] = FTSTS_WRITE_INST;
    tx_buffer[5] = 0x2AU;

    angle = angle / 300.0f * 1023.0f;
    pos_value = (int16_t)angle;
    speed_value = (int16_t)speed;

    tx_buffer[6] = (uint8_t)(pos_value & 0xFF);
    tx_buffer[7] = (uint8_t)((pos_value >> 8) & 0xFF);
    tx_buffer[8] = 0x00U;
    tx_buffer[9] = 0x00U;
    tx_buffer[10] = (uint8_t)(speed_value & 0xFF);
    tx_buffer[11] = (uint8_t)((speed_value >> 8) & 0xFF);
    tx_buffer[12] = ftsts_checksum(&tx_buffer[2], 10U);

    (void)HAL_UART_Transmit(uart, tx_buffer, sizeof(tx_buffer), 20U);
}


void FT_rx_data_decode(uint8_t* rxdata)
{

}

// 设置舵机目标位置，参数id为舵机ID，angle为目标角度（单位：度），time为运动时间（单位：毫秒），speed为运动速度（单位：0.1度/秒）
void FTSTS_servo_Syncwrite_pos(UART_HandleTypeDef *huart, uint8_t id, float angle, float time, float speed)
{
    // uint8_t tx_buffer[13];
    // int16_t pos_value;
    // int16_t time_value;
    // int16_t speed_value;
    // UART_HandleTypeDef *uart = (huart != NULL) ? huart : s_ft_uart;

    // int lenth = 

    // if (uart == NULL) {
    //     return;
    // }

    // tx_buffer[0] = 0xFFU;
    // tx_buffer[1] = 0xFFU;
    // tx_buffer[2] = id;
    // tx_buffer[3] = 0x09U;
    // tx_buffer[4] = FTSTS_WRITE_INST;
    // tx_buffer[5] = 0x2AU;

    // angle = angle / 300.0f * 1023.0f;
    // pos_value = (int16_t)angle;
    // time_value = (int16_t)time;
    // speed_value = (int16_t)speed;

    // tx_buffer[6] = (uint8_t)(pos_value & 0xFF);
    // tx_buffer[7] = (uint8_t)((pos_value >> 8) & 0xFF);
    // tx_buffer[8] = (uint8_t)(time_value & 0xFF);
    // tx_buffer[9] = (uint8_t)((time_value >> 8) & 0xFF);
    // tx_buffer[10] = (uint8_t)(speed_value & 0xFF);
    // tx_buffer[11] = (uint8_t)((speed_value >> 8) & 0xFF);
    // tx_buffer[12] = ftsts_checksum(&tx_buffer[2], 10U);

    // (void)HAL_UART_Transmit(uart, tx_buffer, sizeof(tx_buffer), 20U);
}
