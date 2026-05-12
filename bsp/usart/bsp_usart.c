#include "bsp_usart.h"
#include "main.h"
#include "string.h"
#include <stdarg.h>
#include <stdio.h>

#include "variables.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void usart_tx_dma_init(UART_HandleTypeDef *huart) {
  if (huart == NULL || huart->Instance == NULL) {
    return;
  }

  // enable DMA transfer request for UART TX
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);
}

void usart1_tx_dma_init(void) {
  usart_tx_dma_init(&huart6);
}

void usart1_tx_dma_enable(uint8_t *data, uint16_t len) {

  // disable DMA
  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart1_tx);
  while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
  }

  // clear flag
  // 清除标志位
  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);
  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_HTIF7);

  // set data address
  // 设置数据地址
  hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
  // set data length
  // 设置数据长度
  hdma_usart1_tx.Instance->NDTR = len;

  // enable DMA
  // 使能DMA
  __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart6_tx_dma_enable(uint8_t *data, uint16_t len) {

  // disable DMA
  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart6_tx);
  while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_usart6_tx);
  }

  // clear flag
  // 清除标志位
  __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_LISR_TCIF1);
  __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_LISR_HTIF1);

  // set data address
  // 设置数据地址
  hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
  // set data length
  // 设置数据长度
  hdma_usart6_tx.Instance->NDTR = len;

  // enable DMA
  // 使能DMA
  __HAL_DMA_ENABLE(&hdma_usart6_tx);
}

void usart_printf(const char *fmt, ...) {
  static uint8_t tx_buf[256] = {0};
  static va_list ap;
  static uint16_t len;
  va_start(ap, fmt);

  // return length of string
  // 返回字符串长度
  len = vsprintf((char *)tx_buf, fmt, ap);

  va_end(ap);
  tx_buf[len] = '\0';
  usart1_tx_dma_enable(tx_buf, len);
}

void uart_dma_printf(UART_HandleTypeDef *huart, char *fmt, ...) {
  static uint8_t tx_buf[128] = {0};
  static va_list ap;
  static uint16_t len = 0;

  va_start(ap, fmt);
  len = vsprintf((char *)tx_buf, fmt, ap);
  va_end(ap);

  tx_buf[len] = '\0';

   // CDC_Transmit_FS(tx_buf, len);
    tx_buf[sizeof(tx_buf) - 1] = '\0';
    va_start(ap, fmt);
  HAL_UART_Transmit_DMA(huart, tx_buf, len);

  // len = vsnprintf((char *)tx_buf, sizeof(tx_buf) - 1, fmt,
  //                 ap); // 使用vsnprintf更安全
  // va_end(ap);

  // if (huart->gState == HAL_UART_STATE_READY) {
  //   HAL_UART_Transmit_DMA(huart, tx_buf, len);
  // } else {
  //   // 可以在这里添加等待或错误处理
  //   // 或者使用阻塞式传输作为后备
  //   HAL_UART_Transmit(huart, tx_buf, len, 1000);
  // }
}



extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

#define USART_CMD_DMA_BUF_SIZE 64U
#define USART_CMD_LINE_BUF_SIZE 64U

static UART_HandleTypeDef *s_cmd_uart = NULL;
static uint8_t s_cmd_dma_buf[USART_CMD_DMA_BUF_SIZE];
static char s_cmd_line_buf[USART_CMD_LINE_BUF_SIZE];
static uint16_t s_cmd_line_len = 0U;

static void usart_cmd_handle_line(const char *line) {
  float x = 0.0f;
  float y = 0.0f;

  if (line == NULL) {
    return;
  }

  // 指令格式: RB,10,10\n 或 RB,10,10/n
  if (strncmp(line, "RB,", 3) == 0) {
    if (sscanf(&line[3], "%f,%f", &x, &y) == 2) {
      Arm_RB.end_aim_x = x;
      Arm_RB.end_aim_y = y;
      Arm_RB.state = ARM_STATE_MOVING;
    }
  } else if (strncmp(line, "RF,", 3) == 0) {
    if (sscanf(&line[3], "%f,%f", &x, &y) == 2) {
      Arm_RF.end_aim_x = x;
      Arm_RF.end_aim_y = y;
      Arm_RF.state = ARM_STATE_MOVING;
    }
  } else if (strncmp(line, "LB,", 3) == 0) {
    if (sscanf(&line[3], "%f,%f", &x, &y) == 2) {
      Arm_LB.end_aim_x = x;
      Arm_LB.end_aim_y = y;
      Arm_LB.state = ARM_STATE_MOVING;
    }
  } else if (strncmp(line, "RB,", 3) == 0) {
    if (sscanf(&line[3], "%f,%f", &x, &y) == 2) {
      Arm_RB.end_aim_x = x;
      Arm_RB.end_aim_y = y;
      Arm_RB.state = ARM_STATE_MOVING;
    }
  }
}

static void usart_cmd_parse_bytes(const uint8_t *data, uint16_t size) {
  uint16_t i = 0;

  if (data == NULL || size == 0U) {
    return;
  }

  for (i = 0; i < size; i++) {
    char ch = (char)data[i];

    if (ch == '\r' || ch == '\n') {
      if (s_cmd_line_len > 0U) {
        s_cmd_line_buf[s_cmd_line_len] = '\0';
        usart_cmd_handle_line(s_cmd_line_buf);
        s_cmd_line_len = 0U;
      }
      continue;
    }

    // 兼容 /n 作为结束符
    if (ch == 'n' && s_cmd_line_len > 0U && s_cmd_line_buf[s_cmd_line_len - 1U] == '/') {
      s_cmd_line_buf[s_cmd_line_len - 1U] = '\0';
      usart_cmd_handle_line(s_cmd_line_buf);
      s_cmd_line_len = 0U;
      continue;
    }

    if (s_cmd_line_len < (USART_CMD_LINE_BUF_SIZE - 1U)) {
      s_cmd_line_buf[s_cmd_line_len++] = ch;
    } else {
      // 行缓冲溢出时丢弃当前行，避免脏数据影响后续解析
      s_cmd_line_len = 0U;
    }
  }
}

void usart_cmd_rx_init(UART_HandleTypeDef *huart) {
  s_cmd_uart = huart;
  s_cmd_line_len = 0U;
  memset(s_cmd_dma_buf, 0, sizeof(s_cmd_dma_buf));
  memset(s_cmd_line_buf, 0, sizeof(s_cmd_line_buf));
}

HAL_StatusTypeDef usart_cmd_rx_start(void) {
  HAL_StatusTypeDef ret;

  if (s_cmd_uart == NULL) {
    return HAL_ERROR;
  }

  if (s_cmd_uart->RxState != HAL_UART_STATE_READY) {
    (void)HAL_UART_AbortReceive(s_cmd_uart);
  }

  ret = HAL_UARTEx_ReceiveToIdle_DMA(s_cmd_uart, s_cmd_dma_buf, sizeof(s_cmd_dma_buf));
  if ((ret == HAL_OK) && (s_cmd_uart->hdmarx != NULL)) {
    __HAL_DMA_DISABLE_IT(s_cmd_uart->hdmarx, DMA_IT_HT);
  }

  return ret;
}

void usart_cmd_rx_on_event(UART_HandleTypeDef *huart, uint16_t size) {
  if (huart == NULL || huart != s_cmd_uart) {
    return;
  }

  if (size > 0U && size <= sizeof(s_cmd_dma_buf)) {
    usart_cmd_parse_bytes(s_cmd_dma_buf, size);
  }

  (void)usart_cmd_rx_start();
}
