#ifndef HOST_STUB_STM32F4XX_HAL_H
#define HOST_STUB_STM32F4XX_HAL_H

#include <stdint.h>

typedef struct {
    int dummy;
} CAN_HandleTypeDef;

uint32_t HAL_GetTick(void);

#endif
