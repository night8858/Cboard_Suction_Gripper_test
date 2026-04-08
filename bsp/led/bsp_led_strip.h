#ifndef BSP_LED_STRIP_H
#define BSP_LED_STRIP_H

#include <stdint.h>

void bsp_led_strip_init(void);
void bsp_led_strip_service(uint8_t mode, uint8_t brightness);
uint8_t bsp_led_strip_dma_ready(void);

#endif
