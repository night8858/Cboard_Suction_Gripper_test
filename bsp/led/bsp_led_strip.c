#include "bsp_led_strip.h"

#include "bsp_led.h"
#include "variables.h"

static uint16_t s_frame_index;
static uint8_t s_dma_ready;

void bsp_led_strip_init(void) {
  s_frame_index = 0;
  s_dma_ready = 0;
}

void bsp_led_strip_service(uint8_t mode, uint8_t brightness) {
  (void)brightness;

  s_frame_index++;

  if (mode == 0U) {
    if ((s_frame_index % 20U) == 0U) {
      led_heartbeat();
    }
  } else {
    if ((s_frame_index % 5U) == 0U) {
      led_heartbeat();
    }
  }
}

uint8_t bsp_led_strip_dma_ready(void) { return s_dma_ready; }
