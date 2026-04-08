#include "bsp_solenoid.h"

#include "gpio.h"
#include "variables.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} solenoid_pin_map_t;

static const solenoid_pin_map_t s_solenoid_map[APP_SOLENOID_COUNT] = {
    {GPIOH, GPIO_PIN_12},
    {GPIOH, GPIO_PIN_10},
};

void bsp_solenoid_init(void) {
  uint8_t i = 0;

  for (i = 0; i < APP_SOLENOID_COUNT; i++) {
    HAL_GPIO_WritePin(s_solenoid_map[i].port, s_solenoid_map[i].pin, GPIO_PIN_RESET);
  }
}

void bsp_solenoid_set(uint8_t channel, uint8_t on) {
  if (channel >= APP_SOLENOID_COUNT) {
    return;
  }

  HAL_GPIO_WritePin(s_solenoid_map[channel].port, s_solenoid_map[channel].pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
