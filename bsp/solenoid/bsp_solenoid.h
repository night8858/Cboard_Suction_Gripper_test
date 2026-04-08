#ifndef BSP_SOLENOID_H
#define BSP_SOLENOID_H

#include <stdint.h>

void bsp_solenoid_init(void);
void bsp_solenoid_set(uint8_t channel, uint8_t on);

#endif
