#ifndef HOST_STUB_PNEUMATIC_CONTROL_H
#define HOST_STUB_PNEUMATIC_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool is_running;
    float target_speed_rpm;
} PumpCtrl;

void pump_speed_set(float target_speed);
void relay_control(uint8_t relay_id, uint8_t state);

#endif
