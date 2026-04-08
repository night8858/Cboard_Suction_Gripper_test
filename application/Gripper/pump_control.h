#include "main.h"

#include "DJI_motor.h"

void pump_M3508_init(void);

void pump_speed_set(float target_speed);

enum Solenoid_ID {
    SOLENOID_1 = 0,
    SOLENOID_2 = 1,
    SOLENOID_3 = 2,
    SOLENOID_4 = 3
    // 可添加更多电磁阀ID
};

