#ifndef PNEUMATIC_CONTROL_H
#define PNEUMATIC_CONTROL_H

#include "main.h"
#include "DJI_motor.h"

#include <stdint.h>

void pump_M3508_init(void);
void pump_speed_set(float target_speed);

/* 继电器控制（4路，与电磁阀/电磁铁共享GPIO） */
void relay_init(void);
void relay_control(uint8_t relay_id, uint8_t state);

/* 协议使用 0~3 对应 4 路电磁阀 */
typedef enum {
    SOLENOID_1 = 0,
    SOLENOID_2 = 1,
    SOLENOID_3 = 2,
    SOLENOID_4 = 3,
} Solenoid_ID;

//电磁阀
typedef struct
{

    uint8_t ID[4];
    uint8_t command[4];
    uint8_t state[4];

}Solenoid_state_s;


void Solenoid_Valve_init(void);
void Solenoid_Valve_control(uint8_t id, uint8_t state);
void solenoid_control(Solenoid_ID solenoid_id, uint8_t state);
void pneumatic_control_loop(void);

#endif

