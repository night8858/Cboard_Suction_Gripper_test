#ifndef BLOCK_INSPECT_H
#define BLOCK_INSPECT_H


typedef struct
{
    uint8_t ID[4];         // 4个微动开关的ID   LF RF LB RB
    uint8_t state[4];      // 4个微动开关的状态    0-未吸附 1-吸附

} SwitchInput;

extern SwitchInput g_switch_input;
void block_inspect_process(void);



#endif
