#ifndef VOICE_COMMAND_H
#define VOICE_COMMAND_H

#include <stdint.h>

// 指令协议结构体定义
typedef struct
{
    uint8_t cmd_id;
    const char *command;
    const char *response;
    uint8_t is_master;
    uint8_t tx_data[5];
    uint8_t rx_data[5];
} VoiceCommand;

// 前21个指令表
extern const VoiceCommand commands[];
extern const int commands_count;

#endif
