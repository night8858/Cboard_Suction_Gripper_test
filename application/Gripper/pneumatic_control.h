#ifndef PNEUMATIC_CONTROL_H
#define PNEUMATIC_CONTROL_H

#include "main.h"
#include "DJI_motor.h"

#include <stdint.h>
#include <stdbool.h>

void pump_M3508_init(void);
void pump_speed_set(float target_speed);

/**
 * @brief 气泵控制状态结构体
 *
 * 封装 M3508 电机运行状态, 供 pump_control_loop() 周期驱动.
 * 外部模块通过 pump_ctrl_toggle() 切换启停,
 * 通过 PC/RC 设置 target_speed_rpm 控制转速.
 */
typedef struct {
    uint8_t  motor_id;          /**< M3508 CAN 通信 ID (0x201) */
    float    current_speed_rpm; /**< 当前实际转速 (RPM, 每周期从电机反馈刷新) */
    float    target_speed_rpm;  /**< 目标转速 (RPM, is_running 时输出) */
    int16_t  temperature;       /**< 电机温度 (°C, 从 CAN 反馈刷新) */
    bool     is_running;        /**< 气泵启停状态: true=运行, false=停止 */
} PumpCtrl;

/**
 * @brief 气泵控制循环 — 每周期调用, 管理启停与转速输出
 *
 * 内部根据 is_running 决定输出 target_speed_rpm 还是 0,
 * 并从 DJI_motor_3508 全局实例刷新 current_speed_rpm 与 temperature.
 *
 * @param pump  指向 PumpCtrl 实例的指针
 */
void pump_control_loop(PumpCtrl *pump);

/**
 * @brief 切换气泵启停状态 (边沿触发, 取反)
 * @param pump  指向 PumpCtrl 实例的指针
 */
void pump_ctrl_toggle(PumpCtrl *pump);

/**
 * @brief 查询气泵是否运行中
 * @param pump  指向 PumpCtrl 实例的指针 (NULL 安全)
 * @retval true   运行中
 * @retval false  已停止 或 pump 为空
 */
bool pump_ctrl_is_running(const PumpCtrl *pump);

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

