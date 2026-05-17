#include "main.h"

#include "DJI_motor.h"
#include "variables.h"
#include "user_lib.h"

#include "pneumatic_control.h"

#define USE_RELAY_MODULE


extern s_Dji_motor_data_t DJI_motor_3508;
extern ramp_function_source_t pump_ramp;

/* ════════════════════════════════════════════════════════════════
 * 气泵全局控制实例
 * ════════════════════════════════════════════════════════════════ */

/** @brief 气泵控制状态 (全局单例, 供 input_arbiter 等模块访问) */
PumpCtrl g_pump = {
    .motor_id         = 1,
    .current_speed_rpm = 0.0f,
    .target_speed_rpm  = 3000.0f,  /**< 默认目标转速 3000 RPM */
    .temperature       = 0,
    .is_running        = false,
};

// static GPIO_PinState s_solenoid_state_to_pin(uint8_t state)
// {
//     return (state != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
// }


void pump_M3508_init(void)
{
    s_Dji_motor_config(&DJI_motor_3508, 1, &hcan1);
    DJI_motor_SPEED_PID_init(&DJI_motor_3508, 14.0f, 1.0f, 1.5f, 1000, 20000);
}

/**
 * @brief 设置气泵目标转速并立即执行一次 PID 控制输出
 *
 * 直接控制 M3508 电机转速, 不经过 PumpCtrl 状态机.
 * 用于 PC 上位机直接调速或紧急控制场景.
 *
 * @param target_speed  目标转速 (RPM), 0=停止
 */
void pump_speed_set(float target_speed)
{
    M3508_PID_SPEED_CONTROL(&DJI_motor_3508, target_speed);
    DJI_motor_3508_ID_1to4_control(DJI_motor_3508.out_current, 0, 0, 0);
}

/**
 * @brief 气泵控制循环 — 每周期驱动启停与转速
 *
 * 从 DJI_motor_3508 刷新实时数据, 根据 is_running 输出转速.
 * 在 pump_control_task 中周期调用 (建议 ≤10ms).
 *
 * @param pump  指向 PumpCtrl 实例的指针 (NULL 安全)
 */
void pump_control_loop(PumpCtrl *pump)
{
    if (pump == NULL) {
        return;
    }

    /* 从 CAN 电机反馈刷新实时转速与温度 */
    pump->current_speed_rpm = (float)DJI_motor_3508.back_motor_speed;
    pump->temperature       = DJI_motor_3508.temperature;

    /* 根据启停状态决定输出转速:
     * is_running=true  → 输出 target_speed_rpm
     * is_running=false → 输出 0 (停止)                     */
    float out = pump->is_running ? pump->target_speed_rpm : 0.0f;

    M3508_PID_SPEED_CONTROL(&DJI_motor_3508, out);
    DJI_motor_3508_ID_1to4_control(DJI_motor_3508.out_current, 0, 0, 0);
}

/**
 * @brief 切换气泵启停 (取反)
 * @param pump  指向 PumpCtrl 实例的指针
 */
void pump_ctrl_toggle(PumpCtrl *pump)
{
    if (pump == NULL) {
        return;
    }
    pump->is_running = !pump->is_running;
}

/**
 * @brief 查询气泵是否运行中
 * @param pump  指向 PumpCtrl 实例的指针
 * @retval true   运行中
 * @retval false  已停止 或 pump 为空
 */
bool pump_ctrl_is_running(const PumpCtrl *pump)
{
    return (pump != NULL) && pump->is_running;
}

#ifdef USE_RELAY_MODULE
// 继电器模块控制函数
// 继电器ID与GPIO引脚的映射关系（根据实际连接修改了）
static const uint16_t relay_pins[4] = {RELAY1_Pin_Pin, RELAY2_Pin_Pin, RELAY4_Pin_Pin,RELAY3_Pin_Pin};

void relay_init(void)
{
    /* 初始化时默认全部打开，确保安全 */
    for (uint8_t i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(GPIOB, relay_pins[i], GPIO_PIN_SET);
    }
}

void relay_control(uint8_t relay_id, uint8_t state)
{
    if (relay_id >= 4) {
        return;
    }

    GPIO_PinState pin_state = (state != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOB, relay_pins[relay_id], pin_state);
}


#endif /* USE_RELAY_MODULE */
