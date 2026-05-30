/**
 * @file    input_arbiter.h
 * @brief   输入仲裁器 —— 按优先级将 PC/RC 输入合并为机械臂目标位置
 *
 * 优先级: PC(上位机) > RC(遥控器)
 *
 * 工作流程:
 *   1. 外部周期调用 input_arbiter_update_rc() 和 input_arbiter_update_pc()
 *      喂入最新的 RC 数据和 PC 指令.
 *   2. 在控制循环中调用 input_arbiter_resolve(), 将仲裁结果写入
 *      target_x_test[] / target_y_test[] 全局数组.
 *   3. 当 action_active=true 时跳过解析 (由 ACTION_loop 接管).
 *
 * 集成方式:
 *   在 arm_control_task 循环中:
 *     input_arbiter_update_rc(get_remote_control_point());
 *     input_arbiter_resolve(g_action_state != ACTION_STATE_IDLE);
 *     ACTION_loop();
 *     planar_arm_control_loop();
 */

#ifndef INPUT_ARBITER_H
#define INPUT_ARBITER_H

#include <stdbool.h>
#include <stdint.h>
#include "variables.h"
#include "DT7.h"

/* ════════════════════════════════════════════════════════════════
 * 安全参数
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 输入数据新鲜度超时 (ms)
 *
 * 若 RC 或 PC 输入的最后更新时间距今超过此阈值,
 * 该来源被视为"过期", 输入仲裁器拒绝消费过期数据,
 * 机械臂保持最后有效指令位置不动.
 *
 * 500ms 对应 RC 约 50 帧丢失 (100Hz), 或 PC 约 25 个控制周期无数据.
 */
#define INPUT_FRESHNESS_TIMEOUT_MS  500U

/* ════════════════════════════════════════════════════════════════
 * 公共 API
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 初始化输入仲裁器
 *
 * 将内部 PC/RC 缓冲区清零, 设置默认目标为归位点.
 * 必须在 RTOS 任务启动后、控制循环开始前调用一次.
 */
void input_arbiter_init(void);

/**
 * @brief 更新遥控器(RC)输入缓冲区
 *
 * 在控制循环中每周期调用, 将最新的 RC 数据快照到内部缓冲区.
 * 本函数不执行 I/O, 仅做数据拷贝.
 *
 * @param rc  指向 RC_ctrl_t 遥控器数据结构的指针
 */
void input_arbiter_update_rc(const RC_ctrl_t *rc);

/**
 * @brief 更新上位机(PC)输入缓冲区
 *
 * 当 command_decode 模块收到并解析 PC 指令后调用,
 * 将 all_pc_command 中的控制数据快照到内部缓冲区.
 *
 * @param cmd  指向 all_pc_command 的指针 (由 cmd_execute_all 传入)
 */
void input_arbiter_update_pc(const all_pc_command *cmd);

/**
 * @brief 执行输入仲裁, 将结果写入 target_x_test/y_test
 *
 * 仲裁规则:
 *   1. 若 action_active=true, 跳过所有输入, 直接返回
 *      (target 数组由 ACTION_loop 写入).
 *   2. 遍历四个机械臂, 对每个臂:
 *      - 检查 PC 缓冲区中对应字段是否为有效值 (非 NaN/Inf).
 *      - 若 PC 有效, 使用 PC 值.
 *      - 否则使用 RC 值 (RC 缓冲区中存储的是 DT7 解析后的目标坐标).
 *      - 若 RC 也未就绪, 保持 target 数组当前值不变.
 *   3. 执行路径点状态机 (P2→P4→P3 中间过渡).
 *
 * @param action_active  当前是否有动作正在执行 (true=跳过仲裁)
 */
void input_arbiter_resolve(bool action_active);

/**
 * @brief 执行 4DOF 输入仲裁, 将 RC 手动控制映射到右臂目标位姿。
 *
 * @param action_active 当前是否有动作正在执行(true=跳过仲裁)
 */
void input_arbiter_resolve_4dof(bool action_active);

/**
 * @brief 查询输入仲裁器是否已就绪
 *
 * 就绪条件: RC 或 PC 中至少一个来源收到过有效数据且处于新鲜期内.
 * 冷启动时返回 false, 直到首次有效输入到达.
 *
 * 调用方 (arm_control_task) 应在调用 planar_arm_control_loop()
 * 前检查此函数, 未就绪时跳过运动控制以保障安全.
 *
 * @retval true   至少一个输入源就绪, 可以执行运动控制
 * @retval false  无有效输入源, 应跳过运动控制
 */
bool input_arbiter_is_ready(void);

#endif /* INPUT_ARBITER_H */
