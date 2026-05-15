/**
 * @file    action_scheduler.h
 * @brief   动作调度器 —— 管理机械臂高级动作（物块交接、归位等）的状态机
 *
 * 设计原则:
 *   - 单一职责: 本模块仅负责动作流程的状态推进与路径点输出,
 *     不涉及机械臂运动学/轨迹规划/舵机通信.
 *   - 输入: 外部通过 associate_trigger() 等 API 触发动作.
 *   - 输出: 每周期 ACTION_loop() 将当前动作路径点写入
 *     target_x_test[] / target_y_test[] 全局数组,
 *     由 Planar_Robot_Arm 控制循环消费.
 *
 * 集成方式:
 *   在 arm_control_task 中按以下顺序周期调用:
 *     input_arbiter_resolve(...);   // PC/RC 输入仲裁
 *     ACTION_loop();                 // 动作调度(动作激活时覆盖 target 数组)
 *     planar_arm_control_loop();    // 机械臂 IK+轨迹+舵机输出
 *
 * 扩展 action_state_e 的步骤:
 *   1. 在 action_state_e 枚举中新增状态值(如 ACTION_STATE_HOME).
 *   2. 在 action_scheduler.c 中实现对应的处理函数.
 *   3. 在 ACTION_loop() 的 switch 中增加新状态的 case 分支.
 *   4. 若需要外部触发, 在 action_scheduler.h 中声明对应的 trigger 函数.
 */

#ifndef ACTION_SCHEDULER_H
#define ACTION_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

/* ════════════════════════════════════════════════════════════════
 * 交接方向定义
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 物块交接方向枚举
 * @note  dir_id=0: 左臂供给右臂 (LF→RF 或 LB→RB)
 *        dir_id=1: 右臂供给左臂 (RF→LF 或 RB→LB)
 */
typedef enum {
    ARM_DIR_L_TO_R = 0,  /**< 左臂供给方向: LF→RF, LB→RB */
    ARM_DIR_R_TO_L = 1,  /**< 右臂供给方向: RF→LF, RB→LB */
} arm_dir_id_e;

/* ════════════════════════════════════════════════════════════════
 * 全局动作状态枚举（可扩展）
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 全局动作状态机状态枚举
 *
 * 【扩展方法】
 *   需要新增动作类型时, 在此枚举中追加新的状态常量.
 *   例如要增加"舞蹈动作":
 *     ACTION_STATE_DANCE = 2,
 *
 *   然后在 action_scheduler.c 中:
 *     1. 实现 dance_state_machine() 处理函数.
 *     2. 在 ACTION_loop() 末尾增加:
 *        case ACTION_STATE_DANCE: dance_state_machine(); break;
 *     3. 如需外部触发, 声明 dance_trigger() 并在 cmd_dispatch_frame 中增加命令字.
 */
typedef enum {
    ACTION_STATE_IDLE      = 0,   /**< 空闲, 等待动作指令 */
    ACTION_STATE_ASSOCIATE = 1,   /**< 执行物块交接动作中 */
    /* ── 预留扩展位 (按需取消注释) ── */
    /* ACTION_STATE_HOME      = 2,   // 归位中 */
    /* ACTION_STATE_DANCE     = 3,   // 舞蹈动作 */
    /* ACTION_STATE_CALIB     = 4,   // 标定中 */
} action_state_e;

/* ════════════════════════════════════════════════════════════════
 * 物块交接子状态枚举
 * ════════════════════════════════════════════════════════════════ */

typedef enum {
    BLOCK_ASSOCIATE_IDLE          = 0,  /**< 空闲, 等待交接指令 */
    BLOCK_ASSOCIATE_TO_MIDDLE     = 1,  /**< 移动到交接中间位置 */
    BLOCK_ASSOCIATE_WAIT          = 2,  /**< 等待双方就绪 */
    BLOCK_ASSOCIATE_ADSORB        = 3,  /**< 等待吸附确认 */
    BLOCK_ASSOCIATE_VALVE_CONTROL = 4,  /**< 电磁阀控制释放/吸取 */
    BLOCK_ASSOCIATE_COMPLETE      = 5,  /**< 交接完成, 移动到安全位置 */
} block_associate_state_e;

/* ════════════════════════════════════════════════════════════════
 * 公共 API
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 周期调用: 驱动所有动作状态机推进
 *
 * 必须在 RTOS 控制任务循环中调用 (建议 5~20ms 周期).
 * 内部根据 g_action_state 分派到对应的子状态机.
 * 动作激活时, 本函数会将路径点写入 target_x_test/y_test 全局数组,
 * 覆盖 input_arbiter 的输出.
 */
void ACTION_loop(void);

/**
 * @brief 接收遥控器指令并触发交接 (由 ACTION_loop 内部调用)
 *
 * 读取 DT7 遥控器状态, 在满足条件时调用 associate_trigger().
 * 将此逻辑独立为函数, 方便在不同遥控器协议间切换.
 */
void ACTION_recvie(void);

/**
 * @brief 触发指定交接对开始物块移交流程
 *
 * 仅在对应交接对处于 IDLE 状态时才能成功触发.
 * 触发后全局动作状态切换到 ACTION_STATE_ASSOCIATE,
 * 交接子状态进入 TO_MIDDLE, 后续由 ACTION_loop() 自动推进.
 *
 * @param pair_idx  交接对索引: 0=前侧(LF↔RF), 1=后侧(LB↔RB)
 * @param dir_id    交接方向: ARM_DIR_L_TO_R(0)=左→右, ARM_DIR_R_TO_L(1)=右→左
 * @retval true   成功触发
 * @retval false  触发失败 (交接对正忙或参数非法)
 */
bool associate_trigger(uint8_t pair_idx, uint8_t dir_id);

/**
 * @brief 强制中止指定交接对的当前流程
 *
 * 立即回到 IDLE 状态, 可用于紧急停止或故障恢复.
 * 若所有交接对均回到 IDLE, 全局动作状态恢复为 ACTION_STATE_IDLE.
 *
 * @param pair_idx  交接对索引: 0=前侧, 1=后侧
 */
void associate_abort(uint8_t pair_idx);

/**
 * @brief 查询指定交接对的当前子状态
 *
 * @param pair_idx  交接对索引: 0=前侧, 1=后侧
 * @return 当前 block_associate_state_e 枚举值的 uint8_t 表示
 *         非法索引返回 BLOCK_ASSOCIATE_IDLE
 */
uint8_t associate_get_state(uint8_t pair_idx);

/**
 * @brief 查询全局动作状态机的当前状态
 *
 * @return 当前 action_state_e 枚举值
 */
action_state_e action_get_global_state(void);

/**
 * @brief 获取全局动作状态指针(供 arm_control_task 查询是否在动作中)
 *
 * @return 指向 g_action_state 的指针
 */
const volatile action_state_e *action_get_state_ptr(void);

/**
 * @brief 获取前侧/后侧交接对手状态（外部查询用）
 *
 * @return pair_idx=0(前侧) 的 block_associate_state_e 值
 */
block_associate_state_e action_get_left_state(void);

/**
 * @brief 获取后侧交接对手状态（外部查询用）
 *
 * @return pair_idx=1(后侧) 的 block_associate_state_e 值
 */
block_associate_state_e action_get_right_state(void);

#endif /* ACTION_SCHEDULER_H */
