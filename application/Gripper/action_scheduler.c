/**
 * @file    action_scheduler.c
 * @brief   动作调度器实现 —— 物块交接状态机、全局动作管理
 *
 * 本文件从 Planar_Robot_Arm.c 中提取交接相关代码,
 * 遵循单一职责原则, 与机械臂运动学解耦.
 *
 * 原始代码迁移自 Planar_Robot_Arm.c (2026-05), 以下函数保持原有实现逻辑不变:
 *   - assoc_set_state()
 *   - assoc_is_timed_out()
 *   - associate_run_one_pair() (增加 dir_id 参数, 恢复被注释的方向分支)
 *   - associate_abort()
 *   - associate_get_state()
 *   - ACTION_recvie()
 *
 * 新增功能:
 *   - assist_run_one_pair 支持 dir_id 控制交接方向
 *   - 交接位置简化为 ASSOC_MID (交接位置) + ASSOC_SAFE (安全位置, 即原 DONE 位置)
 *   - 全局动作状态机支持扩展
 */

#include <stdbool.h>
#include <math.h>
#include "action_scheduler.h"
#include "Planar_Robot_Arm.h"
#include "DT7.h"
#include "pneumatic_control.h"
#include "stm32f4xx_hal.h"

/* ── 外部引用: target_x_test/y_test 由 Planar_Robot_Arm.c 定义 ── */
extern float target_x_test[4];
extern float target_y_test[4];

/* ════════════════════════════════════════════════════════════════
 * 交接时序参数 (占位符, 后续根据实测调整)
 * ════════════════════════════════════════════════════════════════ */
#define ASSOC_TIMEOUT_MS        3000U   /**< 单步最大超时 (ms) */
#define ASSOC_MOVE_DELAY_MS     1500U   /**< 机械臂移动到位预估时间 (ms) */
#define ASSOC_WAIT_DELAY_MS     600U    /**< 双方到位后的稳定等待 (ms) */
#define ASSOC_ADSORB_TIMEOUT_MS 1500U   /**< 吸取物块最大等待时间 (ms) */
#define ASSOC_HOLD_MS           500U    /**< 交接完成后保持时间 (ms) */

/* ════════════════════════════════════════════════════════════════
 * 交接空间位置 (占位坐标, 外部坐标系, 单位 mm)
 *
 * 位置语义:
 *   ASSOC_MID  — 交接目标位置 (两臂在此处进行物块物理交接)
 *   ASSOC_SAFE — 交接安全位置 (准备/完成/归位用, 各臂独立安全位)
 *
 * 前侧交接对用正 X, 后侧交接对用负 X.
 * 按 arm_id 索引: [ARM_ID_LF, ARM_ID_RF, ARM_ID_LB, ARM_ID_RB]
 * ════════════════════════════════════════════════════════════════ */

/* 交接中间位置 — 前侧/后侧共用 X 坐标, Y 对称 */
#define ASSOC_MID_X_FRONT    550.0f      /**< 前侧交接中间 X (mm) */
#define ASSOC_MID_X_REAR    -600.0f      /**< 后侧交接中间 X (mm) */
#define ASSOC_MID_Y          40.0f       /**< 交接中间 |Y| (mm, armA=+Y, armB=-Y) */

/* 各臂安全位置 (占位数值, 后续根据实际机械结构调整) */
static const float ASSOC_SAFE_X[4] = {
    250.0f,    /**< ARM_ID_LF 安全 X */
    250.0f,    /**< ARM_ID_RF 安全 X */
   -250.0f,    /**< ARM_ID_LB 安全 X */
   -250.0f,    /**< ARM_ID_RB 安全 X */
};

static const float ASSOC_SAFE_Y[4] = {
    400.0f,    /**< ARM_ID_LF 安全 Y */
   -400.0f,    /**< ARM_ID_RF 安全 Y */
    400.0f,    /**< ARM_ID_LB 安全 Y */
   -400.0f,    /**< ARM_ID_RB 安全 Y */
};

/* ════════════════════════════════════════════════════════════════
 * 全局状态变量
 * ════════════════════════════════════════════════════════════════ */

/** @brief 全局动作状态机当前状态 */
static volatile action_state_e g_action_state = ACTION_STATE_IDLE;

/** @brief 交接对子状态 (外部查询用, 保持与原 LEFT/RIGHT 全局变量兼容) */
block_associate_state_e LEFT  = BLOCK_ASSOCIATE_IDLE;
block_associate_state_e RIGHT = BLOCK_ASSOCIATE_IDLE;

static volatile block_associate_state_e g_left_state  = BLOCK_ASSOCIATE_IDLE;
static volatile block_associate_state_e g_right_state = BLOCK_ASSOCIATE_IDLE;

/* ════════════════════════════════════════════════════════════════
 * 内部状态机上下文
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 交接对内部上下文结构体
 *
 * 每对交接臂独立拥有一个实例, 封装当前状态、计时、方向、吸附标志.
 */
typedef struct {
    block_associate_state_e state;         /**< 当前主状态 */
    uint32_t                enter_tick;    /**< 进入当前状态的系统 tick */
    uint32_t                state_timeout; /**< 当前状态的超时时间 (ms), 0=无超时 */
    bool                    block_grabbed; /**< 物块已成功被吸附 */
    uint8_t                 dir_id;        /**< 本对当前交接方向: 0=L→R, 1=R→L */
} AssociateCtx;

/**
 * @brief 两组交接对的内部上下文
 *        idx 0 = 前侧 (LF ↔ RF)
 *        idx 1 = 后侧 (LB ↔ RB)
 */
static AssociateCtx s_assoc_ctx[2] = {
    { .state = BLOCK_ASSOCIATE_IDLE, .enter_tick = 0, .state_timeout = 0,
      .block_grabbed = false, .dir_id = 0 },
    { .state = BLOCK_ASSOCIATE_IDLE, .enter_tick = 0, .state_timeout = 0,
      .block_grabbed = false, .dir_id = 0 },
};

/* ════════════════════════════════════════════════════════════════
 * arm_id 到数组索引的转换 (与 Planar_Robot_Arm.c 中的 arm_id_to_index 一致)
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 将 arm_id_e 转换为 target_x_test[] 数组索引
 * @param arm_id  机械臂 ID 枚举值
 * @return 数组索引 (0~3), 非法 ID 返回 -1
 */
static int arm_id_to_idx(arm_id_e arm_id)
{
    switch (arm_id) {
        case ARM_ID_LF: return 0;
        case ARM_ID_RF: return 1;
        case ARM_ID_LB: return 2;
        case ARM_ID_RB: return 3;
        default:        return -1;
    }
}

/* ════════════════════════════════════════════════════════════════
 * 内部辅助函数
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 切换到新状态, 记录进入时刻和超时时间
 * @param pair_idx   交接对索引 (0=前侧, 1=后侧)
 * @param new_state  目标状态
 * @param timeout_ms 该状态的超时时间 (ms), 0=永不超时
 */
static void assoc_set_state(uint8_t pair_idx, block_associate_state_e new_state, uint32_t timeout_ms)
{
    if (pair_idx >= 2) {
        return;
    }
    s_assoc_ctx[pair_idx].state         = new_state;
    s_assoc_ctx[pair_idx].enter_tick    = HAL_GetTick();
    s_assoc_ctx[pair_idx].state_timeout = timeout_ms;
}

/**
 * @brief 检查当前状态是否超时
 * @param ctx  交接对上下文指针
 * @retval true   已超时 (仅当 state_timeout > 0 时有效)
 * @retval false  未超时, 或上下文为空, 或无超时设置
 */
static inline bool assoc_is_timed_out(const AssociateCtx *ctx)
{
    if (ctx == NULL || ctx->state_timeout == 0) {
        return false;
    }
    /* HAL_GetTick() 使用 uint32_t, 差值运算对溢出也是安全的 */
    return ((HAL_GetTick() - ctx->enter_tick) >= ctx->state_timeout);
}

/**
 * @brief 检查所有交接对是否均已回到 IDLE
 * @retval true   全部 IDLE (可恢复全局 ACTION_STATE_IDLE)
 * @retval false  至少一对仍在交接中
 */
static bool assoc_all_idle(void)
{
    return (s_assoc_ctx[0].state == BLOCK_ASSOCIATE_IDLE) &&
           (s_assoc_ctx[1].state == BLOCK_ASSOCIATE_IDLE);
}

/* ════════════════════════════════════════════════════════════════
 * 单对交接状态机
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 单对交接臂的状态机核心推进逻辑
 *
 * 根据 pair_idx 和内部存储的 dir_id 确定供给方(armA)与接收方(armB),
 * 按当前子状态执行对应动作, 并在条件满足时切换到下一状态.
 * 所有状态均包含超时保护, 防止流程卡死.
 *
 * 交接流程:
 *   IDLE → TO_MIDDLE(移动到交接位) → WAIT(等待就绪)
 *   → ADSORB(吸取物块) → VALVE_CONTROL(阀切换) → COMPLETE(完成) → IDLE
 *
 * @param pair_idx  交接对索引: 0=前侧(LF↔RF), 1=后侧(LB↔RB)
 * @param dir_id    交接方向: ARM_DIR_L_TO_R(0)=左→右, ARM_DIR_R_TO_L(1)=右→左
 */
static void associate_run_one_pair(uint8_t pair_idx, uint8_t dir_id)
{
    if (pair_idx >= 2 || dir_id > 1) {
        return;
    }

    AssociateCtx *ctx = &s_assoc_ctx[pair_idx];

    /* 空闲态不执行任何动作 */
    if (ctx->state == BLOCK_ASSOCIATE_IDLE) {
        return;
    }

    /* ── 根据 pair_idx 和 dir_id 确定供给方(armA)和接收方(armB) ──
     * dir_id=0 (L→R): armA=左侧臂(LF/LB), armB=右侧臂(RF/RB)
     * dir_id=1 (R→L): armA=右侧臂(RF/RB), armB=左侧臂(LF/LB)         */
    arm_id_e armA, armB;

    if (pair_idx == 0) {
        /* 前侧交接对: LF ↔ RF */
        if (dir_id == ARM_DIR_L_TO_R) {
            /* 左→右: LF(供给方) → RF(接收方) */
            armA = ARM_ID_LF;
            armB = ARM_ID_RF;
        } else {
            /* 右→左: RF(供给方) → LF(接收方) */
            armA = ARM_ID_RF;
            armB = ARM_ID_LF;
        }
    } else {
        /* 后侧交接对: LB ↔ RB */
        if (dir_id == ARM_DIR_L_TO_R) {
            /* 左→右: LB(供给方) → RB(接收方) */
            armA = ARM_ID_LB;
            armB = ARM_ID_RB;
        } else {
            /* 右→左: RB(供给方) → LB(接收方) */
            armA = ARM_ID_RB;
            armB = ARM_ID_LB;
        }
    }
    (void)dir_id;

    /* ── 状态机主分支 ── */
    switch (ctx->state)
    {
        case BLOCK_ASSOCIATE_TO_MIDDLE:
            /* Step 1: 双方机械臂同时向交接中间位置移动.
             * 实际轨迹规划由 controlA_loop 完成, 此处仅设置目标点.
             * 等待 MOVE_DELAY 时间后视为到位, 转入下一步.
             * 注意: 前侧(pair=0)与后侧(pair=1)使用不同的 X 坐标,
             * 因为 LF/RF 工作空间在 X 正半轴, LB/RB 在 X 负半轴. */
            {
                float mid_x = (pair_idx == 0) ? ASSOC_MID_X_FRONT : ASSOC_MID_X_REAR;
                int idxA = arm_id_to_idx(armA);
                int idxB = arm_id_to_idx(armB);
                /* armA(供给方)取 +Y, armB(接收方)取 -Y, 双方在同一 X=mid_x 处会合 */
                if (idxA >= 0) { target_x_test[idxA] = mid_x;  target_y_test[idxA] =  ASSOC_MID_Y; }
                if (idxB >= 0) { target_x_test[idxB] = mid_x;  target_y_test[idxB] = -ASSOC_MID_Y; }
            }
            if ((HAL_GetTick() - ctx->enter_tick) >= ASSOC_MOVE_DELAY_MS) {
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_WAIT, ASSOC_TIMEOUT_MS);
            }
            break;

        case BLOCK_ASSOCIATE_WAIT:
            /* Step 2: 等待双方机械臂稳定就绪.
             * 当前实现采用固定延时, 后续可替换为基于末端位置误差的判断逻辑. */
            if ((HAL_GetTick() - ctx->enter_tick) >= ASSOC_WAIT_DELAY_MS) {
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_ADSORB, ASSOC_ADSORB_TIMEOUT_MS);
            }
            break;

        case BLOCK_ASSOCIATE_ADSORB:
            /* Step 3: 等待吸附确认. 本状态不控制电磁阀, 接收方真空应在
             * 更早阶段（如 TO_MIDDLE）已建立.
             * 通过超时（或未来取消注释的微动开关 g_switch_input）检测
             * 物块是否已被接收方吸盘吸附, 超时后仍继续推进（容错处理）. */
            // if (g_switch_input.state[armB] != 0) {
            //     ctx->block_grabbed = true;
            // }
            // if (ctx->block_grabbed) {
            //     assoc_set_state(pair_idx, BLOCK_ASSOCIATE_VALVE_CONTROL, ASSOC_TIMEOUT_MS);
            // }
            if (assoc_is_timed_out(ctx)) {
                /* 即使未检测到吸附成功, 仍继续推进, 避免流程卡死 */
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_VALVE_CONTROL, 1500U);
            }
            break;

        case BLOCK_ASSOCIATE_VALVE_CONTROL:
            /* Step 4: 断开供给方电磁阀, 释放物块.
             * 接收方吸盘真空应在更早阶段（如 TO_MIDDLE）已建立,
             * 因此只需关闭供给方, 接收方即可自然吸附物块. */
            relay_control((uint8_t)armA, 0);
            if (assoc_is_timed_out(ctx)) {
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_COMPLETE, ASSOC_TIMEOUT_MS);
            }
            break;

        case BLOCK_ASSOCIATE_COMPLETE:
            /* Step 5: 交接完成, 双方机械臂移动到各自的安全位置.
             * 短暂保持后自动回到 IDLE, 准备接受下一次交接指令. */
            {
                int idxA = arm_id_to_idx(armA);
                int idxB = arm_id_to_idx(armB);
                if (idxA >= 0) {
                    target_x_test[idxA] = ASSOC_SAFE_X[(int)armA];
                    target_y_test[idxA] = ASSOC_SAFE_Y[(int)armA];
                }
                if (idxB >= 0) {
                    target_x_test[idxB] = ASSOC_SAFE_X[(int)armB];
                    target_y_test[idxB] = ASSOC_SAFE_Y[(int)armB];
                }
            }

            /* 恢复供给方电磁阀状态 (重新建立真空) */
            relay_control((uint8_t)armA, 1);

            if ((HAL_GetTick() - ctx->enter_tick) >= ASSOC_HOLD_MS) {
                ctx->block_grabbed = false;
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_IDLE, 0);
            }
            break;

        default:
            /* 未知状态防御性处理: 回退到 IDLE */
            assoc_set_state(pair_idx, BLOCK_ASSOCIATE_IDLE, 0);
            break;
    }

    /* 兜底超时保护: 状态机代码执行完毕后若仍未跳转且超时, 回退到 IDLE.
     * 此检查放在 switch 之后, 确保状态机自身的跳转逻辑
     * (如 ADSORB→VALVE_CONTROL) 优先执行, 不会被前置超时检查截断.     */
    if (ctx->state != BLOCK_ASSOCIATE_IDLE && assoc_is_timed_out(ctx)) {
        assoc_set_state(pair_idx, BLOCK_ASSOCIATE_IDLE, 0);
    }
}

/* ════════════════════════════════════════════════════════════════
 * 遥控器指令接收
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 接收遥控器控制指令, 在满足条件时触发交接流程
 *
 * 当前逻辑: 当 DT7 遥控器 s[0]==1 且 s[1]==1 且 ch[0]>400 时,
 * 触发后侧交接对 (LB↔RB) 的左→右方向交接.
 *
 * 此函数保留原始实现逻辑不变, 仅独立为单独函数以便维护和替换.
 */
void ACTION_recvie(void)
{
    if (get_remote_control_point()->rc.s[0] == 1 && get_remote_control_point()->rc.s[1] == 1)
    {
        if (get_remote_control_point()->rc.ch[0] > 400)
        {
            /* 触发后侧交接对(LB↔RB), 方向: 左→右 */
            associate_trigger(1, ARM_DIR_L_TO_R);
        }
    }
}

/* ════════════════════════════════════════════════════════════════
 * 对外接口实现
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 周期调用: 驱动所有动作状态机推进
 *
 * 执行顺序:
 *   1. ACTION_recvie() — 接收遥控器指令
 *   2. 根据 g_action_state 分派到对应子状态机
 *   3. 同步外部查询用的全局状态变量
 *   4. 当所有交接对都 IDLE 时, 恢复全局状态为 IDLE
 */
void ACTION_loop(void)
{
    /* Step 1: 接收遥控器指令, 可能触发新的交接 */
    ACTION_recvie();

    /* Step 2: 根据全局动作状态分派 */
    switch (g_action_state) {
        case ACTION_STATE_IDLE:
            /* 空闲态: 不执行任何动作 */
            break;

        case ACTION_STATE_ASSOCIATE:
            /* 驱动前侧交接对: 使用该对内部存储的 dir_id */
            associate_run_one_pair(0, s_assoc_ctx[0].dir_id);
            /* 驱动后侧交接对 */
            associate_run_one_pair(1, s_assoc_ctx[1].dir_id);
            break;

        /* ── 扩展示例 (取消注释并实现对应函数即可) ──
        case ACTION_STATE_HOME:
            home_state_machine();
            break;
        case ACTION_STATE_DANCE:
            dance_state_machine();
            break;
        ───────────────────────────────────────────── */

        default:
            /* 未知状态防御: 回退到 IDLE */
            g_action_state = ACTION_STATE_IDLE;
            break;
    }

    /* Step 3: 同步外部查询用的全局状态变量
     * 保持与原 LEFT/RIGHT 全局变量兼容, 外部模块可通过
     * LEFT/RIGHT 变量或 action_get_left_state()/action_get_right_state() 查询 */
    g_left_state = s_assoc_ctx[0].state;
    g_right_state = s_assoc_ctx[1].state;
    LEFT  = g_left_state;
    RIGHT = g_right_state;

    /* Step 4: 当所有交接对都回到 IDLE, 恢复全局动作状态 */
    if (assoc_all_idle()) {
        g_action_state = ACTION_STATE_IDLE;
    }
}

/**
 * @brief 触发指定交接对开始物块移交流程
 *
 * @param pair_idx  交接对索引: 0=前侧(LF↔RF), 1=后侧(LB↔RB)
 * @param dir_id    交接方向: ARM_DIR_L_TO_R(0)=左→右, ARM_DIR_R_TO_L(1)=右→左
 * @retval true   成功触发
 * @retval false  触发失败 (交接对正忙或参数非法)
 */
bool associate_trigger(uint8_t pair_idx, uint8_t dir_id)
{
    if (pair_idx >= 2 || dir_id > 1) {
        return false;
    }
    if (s_assoc_ctx[pair_idx].state != BLOCK_ASSOCIATE_IDLE) {
        /* 当前交接流程尚未完成, 拒绝重复触发 */
        return false;
    }

    /* 记录本次交接的方向, 供 associate_run_one_pair 使用 */
    s_assoc_ctx[pair_idx].dir_id = dir_id;

    /* 切换到交接中状态, 全局动作为 ASSOCIATE */
    g_action_state = ACTION_STATE_ASSOCIATE;
    assoc_set_state(pair_idx, BLOCK_ASSOCIATE_TO_MIDDLE, ASSOC_TIMEOUT_MS);
    return true;
}

/**
 * @brief 强制中止指定交接对的当前流程
 * @param pair_idx  交接对索引: 0=前侧, 1=后侧
 */
void associate_abort(uint8_t pair_idx)
{
    if (pair_idx >= 2) {
        return;
    }
    AssociateCtx *ctx = &s_assoc_ctx[pair_idx];
    ctx->block_grabbed = false;
    assoc_set_state(pair_idx, BLOCK_ASSOCIATE_IDLE, 0);

    /* 若所有交接对均回到 IDLE, 恢复全局状态 */
    if (assoc_all_idle()) {
        g_action_state = ACTION_STATE_IDLE;
    }
}

/**
 * @brief 查询指定交接对的当前子状态
 * @param pair_idx  交接对索引: 0=前侧, 1=后侧
 * @return 当前 block_associate_state_e 枚举值的 uint8_t 表示
 */
uint8_t associate_get_state(uint8_t pair_idx)
{
    if (pair_idx >= 2) {
        return (uint8_t)BLOCK_ASSOCIATE_IDLE;
    }
    return (uint8_t)s_assoc_ctx[pair_idx].state;
}

/**
 * @brief 查询全局动作状态机的当前状态
 * @return 当前 action_state_e 枚举值
 */
action_state_e action_get_global_state(void)
{
    return g_action_state;
}

/**
 * @brief 获取全局动作状态指针
 * @return 指向 g_action_state 的指针
 */
const volatile action_state_e *action_get_state_ptr(void)
{
    return &g_action_state;
}

/**
 * @brief 获取前侧交接对状态 (外部查询用)
 * @return 前侧(pair_idx=0)的 block_associate_state_e 值
 */
block_associate_state_e action_get_left_state(void)
{
    return LEFT;
}

/**
 * @brief 获取后侧交接对状态 (外部查询用)
 * @return 后侧(pair_idx=1)的 block_associate_state_e 值
 */
block_associate_state_e action_get_right_state(void)
{
    return RIGHT;
}
