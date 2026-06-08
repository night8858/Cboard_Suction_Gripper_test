/**
 * @file    input_arbiter.c
 * @brief   输入仲裁器实现 —— PC/RC 优先级合并与路径点平滑
 *
 * 本模块从 Planar_Robot_Arm.c 的 controlA_loop() 中提取遥控器处理逻辑,
 * 遵循单一职责原则, 将输入处理与机械臂运动学解耦.
 *
 * 原始 DT7 遥控器处理代码迁移自 Planar_Robot_Arm.c controlA_loop(),
 * 保留原有逻辑不变, 仅封装为独立函数.
 */

#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include "input_arbiter.h"
#include "Planar_Robot_Arm.h"
#include "Dof4_Arm.h"
#include "pneumatic_control.h"
#include "action_scheduler.h"
#include "DT7.h"
#include "stm32f4xx_hal.h"
#include "variables.h"

/* ── 外部引用: 目标数组由 Planar_Robot_Arm.c 定义 ── */
extern float target_x_test[4];
extern float target_y_test[4];

/* ════════════════════════════════════════════════════════════════
 * 预定义目标位置 (迁移自 Planar_Robot_Arm.c 的 TARGET_P0~P4)
 *
 * 坐标定义 (外部坐标系, 单位 mm):
 *   P0 — 归位/收拢位 (各臂靠近机体)
 *   P1 — 伸直吸取物块位
 *   P2 — 携带物块位
 *   P3 — 伸直放置物块位
 *   P4 — 放置中途过渡位 (P2→P3 的中继点, 避免轨迹突变)
 *
 * 数组索引: [ARM_ID_LF, ARM_ID_RF, ARM_ID_LB, ARM_ID_RB]
 * ════════════════════════════════════════════════════════════════ */

static const float TARGET_P0_X[4] = {  200.0f,  200.0f, -200.0f, -200.0f };
static const float TARGET_P0_Y[4] = {  240.0f, -240.0f,  240.0f, -240.0f };

static const float TARGET_P1_X[4] = {  425.0f,  425.0f, -425.0f, -425.0f };
static const float TARGET_P1_Y[4] = {  425.0f, -425.0f,  425.0f, -425.0f };

static const float TARGET_P2_X[4] = {  250.0f,  250.0f, -250.0f, -250.0f };
static const float TARGET_P2_Y[4] = {  400.0f, -400.0f,  400.0f, -400.0f };

static const float TARGET_P3_X[4] = {   60.0f,   60.0f,  -60.0f,  -60.0f };
static const float TARGET_P3_Y[4] = {  900.0f, -900.0f,  900.0f, -900.0f };

static const float TARGET_P4_X[4] = {  280.0f,  280.0f, -280.0f, -280.0f };
static const float TARGET_P4_Y[4] = {  600.0f, -600.0f,  600.0f, -600.0f };

/* ════════════════════════════════════════════════════════════════
 * 输入缓冲区
 * ════════════════════════════════════════════════════════════════ */

/** @brief 标记 PC 缓冲区是否已收到有效数据 (上电后初始为 false) */
static bool s_pc_data_valid[4] = { false, false, false, false };

/** @brief PC 输入缓冲区 (由 input_arbiter_update_pc 写入) */
static float s_pc_target_x[4];
static float s_pc_target_y[4];

/** @brief RC 遥控器原始数据快照 (由 input_arbiter_update_rc 写入) */
static RC_ctrl_t s_rc_snapshot;

/** @brief RC 数据是否已初始化 (上电后首次收到有效帧后置 true) */
static bool s_rc_initialized = false;

/** @brief 输入源最后更新时间戳 (ms, 来自 HAL_GetTick)
 *
 * s_rc_last_feed_ms  — 最后一次 input_arbiter_update_rc() 调用时刻
 * s_pc_last_feed_ms  — 最后一次 input_arbiter_update_pc() 调用时刻
 *
 * 用于实现思路 A (数据新鲜度看门狗):
 *   若 (当前时刻 - 最后更新时间) > INPUT_FRESHNESS_TIMEOUT_MS,
 *   则认为该来源数据过期, 拒绝消费.
 *
 * 冷启动时两个时间戳均为 0, is_ready() 返回 false,
 * 直到至少一个来源被首次更新 (思路 B: 初始化就绪门).            */
static uint32_t s_rc_last_feed_ms = 0;
static uint32_t s_pc_last_feed_ms = 0;

/** @brief PC 是否收到过有效数据 (用于 is_ready 判断) */
static bool s_pc_ever_valid = false;

/* ════════════════════════════════════════════════════════════════
 * 路径点状态机 (迁移自 controlA_loop)
 *
 * P2→P3 切换时先经过 P4, 停留 WP_HOLD_MS 后再前进到 P3,
 * 避免末端轨迹突变导致机械结构冲击.
 * ════════════════════════════════════════════════════════════════ */

/** @brief 路径点过渡停留时间 (ms) */
#define WP_HOLD_MS  80U

/** @brief 路径点状态: 0=idle/P2, 1=P4 中继, 2=P3 目标 */
static uint8_t  s_wp_phase[4] = { 0, 0, 0, 0 };
static uint32_t s_wp_tick[4]  = { 0, 0, 0, 0 };

/**
 * @brief 路径点状态机: P2→P3 平滑过渡
 *
 * 对每个臂独立检查: 若当前目标为 P3(伸直放置位),
 * 则先经过 P4 中继点停留 WP_HOLD_MS, 再前进到 P3.
 *
 * 此函数直接修改 target_x_test/y_test[] 的内容.
 */
static void waypoint_smooth_filter(void)
{
    for (int i = 0; i < 4; i++)
    {
        /* 判断当前期望目标是否为 P3 (伸直放置位) */
        bool want_p3 = (fabsf(target_x_test[i] - TARGET_P3_X[i]) < 1.0f &&
                        fabsf(target_y_test[i] - TARGET_P3_Y[i]) < 1.0f);

        if (want_p3)
        {
            if (s_wp_phase[i] == 0)
            {
                /* 刚从 P2 切换到 P3 → 先发往 P4 */
                s_wp_phase[i] = 1;
                s_wp_tick[i]  = HAL_GetTick();
                target_x_test[i] = TARGET_P4_X[i];
                target_y_test[i] = TARGET_P4_Y[i];
            }
            else if (s_wp_phase[i] == 1)
            {
                if ((HAL_GetTick() - s_wp_tick[i]) >= WP_HOLD_MS)
                {
                    /* P4 停留时间到 → 前进到 P3 */
                    s_wp_phase[i] = 2;
                    target_x_test[i] = TARGET_P3_X[i];
                    target_y_test[i] = TARGET_P3_Y[i];
                }
                else
                {
                    /* 仍在 P4, 保持目标 */
                    target_x_test[i] = TARGET_P4_X[i];
                    target_y_test[i] = TARGET_P4_Y[i];
                }
            }
            /* phase == 2: 目标已是 P3, 保持不变 */
        }
        else
        {
            /* 目标不是 P3 → 复位状态机, 允许下次 P2→P3 时重新触发 */
            s_wp_phase[i] = 0;
        }
    }
}

/* ════════════════════════════════════════════════════════════════
rc_map_to_targets() 指令一览:
  s[1]==3:
    s[0]==1  → LF/RF 单臂 P2↔P3 + 电磁阀 ch2
    s[0]==2  → RF/LB 单臂 P2↔P3 + 电磁阀 ch2
    s[0]==3  → 四臂群控 P1↔P2
  s[0]==2, s[1]==2:
    ch[3] 上升沿 → 切换气泵启停
  s[0]==1, s[1]==1:              ← 新迁移
    ch[0] > 400 → 触发后侧交接(LB↔RB, 左→右)
 */
/** @brief 遥控器摇杆/拨杆阈值: 绝对值超过此值视为有效操作 */
#define RC_CH_THRESHOLD  400
#define DOF4_RC_DEADZONE 200
#define DOF4_RC_MANUAL_SPEED_MPS 0.25f
#define DOF4_RC_MANUAL_FALLBACK_DT_MS 5U
#define DOF4_RC_MANUAL_MAX_DT_MS 50U

/**
 * @brief 将 DT7 遥控器数据转换为目标位置并写入 target_x_test/y_test
 *
 * 仅在 s_rc_initialized=true 时执行.
 * 本函数不检查 action_active, 由调用方(input_arbiter_resolve)决定是否调用.
 */
static void rc_map_to_targets(void)
{
    if (!s_rc_initialized) {
        return;
    }

    const RC_ctrl_t *rc = &s_rc_snapshot;

    /* 拨杆 s[1]==3 时进入手动控制模式 */
    if (rc->rc.s[1] == 3)
    {
        if (rc->rc.s[0] == 1)
        {
            /* ── s[0]==1: 单独控制 LF(前左侧)臂 ── */
            if (rc->rc.ch[0] < -RC_CH_THRESHOLD)
            {
                target_x_test[0] = TARGET_P3_X[0];
                target_y_test[0] = TARGET_P3_Y[0];
            }
            else
            {
                target_x_test[0] = TARGET_P2_X[0];
                target_y_test[0] = TARGET_P2_Y[0];
            }

            /* 摇杆反向: 控制 RF 臂 */
            if (rc->rc.ch[0] > RC_CH_THRESHOLD)
            {
                target_x_test[1] = TARGET_P3_X[1];
                target_y_test[1] = TARGET_P3_Y[1];
            }
            else
            {
                target_x_test[1] = TARGET_P2_X[1];
                target_y_test[1] = TARGET_P2_Y[1];
            }

            /* 电磁阀控制: ch2 映射到 relay 0/1 */
            if (rc->rc.ch[2] < -RC_CH_THRESHOLD)
            {
                relay_control(0, 0);
            }
            else
            {
                relay_control(0, 1);
            }

            if (rc->rc.ch[2] > RC_CH_THRESHOLD)
            {
                relay_control(1, 0);
            }
            else
            {
                relay_control(1, 1);
            }
        }
        else if (rc->rc.s[0] == 2)
        {
            /* ── s[0]==2: 单独控制 RF/LB/RB 臂 ── */
            if (rc->rc.ch[0] < - RC_CH_THRESHOLD)
            {
                target_x_test[2] = TARGET_P3_X[2];
                target_y_test[2] = TARGET_P3_Y[2];
            }
            else
            {
                target_x_test[2] = TARGET_P2_X[2];
                target_y_test[2] = TARGET_P2_Y[2];
            }

            if (rc->rc.ch[0] > RC_CH_THRESHOLD)
            {
                target_x_test[3] = TARGET_P3_X[3];
                target_y_test[3] = TARGET_P3_Y[3];
            }
            else
            {
                target_x_test[3] = TARGET_P2_X[3];
                target_y_test[3] = TARGET_P2_Y[3];
            }

            /* 电磁阀控制: ch2 映射到 relay 2/3 */
            if (rc->rc.ch[2] < -RC_CH_THRESHOLD)
            {
                relay_control(2, 0);
            }
            else
            {
                relay_control(2, 1);
            }

            if (rc->rc.ch[2] > RC_CH_THRESHOLD)
            {
                relay_control(3, 0);
            }
            else
            {
                relay_control(3, 1);
            }
        }
        else if (rc->rc.s[0] == 3)
        {
            /* ── s[0]==3: 四臂群控, P1/P2 之间切换 ── */
            if (rc->rc.ch[0] > RC_CH_THRESHOLD)
            {
                for (int i = 0; i < 4; i++) {
                    target_x_test[i] = TARGET_P1_X[i];
                    target_y_test[i] = TARGET_P1_Y[i];
                }
            }
            else
            {
                for (int i = 0; i < 4; i++) {
                    target_x_test[i] = TARGET_P2_X[i];
                    target_y_test[i] = TARGET_P2_Y[i];
                }
            }
        }
    }
            /* ════════════════════════════════════════════════════════
         * 气泵手动控制: s[0]==1 且 s[1]==1 时,
         * ch[3] 上升沿 (>RC_CH_THRESHOLD) 切换气泵启停.
         *
         * 使用 static 变量记录 ch[3] 上一次电平状态,
         * 实现边沿检测: 仅当 ch[3] 从低→高跳变时触发一次,
         * 避免摇杆持续推高时反复切换导致气泵抖动.            */
    if (rc->rc.s[0] == 2 )
    {
        if(rc->rc.s[1] == 2)
        {
            static bool s_ch3_was_high = false;
            bool ch3_high = (rc->rc.ch[3] > RC_CH_THRESHOLD);
            /* 上升沿: 上一次 ≤ 阈值, 本次 > 阈值 → 切换 */
            if (ch3_high && !s_ch3_was_high) {
                extern PumpCtrl g_pump;  /* pneumatic_control.c 定义 */
                pump_ctrl_toggle(&g_pump);
            }
            s_ch3_was_high = ch3_high;
            }
    }

        /* ════════════════════════════════════════════════════════
         * 物块交接触发: s[0]==1 且 s[1]==1 时,
         * ch[0] 高电平 (>RC_CH_THRESHOLD) 触发后侧交接对
         * (LB↔RB), 方向: 左→右.
         *
         * 此逻辑从 action_scheduler.c 的 ACTION_recvie() 迁移,
         * 统一所有 RC 遥控器指令在 rc_map_to_targets() 中处理.  */
    if (rc->rc.s[0] == 1 && rc->rc.s[1] == 1) {
        if (rc->rc.ch[0] > RC_CH_THRESHOLD) {
            associate_trigger(1, ARM_DIR_L_TO_R);
        }
        else if (rc->rc.ch[0] < -RC_CH_THRESHOLD) {
            associate_trigger(1, ARM_DIR_R_TO_L);
        }
        if (rc->rc.ch[2] > RC_CH_THRESHOLD) {
            associate_trigger(0, ARM_DIR_L_TO_R);
        }
        else if (rc->rc.ch[2] < -RC_CH_THRESHOLD) {
            associate_trigger(0, ARM_DIR_R_TO_L);
        }
    }
    
}

/* ════════════════════════════════════════════════════════════════
 * PC 指令 → 目标位置映射
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 将 PC 缓冲区中的有效数据写入 target_x_test/y_test
 *
 * 仅当对应臂的 s_pc_data_valid[i]==true 时才写入.
 * 写入后清除 valid 标志 (边沿触发, 防止同一指令反复覆盖).
 */
static void pc_map_to_targets(void)
{
    for (int i = 0; i < 4; i++) {
        if (s_pc_data_valid[i]) {
            target_x_test[i] = s_pc_target_x[i];
            target_y_test[i] = s_pc_target_y[i];
            /* 单次消费: 写入后清除, 等待下一次 PC 指令 */
            s_pc_data_valid[i] = false;
        }
    }
}

/* ════════════════════════════════════════════════════════════════
 * 对外接口实现
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief 初始化输入仲裁器
 */
void input_arbiter_init(void)
{
    s_rc_initialized = false;
    s_pc_ever_valid  = false;
    s_rc_last_feed_ms = 0;
    s_pc_last_feed_ms = 0;
    for (int i = 0; i < 4; i++) {
        s_pc_data_valid[i] = false;
        s_pc_target_x[i]   = 0.0f;
        s_pc_target_y[i]   = 0.0f;
        s_wp_phase[i]      = 0;
        s_wp_tick[i]       = 0;
    }
}

/**
 * @brief 更新遥控器(RC)输入缓冲区
 * @param rc  指向 RC_ctrl_t 的指针
 */
void input_arbiter_update_rc(const RC_ctrl_t *rc)
{
    if (rc == NULL) {
        return;
    }
    /* 全量拷贝: RC_ctrl_t 结构体较小 (约 20 字节), 拷贝开销可忽略 */
    s_rc_snapshot   = *rc;
    s_rc_initialized = true;
    /* 记录本次喂入时间戳, 供新鲜度看门狗使用 (思路 A) */
    s_rc_last_feed_ms = HAL_GetTick();
}

/**
 * @brief 更新上位机(PC)输入缓冲区
 * @param cmd  指向 all_pc_command 的指针
 */
void input_arbiter_update_pc(const all_pc_command *cmd)
{
    if (cmd == NULL) {
        return;
    }

    /* 将 all_pc_command 中的四臂目标位置存入 PC 缓冲区.
     * 使用 isnan()/isinf() 检测无效值, 仅有效值才置 valid.
     * 注意: 此处的 LF/RF/LB/RB 直接映射到 target 数组索引 0/1/2/3. */

    float val;

    val = cmd->LF_target_x;
    if (!isnan(val) && !isinf(val)) { s_pc_target_x[0] = val; s_pc_data_valid[0] = true; }
    val = cmd->LF_target_y;
    if (!isnan(val) && !isinf(val)) { s_pc_target_y[0] = val; s_pc_data_valid[0] = true; }

    val = cmd->RF_target_x;
    if (!isnan(val) && !isinf(val)) { s_pc_target_x[1] = val; s_pc_data_valid[1] = true; }
    val = cmd->RF_target_y;
    if (!isnan(val) && !isinf(val)) { s_pc_target_y[1] = val; s_pc_data_valid[1] = true; }

    val = cmd->LB_target_x;
    if (!isnan(val) && !isinf(val)) { s_pc_target_x[2] = val; s_pc_data_valid[2] = true; }
    val = cmd->LB_target_y;
    if (!isnan(val) && !isinf(val)) { s_pc_target_y[2] = val; s_pc_data_valid[2] = true; }

    val = cmd->RB_target_x;
    if (!isnan(val) && !isinf(val)) { s_pc_target_x[3] = val; s_pc_data_valid[3] = true; }
    val = cmd->RB_target_y;
    if (!isnan(val) && !isinf(val)) { s_pc_target_y[3] = val; s_pc_data_valid[3] = true; }

    /* 若本次至少有一个臂的坐标有效, 标记 PC 已就绪并更新时间戳 */
    for (int i = 0; i < 4; i++) {
        if (s_pc_data_valid[i]) {
            s_pc_ever_valid   = true;
            s_pc_last_feed_ms = HAL_GetTick();
            break;
        }
    }
}

static void rc_map_to_targets_4dof(uint32_t now_ms, bool arm_started)
{
    if (!s_rc_initialized) {
        return;
    }

    const RC_ctrl_t *rc = &s_rc_snapshot;
    static uint32_t s_last_step_ms = 0U;

    /* ── 启动门控：未启动时仅处理启动触发 + 气泵控制，
     *             拒绝所有手动控制、动作触发等运动指令 ── */
    if (!arm_started) {
        if (rc->rc.s[0] == 2 && rc->rc.s[1] == 2) {
            static bool s_ch3_was_high = false;
            bool ch3_high = (rc->rc.ch[3] > RC_CH_THRESHOLD);
            if (ch3_high && !s_ch3_was_high) {
                extern PumpCtrl g_pump;
                pump_ctrl_toggle(&g_pump);
            }
            s_ch3_was_high = ch3_high;
            Dof4_double_arm_start();
        }
        return;
    }

    /* 拨杆 s[1]==3 时进入手动控制模式 */
    if (rc->rc.s[1] == 3 && rc->rc.s[0] == 1) {
        const bool x_active = (abs(rc->rc.ch[3]) > DOF4_RC_DEADZONE);
        const bool y_active = (abs(rc->rc.ch[2]) > DOF4_RC_DEADZONE);
        const bool z_active = (abs(rc->rc.ch[1]) > DOF4_RC_DEADZONE);
        if (!x_active && !y_active && !z_active) {
            s_last_step_ms = now_ms;
            return;
        }

        uint32_t dt_ms = (s_last_step_ms == 0U)
                             ? DOF4_RC_MANUAL_FALLBACK_DT_MS
                             : (uint32_t)(now_ms - s_last_step_ms);
        if (dt_ms > DOF4_RC_MANUAL_MAX_DT_MS) {
            dt_ms = DOF4_RC_MANUAL_MAX_DT_MS;
        }
        s_last_step_ms = now_ms;

        Dof4_Pose target = g_dof4_arm_right.target_valid
                               ? g_dof4_arm_right.target_pose
                               : g_dof4_arm_right.current_pose;
        const float step = DOF4_RC_MANUAL_SPEED_MPS * ((float)dt_ms * 0.001f);

        if (x_active && rc->rc.ch[3] > 0) {
            target.x += step;
        } else if (x_active) {
            target.x -= step;
        }

        if (y_active && rc->rc.ch[2] > 0) {
            target.y += step;
        } else if (y_active) {
            target.y -= step;
        }

        if (z_active && rc->rc.ch[1] > 0) {
            target.z += step;
        } else if (z_active) {
            target.z -= step;
        }

        Dof4_clamp_to_workspace(&g_dof4_arm_right, &target);
        (void)Dof4_arm_set_target(&g_dof4_arm_right,
                                  target.x,
                                  target.y,
                                  target.z,
                                  target.pitch);
    } 
    else if (rc->rc.s[1] == 3 && rc->rc.s[0] == 3)
    {
        //固定位置测试的程序
            if (rc->rc.ch[0] < - RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_GET_FORWARD_LEFT_ARM);     //done

            }

            if (rc->rc.ch[0] > RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_GET_FORWARD_RIGHT_ARM);       //done
            }

            if (rc->rc.ch[1] < - RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F1);
            }

            if (rc->rc.ch[1] > RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F1);
            }

            if (rc->rc.ch[2] < - RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_LEFT_ARM);
            }

            if (rc->rc.ch[2] > RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_RIGHT_ARM);
            }

            if (rc->rc.ch[3] < - RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_GET_FORWARD);

            }
            if (rc->rc.ch[3] > RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_PLACE_BACK);
            }

    }
    else if (rc->rc.s[1] == 1 && rc->rc.s[0] == 1)
    {

        if (rc->rc.ch[0] < - RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_GET_FORWARD);

            }

            if (rc->rc.ch[0] > RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_PLACE_BACK);
            }


            if (rc->rc.ch[1] < - RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_GET_LEFT_BACK_TO_HAND_LEFT_ARM);
            }

            if (rc->rc.ch[1] > RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_GET_RIGHT_BACK_TO_HAND_RIGHT_ARM);
            }
            if (rc->rc.ch[2] < - RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F1);
            }

            if (rc->rc.ch[2] > RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F1);
            }

            if (rc->rc.ch[3] < - RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_PLACE_LEFT_ARM_TO_LEFT_POINT1_F2);
            }

            if (rc->rc.ch[3] > RC_CH_THRESHOLD)
            {
                action_4dof_trigger(ACTION_BLOCK_PLACE_RIGHT_ARM_TO_RIGHT_POINT1_F2);
            }
    }

             /* 气泵手动控制: s[0]==1 且 s[1]==1 时,
         * ch[3] 上升沿 (>RC_CH_THRESHOLD) 切换气泵启停.
         *
         * 使用 static 变量记录 ch[3] 上一次电平状态,
         * 实现边沿检测: 仅当 ch[3] 从低→高跳变时触发一次,
         * 避免摇杆持续推高时反复切换导致气泵抖动.            */
    else if (rc->rc.s[0] == 2 )
    {
        if(rc->rc.s[1] == 2)
        {
            static bool s_ch3_was_high = false;
            bool ch3_high = (rc->rc.ch[3] > RC_CH_THRESHOLD);
            /* 上升沿: 上一次 ≤ 阈值, 本次 > 阈值 → 切换 */
            if (ch3_high && !s_ch3_was_high) {
                extern PumpCtrl g_pump;  /* pneumatic_control.c 定义 */
                pump_ctrl_toggle(&g_pump);
            }
            s_ch3_was_high = ch3_high;
        }

            Dof4_double_arm_start();
    }


    else {
        s_last_step_ms = now_ms;
    }
}

void input_arbiter_resolve_4dof(bool action_active, bool arm_started)
{
    if (action_active) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    const bool rc_fresh = s_rc_initialized &&
                          ((now - s_rc_last_feed_ms) < INPUT_FRESHNESS_TIMEOUT_MS);
    if (rc_fresh) {
        rc_map_to_targets_4dof(now, arm_started);
    }
}

/**
 * @brief 执行输入仲裁: PC(高优先级) > RC(低优先级)
 *
 * 仲裁规则:
 *   1. action_active=true 时直接返回 (target 由 ACTION_loop 接管).
 *   2. 优先消费 PC 缓冲区中的有效数据.
 *   3. PC 无数据时降级到 RC 映射.
 *   4. 最后执行路径点平滑滤波器.
 *
 * @param action_active  当前是否有动作正在执行
 */
void input_arbiter_resolve(bool action_active)
{
    /* 动作激活时, target 数组由 ACTION_loop 写入, 跳过输入仲裁 */
    if (action_active) {
        return;
    }

    /* ════════════════════════════════════════════════════════════
     * 思路 A: 数据新鲜度看门狗 (Data Freshness Watchdog)
     *
     * 检查 RC 和 PC 两个输入源的最后更新时间.
     * 若两个来源均过期 (超过 INPUT_FRESHNESS_TIMEOUT_MS 无更新),
     * 则本轮不修改 target_x_test/y_test, 机械臂保持在上一个
     * 有效指令位置不动.
     *
     * 这解决了:
     *   - RC 掉线后快照残留导致持续运动的问题
     *   - PC 断连后回退到过期 RC/无数据的问题
     * ════════════════════════════════════════════════════════════ */
    uint32_t now = HAL_GetTick();
    bool rc_fresh = s_rc_initialized &&
                    ((now - s_rc_last_feed_ms) < INPUT_FRESHNESS_TIMEOUT_MS);
    bool pc_fresh = s_pc_ever_valid &&
                    ((now - s_pc_last_feed_ms) < INPUT_FRESHNESS_TIMEOUT_MS);

    /* uint32 减法对 HAL_GetTick 溢出是安全的 (模 2^32 运算) */

    if (!rc_fresh && !pc_fresh) {
        /* 两个输入源均过期: 本轮不修改 target,
         * 机械臂保持在最后有效指令位置.
         * waypoint_smooth_filter 也跳过 (没有新目标无需平滑).   */
        return;
    }

    /*
     * 优先级: PC > RC
     * PC 优先仅在 PC 数据新鲜时生效; 若 PC 过期但 RC 新鲜,
     * 则降级到 RC 映射.                                        */
    if (pc_fresh) {
        /* PC 模式: 仅消费已 valid 的臂, 未 valid 的臂保持 target 不变 */
        pc_map_to_targets();
    } else if (rc_fresh) {
        /* RC 模式: 遥控器数据映射到 target 数组 (保留原有 DT7 控制逻辑) */
        rc_map_to_targets();
    }
    /* 两者都过期已在上面 return, 此处不会到达 else 分支 */

    /* 路径点平滑: P2→P3 切换通过 P4 中继 */
    waypoint_smooth_filter();
}

/**
 * @brief 查询输入仲裁器是否已就绪 (思路 B: 初始化就绪门)
 *
 * 就绪条件: RC 或 PC 中至少一个来源收到过有效数据且处于新鲜期内.
 *
 * 冷启动时 RC DMA 尚未收到第一帧、PC 也未连接,
 * s_rc_last_feed_ms 和 s_pc_last_feed_ms 均为 0,
 * 本函数返回 false, 调用方应跳过运动控制.
 *
 * 一旦任一来源更新 (input_arbiter_update_rc/pc 被调用),
 * 对应时间戳变为非零, 本函数返回 true.
 *
 * 注意: 若运行中两个来源均掉线超过 INPUT_FRESHNESS_TIMEOUT_MS,
 *       则 is_ready() 会重新变为 false (因为两个源都过期).
 *       此时 resolve() 也会停止写入 target, 双重保护.
 *
 * @retval true   至少一个输入源就绪且新鲜
 * @retval false  无有效输入源, 应跳过运动控制
 */
bool input_arbiter_is_ready(void)
{
    uint32_t now = HAL_GetTick();
    bool rc_ready = s_rc_initialized &&
                    ((now - s_rc_last_feed_ms) < INPUT_FRESHNESS_TIMEOUT_MS);
    bool pc_ready = s_pc_ever_valid &&
                    ((now - s_pc_last_feed_ms) < INPUT_FRESHNESS_TIMEOUT_MS);
    return rc_ready || pc_ready;
}
