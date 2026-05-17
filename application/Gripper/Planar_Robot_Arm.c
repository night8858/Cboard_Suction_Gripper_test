#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "Planar_Robot_Arm.h"

//#include "my_feetech.h"
#include "DT7.h"
#include "SCSerail.h"
#include "SCS.h"
#include "SMS_STS.h"
#include "SCSCL.h"
#include "usart.h"
#include "variables.h"
#include "pneumatic_control.h"

#include "cmsis_os.h"

#include "action_scheduler.h"
#include "input_arbiter.h"


/*
 * ================================================================
 *  平面机械臂运动学参数 & 理论工作空间分析
 * ================================================================
 *
 * 【连杆几何】（ARM_TYPE_1）
 *   第一连杆（大臂）：L1 = sqrt(295.1² + 25²) ≈ 296.2 mm
 *      - 长直边 295.1 mm，短边 25 mm（几何偏角 α = atan2(25, 295.1) ≈ 4.84°）
 *   第二连杆（小臂）：L2 = 337.27 mm
 *   臂基偏置（offset_X/Y）：以机体中心为原点的各臂安装偏移
 *
 * 【理论可达工作空间】（无关节限位）
 *   最大半径：Rmax = L1 + L2 ≈ 633.5 mm
 *   最小半径：Rmin = |L1 - L2| ≈ 41.1 mm
 *   可达域   ：内外半径分别为 Rmin / Rmax 的圆环（annulus）
 *
 * 【关节限位对工作空间的约束】
 *   J1（大臂舵机）：伺服步进 794~3302（约 -110°~+110°），经 offset 后关节角 ≈ -15°~+205°
 *   J2（小臂舵机）：伺服步进 1024~3072（约 -90°~+90°），经 offset 后关节角 ≈ 0°~180°
 *   → J2 范围覆盖全折叠（θ2=180°）到全伸直（θ2=0°），圆环完整可达
 *
 * 【工作空间矩形限位】（外部坐标系，mm）
 *     LF: x∈[  0, 800], y∈[   0, 900]   （第一象限）
 *     RF: x∈[  0, 800], y∈[-900,   0]   （第四象限）
 *     LB: x∈[-800,   0], y∈[   0, 900]   （第二象限）
 *     RB: x∈[-800,   0], y∈[-900,   0]   （第三象限）
 *
 * 【有效工作空间】= 圆环 ∩ 象限矩形
 *
 * ── ASCII 示意图（俯视图，机械臂中心为原点）──
 *
 *                      Y+
 *                       ^
 *                       |900
 *              ┌────────┼────────┐
 *              │   LB   │   LF   │
 *              │        │        │
 *              │  ╭～～～╮       │
 *              │ ╱       ╲      │
 *              ││ Rmax    │     │
 *     -800 ────┼┤  633mm ├─────┤─── 800 → X+
 *              ││ Rmin   │     │
 *              │ ╲  41mm ╱     │
 *              │  ╰～～～╯       │
 *              │        │        │
 *              │   RB   │   RF  │
 *              └────────┼────────┘
 *              Y-      -900
 *
 * ================================================================
 */
/*
DH参数
*/
/* 轨迹规划固定使用五次多项式插补（关节空间平滑启停）。 */


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARM1_LINK1_LONG_EDGE_MM 295.1f
#define ARM1_LINK1_SHORT_EDGE_MM 25.0f
#define ARM2_LINK2_LONG_EDGE_MM 337.27f

#define LF 1        //左前
#define RF 2        //右前  
#define LB 3        //左后
#define RB 4        //右后


//169.6，纵向
//351.81，横向
#define offset_X  84.8f
#define offset_Y  175.9f

/////////////////////////////////////////////////

#define USE_FISH
//#define arm_FKIK_debug 
//#define movedebug 

/////////////////////////////////////////////////


// 机械臂参数配置和控制函数实现
extern Planar_Robot_Arm Arm_LF;       //左前
extern Planar_Robot_Arm Arm_RF;       //右前
extern Planar_Robot_Arm Arm_LB;       //左后
extern Planar_Robot_Arm Arm_RB;       //右后

static ArmTrajectoryContext g_ctx_lf = {0};
static ArmTrajectoryContext g_ctx_rf = {0};
static ArmTrajectoryContext g_ctx_lb = {0};
static ArmTrajectoryContext g_ctx_rb = {0};

// 每只机械臂的当前肘型配置：LF/RB 默认上肘，RF/LB 默认下肘。
static bool g_arm_elbow_up[4] = {true, false, false, true};

static  float all_offset_x[4] = { offset_X , offset_X ,-offset_X ,-offset_X};
static  float all_offset_y[4] = { offset_Y , -offset_Y ,offset_Y ,-offset_Y};

float target_x_test[4];
float target_y_test[4];

    const float TARGET_P0_X[4] = {  200.0f, 200.0f,  -200.0f,  -200.0f };
    const float TARGET_P0_Y[4] = {  300.0f, -300.0f,  300.0f, -300.0f };
    //机械臂伸直吸取物块的位置
    const float TARGET_P1_X[4] = { 425.0f,  425.0f, -425.0f, -425.0f };
    const float TARGET_P1_Y[4] = { 425.0f, -425.0f,  425.0f, -425.0f };
    // const float TARGET_P1_X[4] = { 650.0f,  650.0f, -250.0f, -250.0f };
    // const float TARGET_P1_Y[4] = { 360.0f, -360.0f,  360.0f, -360.0f };
    //机械臂携带物块的位置
    const float TARGET_P2_X[4] = { 250.0f,  250.0f, -250.0f, -250.0f };
    const float TARGET_P2_Y[4] = { 400.0f, -400.0f,  400.0f, -400.0f };
    //机械臂伸直放置物块的位置
    const float TARGET_P3_X[4] = { 60.0f,  60.0f, -60.0f, -60.0f };
    const float TARGET_P3_Y[4] = { 900.0f, -900.0f,  900.0f, -900.0f };
    //伸直放置物块中段的位置
    const float TARGET_P4_X[4] = { 280.0f,  280.0f, -280.0f, -280.0f };
    const float TARGET_P4_Y[4] = { 600.0f, -600.0f,  600.0f, -600.0f };
    

static float deg_to_rad(float deg)
{
    return deg * ((float)M_PI / 180.0f);
}

static float rad_to_deg(float rad)
{
    return rad * (180.0f / (float)M_PI);
}


// 外部实际坐标系与内部运动学坐标系的 Y 方向相反。
static float y_external_to_kinematics(float y_external)
{
    return y_external;
}

static float y_kinematics_to_external(float y_kinematics)
{
    return y_kinematics;
}


//外部运动坐标系指的是与基座相对的坐标系，原点在基座中心，X轴指向前方，Y轴指向左方；
//内部运动学坐标系指的是机械臂运动学计算使用的坐标系，存在与外部坐标系的偏移。

// 机械臂末端位置的坐标系转换：外部实际坐标系 <-> 内部运动学坐标系
static void external_to_kinematics_xy(const Planar_Robot_Arm *arm,
                                      float x_external,
                                      float y_external,
                                      float *x_kinematics,
                                      float *y_kinematics)
{
    float origin_x = 0.0f;
    float origin_y = 0.0f;

    if (arm != NULL) {
        origin_x = arm->origin_point_x_offset;
        origin_y = arm->origin_point_y_offset;
    }

    if (x_kinematics != NULL) {
        *x_kinematics = x_external - origin_x;
    }

    if (y_kinematics != NULL) {
        *y_kinematics = y_external_to_kinematics(y_external - origin_y);
    }
}

// 内部运动学坐标系与外部实际坐标系的转换。
static void kinematics_to_external_xy(const Planar_Robot_Arm *arm,
                                      float x_kinematics,
                                      float y_kinematics,
                                      float *x_external,
                                      float *y_external)
{
    float origin_x = 0.0f;
    float origin_y = 0.0f;

    if (arm != NULL) {
        origin_x = arm->origin_point_x_offset;
        origin_y = arm->origin_point_y_offset;
    }

    if (x_external != NULL) {
        *x_external = x_kinematics + origin_x;
    }

    if (y_external != NULL) {
        *y_external = y_kinematics_to_external(y_kinematics) + origin_y;
    }
}


//link1的几何主方向相对于舵机角存在固定偏角，末端位姿按真实几何计算。
static float arm_link_theta_bias_rad(const Planar_Robot_Arm *arm)
{
    if (arm == NULL) {
        return 0.0f;
    }

    // ARM_TYPE_1 第一连杆是斜边，第二关节相对斜边主方向存在固定偏角。
    if (arm->arm_type == ARM_TYPE_1) {
        return atan2f(ARM1_LINK1_SHORT_EDGE_MM, ARM1_LINK1_LONG_EDGE_MM);
    }else if (arm->arm_type == ARM_TYPE_2) {
        // 根据 ARM_TYPE_2 的参数计算偏角
        //return atan2f(ARM2_LINK1_SHORT_EDGE_MM, ARM2_LINK1_LONG_EDGE_MM);
    }

    return 0.0f;
}

// 将角度限制在指定范围内
static float clampf(float value, float min_val, float max_val)
{
    if (value < min_val) {
        return min_val;
    }
    if (value > max_val) {
        return max_val;
    }
    return value;
}

/* 将角度归一化到 (-180, +180] 度 */
static float normalize_angle_180(float deg)
{
    float a = fmodf(deg, 360.0f);
    if (a > 180.0f)  a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

/**
 * @brief 将运动学坐标系下的目标点投影到可达环形域内（含安全余量）。
 *
 * 当目标距离臂基超出物理可达范围时，沿当前方向将其拉回到边界圆上，
 * 不改变运动方向，只调整径向距离，以避免 IK 中 acos 参数越界。
 *
 * 可达环形域边界：
 *   r_max = link1 + link2 - ARM_WORKSPACE_MARGIN_MM  （防全伸奇异）
 *   r_min = |link1 - link2| + ARM_WORKSPACE_MARGIN_MM （防全折奇异）
 *
 * @param arm  机械臂实例（用于读取连杆长度）
 * @param x    [in/out] 运动学坐标系 X 分量 (mm)
 * @param y    [in/out] 运动学坐标系 Y 分量 (mm)
 */
static void clamp_to_reachable_workspace(const Planar_Robot_Arm *arm,
                                         float *x, float *y)
{
    if (arm == NULL || x == NULL || y == NULL) {
        return;
    }

    const float margin = ARM_WORKSPACE_MARGIN_MM;
    float r_max = arm->link1_length + arm->link2_length - margin;
    float r_min = fabsf(arm->link1_length - arm->link2_length) + margin;

    float r = sqrtf(*x * *x + *y * *y);

    /* 目标极近原点：无法确定方向，推到最小可达圆上（沿 +X 方向）*/
    if (r < 1e-6f) {
        *x = r_min;
        *y = 0.0f;
        return;
    }

    /* 超出环形域时沿方向等比缩放，保持指向角不变 */
    if (r < r_min) {
        float scale = r_min / r;
        *x *= scale;
        *y *= scale;
    } else if (r > r_max) {
        float scale = r_max / r;
        *x *= scale;
        *y *= scale;
    }
}

// static int16_t deg_to_servo_pos(float deg)
// {
//     // float normalized = fmodf(deg, 360.0f);
//     // if (normalized < 0.0f) 
//     // {
//     //     normalized += 360.0f;
//     // }
//     return (int16_t)((deg  + 180.0f) / 360.0f * 4095.0f);
// }

void planar_robot_arm_config_init(int arm_type , Planar_Robot_Arm *arm ,
                                  uint8_t SERVO_ID1 , uint8_t SERVO_ID2,
                                  arm_id_e ARM_ID,
                                  float origin_point_x_offset , 
                                  float origin_point_y_offset , 
                                  float servo_angle_offset1   ,
                                  float servo_angle_offset2)
{

    if (arm == NULL) {
        return;
    }

    arm->arm_type = (uint8_t)arm_type;
    arm->state = ARM_STATE_INITIALIZING;
    arm->SERVO_ID1 = SERVO_ID1;
    arm->SERVO_ID2 = SERVO_ID2;

    arm->arm_id = ARM_ID; // 设置机械臂ID

// 设置机械臂参数，根据arm_type选择不同的DH参数、关节限制等
    arm->origin_point_x_offset = origin_point_x_offset;
    arm->origin_point_y_offset = origin_point_y_offset;

    arm->servo_angle_offset[0] = servo_angle_offset1;
    arm->servo_angle_offset[1] = servo_angle_offset2;

    //高原设计的机械臂参数，单位mm
    if (arm_type == ARM_TYPE_1)
     {
    // 配置机械臂类型1的参数
        arm->link1_length = sqrtf(ARM1_LINK1_SHORT_EDGE_MM * ARM1_LINK1_SHORT_EDGE_MM +
                                  ARM1_LINK1_LONG_EDGE_MM * ARM1_LINK1_LONG_EDGE_MM); // 约为291.2mm
        arm->link2_length = ARM2_LINK2_LONG_EDGE_MM;

        arm->servo_Reverse_installation[0] = 0; // SERVO_ID1正常安装
        arm->servo_Reverse_installation[1] = 1; // SERVO_ID2反向安装

    } 
    //田飞宇设计的机械臂参数，单位mm
    // else if (arm_type == ARM_TYPE_2)
    //  {
    // // 配置机械臂类型2的参数
    //     arm->link1_length = 120.0f;
    //     arm->link2_length = 80.0f;
    // }

    arm->current_joint_angles[0] = 0.0f;
    arm->current_joint_angles[1] = 0.0f;
    arm->current_servo_positions[0] = 0;
    arm->current_servo_positions[1] = 0;

    /* 舵机指令安全初始化:
     * target_servo_positions 若保持 0, 对应舵机角度 -180° (极限).
     * 冷启动首周期若 ReadPos 失败, 轨迹上下文返回 false,
     * 但 controlA_loop 中 planar_arm_all_servo_run 不受返回值约束,
     * 会将 position=0 发送给所有舵机导致剧烈异常运动.
     * 初始化为 2048(≈0°) 作为安全默认值, 确保即使轨迹未就绪
     * 也不会向舵机发送危险指令.                                    */
    arm->target_servo_positions[0] = 2048;
    arm->target_servo_positions[1] = 2048;

    /* 默认软件限位覆盖全量程，由 planar_robot_arm_all_init 按各臂实际结构覆盖 */
    arm->servo_pos_min[0] = 0;    arm->servo_pos_max[0] = 4095;
    arm->servo_pos_min[1] = 0;    arm->servo_pos_max[1] = 4095;

    /* 默认工作空间无限制；由 planar_robot_arm_all_init 按各臂安装位置覆盖 */
    arm->workspace_x_min = -1e6f;  arm->workspace_x_max = 1e6f;
    arm->workspace_y_min = -1e6f;  arm->workspace_y_max = 1e6f;

    planar_arm_forward_kinematics(arm);
    arm->state = ARM_STATE_IDLE;

}

/* 获取机械臂末端当前位姿（正运动学结果），供外部状态查询使用 */
void planar_robot_arm_feedback(Planar_Robot_Arm *arm)
{
    // 从舵机反馈中获取当前关节位置，更新arm结构体中的关节角度和末端位姿信息
    //get_arm_servo_pos(arm);
    planar_arm_forward_kinematics(arm);

}


void get_arm_servo_pos(Planar_Robot_Arm *arm)
{
    if (arm == NULL) {
        return;
    }
    // 从舵机反馈中获取当前关节位置，更新arm结构体中的关节角度信息
    int16_t pos1 = ReadPos(arm->SERVO_ID1);
    int16_t pos2 = ReadPos(arm->SERVO_ID2);

    /* 读取失败（ReadPos 返回负值）时保留上次有效值，不中断控制循环。
     * 原实现在此处设置 ARM_STATE_ERROR 并 return，导致本周期完全跳过
     * 轨迹采样和指令发送，舵机停在原位直到下次成功——这是串口卡顿时
     * 运动指令离散化的直接原因。
     * 改为容错：最多允许 5 次连续失败后才标记 ERROR，期间沿用缓存位置。 */
     
    static uint8_t s_fail_cnt[4] = {0};
    int arm_idx = (int)arm->arm_id;
    if (arm_idx < 0 || arm_idx > 3) return;

    if (pos1 < 0 || pos2 < 0) {
        /* 读取失败：保留 current_servo_positions 上次值，计数器累加 */
        s_fail_cnt[arm_idx]++;
        if (s_fail_cnt[arm_idx] >= 5U) {
            /* 连续多次失败才真正标记为错误（防止偶发超时被误判） */
            arm->state = ARM_STATE_ERROR;
        }
        return; /* 提前返回，joint_angles 保留上次值 */
    }

    /* 读取成功，更新位置并清零失败计数 */
    s_fail_cnt[arm_idx] = 0;
    arm->current_servo_positions[0] = pos1;
    arm->current_servo_positions[1] = pos2;

    /* 四臂角度换算逻辑相同，统一计算，不再逐 case 重复 */
    arm->current_joint_angles[0] =
        (arm->current_servo_positions[0] / 4095.0f * 360.0f) - 180.0f;
    arm->current_joint_angles[1] =
        (arm->current_servo_positions[1] / 4095.0f * 360.0f) - 180.0f;

    if (arm->arm_id == ARM_ID_LF || arm->arm_id == ARM_ID_RF ||
        arm->arm_id == ARM_ID_LB || arm->arm_id == ARM_ID_RB) {
        /* 合法 ID，不做额外处理 */
    } else {
        arm->state = ARM_STATE_ERROR;
    }
}

// 机械臂正运动学计算函数，根据各关节角度计算末端执行器的位姿[good]
static void planar_arm_forward_kinematics_from_cache(Planar_Robot_Arm *arm)
{
    if (arm == NULL) {
        return;
    }

    float theta1, theta2, x_kinematics, y_kinematics, theta1_geom;
    switch (arm->arm_id) 
    {
        case ARM_ID_LF:
            // 左前机械臂的正运动学计算
             theta1 =  -deg_to_rad(arm->current_joint_angles[0] + arm->servo_angle_offset[0]);
             theta2 =   deg_to_rad(arm->current_joint_angles[1] + arm->servo_angle_offset[1]); 

             theta1_geom = theta1 + arm_link_theta_bias_rad(arm);

            // 第一连杆几何主方向相对舵机角存在固定偏角，末端位姿按真实几何计算。
            break;
        case ARM_ID_RF:
            // 右前机械臂的正运动学计算
             theta1 =  -deg_to_rad(arm->current_joint_angles[0] + arm->servo_angle_offset[0]);
             theta2 =  deg_to_rad(arm->current_joint_angles[1] + arm->servo_angle_offset[1]); 
            
             theta1_geom = theta1 - arm_link_theta_bias_rad(arm);
            
            // 第一连杆几何主方向相对舵机角存在固定偏角，末端位姿按真实几何计算。

            break;
        case ARM_ID_LB:
            // 左后机械臂的正运动学计算
             theta1 =  -deg_to_rad(arm->current_joint_angles[0] + arm->servo_angle_offset[0]);
             theta2 =  deg_to_rad(arm->current_joint_angles[1]  + arm->servo_angle_offset[1]); 
            
             theta1_geom = theta1 - arm_link_theta_bias_rad(arm);
            
            // 第一连杆几何主方向相对舵机角存在固定偏角，末端位姿按真实几何计算。
            break;
        case ARM_ID_RB:
            // 右后机械臂的正运动学计算
             theta1 =  -deg_to_rad(arm->current_joint_angles[0] + arm->servo_angle_offset[0]);
             theta2 =  deg_to_rad(arm->current_joint_angles[1] + arm->servo_angle_offset[1]); 
            
             theta1_geom = theta1 + arm_link_theta_bias_rad(arm);
            
            // 第一连杆几何主方向相对舵机角存在固定偏角，末端位姿按真实几何计算。
            break;
        default:
            // 未知机械臂ID，进入错误状态
            arm->state = ARM_STATE_ERROR;
            return;
    }
    //实际角度值映射到运动学计算的关节角度值，考虑舵机安装方向和角度偏移
                x_kinematics = arm->link1_length * cosf(theta1_geom) + arm->link2_length * cosf(theta1_geom + theta2);
                y_kinematics = arm->link1_length * sinf(theta1_geom) + arm->link2_length * sinf(theta1_geom + theta2);

            kinematics_to_external_xy(arm,
                          x_kinematics,
                          y_kinematics,
                          &arm->end_effector_x,
                          &arm->end_effector_y);
}

void planar_arm_forward_kinematics(Planar_Robot_Arm *arm)
{
    if (arm == NULL) {
        return;
    }

    /* 对外接口仍保持“先读舵机再做正解”的行为，保证兼容旧调用路径。 */
    get_arm_servo_pos(arm);
    planar_arm_forward_kinematics_from_cache(arm);
}

//需要修正偏置
bool planar_arm_inverse_kinematics(Planar_Robot_Arm *arm,
                                   bool elbow_up)
{
    if (arm == NULL) {
        return false;
    }

    /* Step 1: 将外部坐标系目标点转换到运动学坐标系（以臂基为原点） */
    float x = 0.0f;
    float y = 0.0f;
    external_to_kinematics_xy(arm, arm->end_aim_x, arm->end_aim_y, &x, &y);

    /* Step 2: 工作空间限幅——将目标点限制在可达环形域内，保护结构安全。
     * 若目标超出范围，沿径向方向拉回到边界处，方向不变。              */
    clamp_to_reachable_workspace(arm, &x, &y);

    float l1 = arm->link1_length;
    float l2 = arm->link2_length;
    float theta_bias = arm_link_theta_bias_rad(arm);

    // if (l1 <= 0.0f || l2 <= 0.0f) {
    //     return false;
    // }

    float r2 = x * x + y * y;
    // 计算关节2的角度，使用余弦定理
    float cos_theta2 = (r2 - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);

    //数值误差容忍，超出较大范围则视为不可达。
    if (cos_theta2 < -1.0001f || cos_theta2 > 1.0001f) {
//        return false;
            cos_theta2 = clampf(cos_theta2, -1.0f, 1.0f);

    }

    float sin_theta2_abs = sqrtf(fmaxf(0.0f, 1.0f - cos_theta2 * cos_theta2));
    float sin_theta2 = elbow_up ? sin_theta2_abs : -sin_theta2_abs;
    float theta2 = atan2f(sin_theta2, cos_theta2);

    float k1 = l1 + l2 * cos_theta2;
    float k2 = l2 * sin_theta2;
    float theta1_geom = atan2f(y, x) - atan2f(k2, k1);
    float theta1 = 0.0f; 


//处理舵机安装方向和角度偏移
        switch (arm->arm_id) 
    {
        case ARM_ID_LF:
            // 左前机械臂的逆运动学计算~
    theta1 = theta1_geom - theta_bias;
    arm->target_joint_angles[0] = theta1 ;
    arm->target_joint_angles[1] = theta2 ;

    arm->target_joint_angles[0] =  -rad_to_deg(arm->target_joint_angles[0])  -  arm->servo_angle_offset[0];
    arm->target_joint_angles[1] =  rad_to_deg(arm->target_joint_angles[1])   -  arm->servo_angle_offset[1];

            break;
        case ARM_ID_RF:
            // 右前机械臂的逆运动学计算
    theta1 = theta1_geom + theta_bias;
    arm->target_joint_angles[0] = theta1 ;
    arm->target_joint_angles[1] = theta2 ;
    
    arm->target_joint_angles[0] =  -rad_to_deg(arm->target_joint_angles[0])  -  arm->servo_angle_offset[0] ;
    arm->target_joint_angles[1] =  (rad_to_deg(arm->target_joint_angles[1])  -  arm->servo_angle_offset[1]);
 
            break;
        case ARM_ID_LB:
            // 左后机械臂的逆运动学计算
    theta1 = theta1_geom + theta_bias;
    arm->target_joint_angles[0] = theta1 ;
    arm->target_joint_angles[1] = theta2 ;
    
    arm->target_joint_angles[0] =   -rad_to_deg(arm->target_joint_angles[0]) -  arm->servo_angle_offset[0];
    arm->target_joint_angles[1] =  rad_to_deg(arm->target_joint_angles[1]) - arm->servo_angle_offset[1];

            break;
        case ARM_ID_RB:
            // 右后机械臂的逆运动学计算A
    theta1 = theta1_geom - theta_bias;
    arm->target_joint_angles[0] = theta1 ;
    arm->target_joint_angles[1] = theta2 ;
    
    arm->target_joint_angles[0] =  -rad_to_deg(arm->target_joint_angles[0]) -  arm->servo_angle_offset[0];
    arm->target_joint_angles[1] =  rad_to_deg(arm->target_joint_angles[1]) -  arm->servo_angle_offset[1];

            break;
        default:
            // 未知机械臂ID，进入错误状态
            arm->state = ARM_STATE_ERROR;
            return false;
    }
    /* Step 5: 归一化到 (-180, +180] 度，避免角度跳变 */
    arm->target_joint_angles[0] = normalize_angle_180(arm->target_joint_angles[0]);
    arm->target_joint_angles[1] = normalize_angle_180(arm->target_joint_angles[1]);

    return true;
}



// 四臂同时控制函数，批量发送位置指令,各个臂测试完后使用。
bool planar_arm_all_servo_run(Planar_Robot_Arm *arm_LF, Planar_Robot_Arm *arm_RF, Planar_Robot_Arm *arm_LB, Planar_Robot_Arm *arm_RB)
{
    if (arm_LF == NULL || arm_RF == NULL || arm_LB == NULL || arm_RB == NULL) {
        return false;
    }

    uint8_t ID[8] = {
        arm_LF->SERVO_ID1, arm_LF->SERVO_ID2,
        arm_RF->SERVO_ID1, arm_RF->SERVO_ID2,
        arm_LB->SERVO_ID1, arm_LB->SERVO_ID2,
        arm_RB->SERVO_ID1, arm_RB->SERVO_ID2
    };
    int16_t Position[8] = {
        arm_LF->target_servo_positions[0], arm_LF->target_servo_positions[1],
        arm_RF->target_servo_positions[0], arm_RF->target_servo_positions[1],
        arm_LB->target_servo_positions[0], arm_LB->target_servo_positions[1],
        arm_RB->target_servo_positions[0], arm_RB->target_servo_positions[1]
    };
    uint16_t Speed[8] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000}; // 速度数组
    uint8_t ACC[8]    = {0, 0, 0, 0, 0, 0, 0, 0}; // 加速度数组

    SyncWritePosEx(ID, 8, Position, Speed, ACC);
    return true;
}

// 将舵机位置转换为关节角度（纯计算，不做 I/O）。
static void update_joint_angles_from_servo_pos(Planar_Robot_Arm *arm)
{
    if (arm == NULL) return;
    arm->current_joint_angles[0] =
        (arm->current_servo_positions[0] / 4095.0f * 360.0f) - 180.0f;
    arm->current_joint_angles[1] =
        (arm->current_servo_positions[1] / 4095.0f * 360.0f) - 180.0f;
}

// 单臂读取舵机位置（2 次 ReadPos），成功后更新 joint_angles。
// 读取失败时保留上次缓存值，连续失败 5 次才标记 ERROR，
// 防止通信偶发超时导致本周期完全跳过轨迹输出。
static bool read_one_arm_servo_pos(Planar_Robot_Arm *arm)
{
    if (arm == NULL) return false;
     int16_t pos1 = ReadPos(arm->SERVO_ID1);
    int16_t pos2 = ReadPos(arm->SERVO_ID2);
    if (pos1 < 0 || pos2 < 0) {
        /* 失败：不立即设 ERROR，保留旧值，由 get_arm_servo_pos 的
         * 连续失败计数器统一管理状态，这里直接返回 false 即可。 */
        return false;
    }
    arm->current_servo_positions[0] = pos1;
    arm->current_servo_positions[1] = pos2;
    update_joint_angles_from_servo_pos(arm);
    return true;
}

// 批量读取全部 4 臂 8 舵机位置，一次完成所有 I/O。
// 返回值仅供上层统计，不影响控制循环的继续执行（读取失败时各臂保留缓存值）。
bool batch_read_all_servo_pos(void)
{
    bool ok = true;
    ok &= read_one_arm_servo_pos(&Arm_LF);
    ok &= read_one_arm_servo_pos(&Arm_RF);
    ok &= read_one_arm_servo_pos(&Arm_LB);
    ok &= read_one_arm_servo_pos(&Arm_RB);
    return ok; /* false 表示本周期有读取失败，但控制循环不中断 */
}

bool planar_robot_arm_all_init(void)
{
    SCS_SetUART(&huart1);
    /* 根据硬件接线配置总线模式：
     *   SCS_SetHalfDuplex(1) — PA9(TX) 与 PB7(RX) 在 PCB 上物理连接到同一总线导线（有回声）
     *   SCS_SetHalfDuplex(0) — TX/RX 走独立线路经收发器连接（无回声，默认）
     * 若发现 ReadPos 始终返回 -1，且硬件为单线总线，请改为 SCS_SetHalfDuplex(1)。 */
    SCS_SetHalfDuplex(0);
    setEnd(0);
    /* 等待舵机总线上电稳定：USART 外设初始化完成后，舵机侧需要一段时间完成
     * 自检并准备好响应读取命令，过早发送 ReadPos 会导致失败，使
     * current_servo_positions 保留初始值 0，进而引发冷启动异常运动。 */
    
    osDelay(300);
    // 配置四个机械臂参数，分别为左前、右前、左后、右后
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_LF, 1, 2, 
                                 ARM_ID_LF, all_offset_x[0], all_offset_y[0],  -90.0f, 90.0f);
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_RF, 3, 4,
                                 ARM_ID_RF, all_offset_x[1], all_offset_y[1], 90.0f,  -90.0f);
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_LB, 5, 6,
                                 ARM_ID_LB, all_offset_x[2], all_offset_y[2], -90.0f, -90.0f);
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_RB, 7, 8,
                                 ARM_ID_RB, all_offset_x[3], all_offset_y[3], 90.0f, 90.0f);

    /* ---- 各臂各关节软件限位（步进值，0~4095 对应 -180°~+180°）----
     * 根据实际机械结构和装配方向配置，防止连杆碰撞或舵机过载。
     * 关节1 = 大臂舵机，关节2 = 小臂舵机。                        */
    
    //230添加是为了拓展角度//
    Arm_LF.servo_pos_min[0] = 1024 - 230; Arm_LF.servo_pos_max[0] = 3072 + 230; /* LF J1: -90°~+90° */
    Arm_LF.servo_pos_min[1] = 1024;       Arm_LF.servo_pos_max[1] = 3072; /* LF J2: -90°~+90° */

    Arm_RF.servo_pos_min[0] = 1024 - 230; Arm_RF.servo_pos_max[0] = 3072 + 230; /* RF J1 */
    Arm_RF.servo_pos_min[1] = 1024;       Arm_RF.servo_pos_max[1] = 3072; /* RF J2 */

    Arm_LB.servo_pos_min[0] = 1024 - 230; Arm_LB.servo_pos_max[0] = 3072 + 230; /* LB J1 */
    Arm_LB.servo_pos_min[1] = 1024;       Arm_LB.servo_pos_max[1] = 3072; /* LB J2 */

    Arm_RB.servo_pos_min[0] = 1024 - 230; Arm_RB.servo_pos_max[0] = 3072 + 230; /* RB J1 */
    Arm_RB.servo_pos_min[1] = 1024;       Arm_RB.servo_pos_max[1] = 3072; /* RB J2 */

    /* ---- 各臂末端工作空间矩形限位（外部坐标系，单位 mm）----
     * 超出范围的目标将被自动钳位到矩形内最近的边界点。
     * 坐标系：X 轴指向前方，Y 轴指向左方，原点在机体中心。
     *
     * 【注意】这些只是软件层面的粗略钳位边界，用于防止极端误指令。
     * 真正的物理可达性由 IK 解算 + 舵机限位 (servo_pos_min/max) 决定，
     * 因此各臂统一使用较大的对称边界，不在此处做象限硬分割。
     * 原版本将各臂限制在各自象限内（如 LF y_min=0），导致 Y=0 成为
     * 不可跨越的禁线，是 |Y|<100 无法达到的原因之一。              */
    Arm_LF.workspace_x_min = 0.0f; Arm_LF.workspace_x_max = 800.0f;
    Arm_LF.workspace_y_min = 0.0f; Arm_LF.workspace_y_max = 900.0f;

    Arm_RF.workspace_x_min = 0.0f; Arm_RF.workspace_x_max = 800.0f;
    Arm_RF.workspace_y_min = -900.0f; Arm_RF.workspace_y_max = 0.0f;

    Arm_LB.workspace_x_min = -800.0f; Arm_LB.workspace_x_max = 0.0f;
    Arm_LB.workspace_y_min = 0.0f;  Arm_LB.workspace_y_max = 900.0f;

    Arm_RB.workspace_x_min = -800.0f; Arm_RB.workspace_x_max = 0.0f;
    Arm_RB.workspace_y_min = -900.0f; Arm_RB.workspace_y_max = 0.0f;

    /* 所有臂初始化完成，将末端目标设置为归位点。
     * 控制任务启动后第一个轨迹段将自动规划并执行归位运动。*/

        for (int i = 0; i < 4; i++) {
            target_x_test[i] = TARGET_P0_X[i];
            target_y_test[i] = TARGET_P0_Y[i];
        }

    return true;
}

/**
 * @brief 将四臂末端目标设为预定义归位坐标（外部坐标系）。
 *
 * 归位位置已通过逆解计算验证可达，且各臂处于各自象限内，
 * 互不干涉，也不穿过机体区域。可在任意时刻调用以触发归位动作。
 *
 * 归位点（外部坐标系，单位 mm）：
 *   LF (+250, +440)  RF (+250, -440)
 *   LB (-250, +440)  RB (-250, -440)
 */



// 将 arm_id 转换为数组索引；非法ID返回 -1。
static int arm_id_to_index(arm_id_e arm_id)
{
    switch (arm_id) {
        case ARM_ID_LF:
            return 0;
        case ARM_ID_RF:
            return 1;
        case ARM_ID_LB:
            return 2;
        case ARM_ID_RB:
            return 3;
        default:
            return -1;
    }
}

// 根据机械臂ID获取结构体指针，供外部接口和内部控制复用。
Planar_Robot_Arm *planar_robot_arm_get_by_id(arm_id_e arm_id)
{
    switch (arm_id) {
        case ARM_ID_LF:
            return &Arm_LF;
        case ARM_ID_RF:
            return &Arm_RF;
        case ARM_ID_LB:
            return &Arm_LB;
        case ARM_ID_RB:
            return &Arm_RB;
        default:
            return NULL;
    }
}

// 外部控制入口：仅更新目标位置，保留该机械臂当前肘型配置。
// 目标超出该臂 workspace 矩形时，自动钳位到矩形内最近边界点（各轴独立钳位）。
bool planar_robot_arm_set_target(arm_id_e arm_id, float target_x, float target_y)
{
    Planar_Robot_Arm *arm = planar_robot_arm_get_by_id(arm_id);
    if (arm == NULL) {
        return false;
    }

    arm->end_aim_x = clampf(target_x, arm->workspace_x_min, arm->workspace_x_max);
    arm->end_aim_y = clampf(target_y, arm->workspace_y_min, arm->workspace_y_max);
    return true;
}

// 外部控制入口：同时更新目标位置和肘型配置。
bool planar_robot_arm_set_target_with_elbow(arm_id_e arm_id,
                                            float target_x,
                                            float target_y,
                                            bool elbow_up)
{
    int index = arm_id_to_index(arm_id);
    if (index < 0) {
        return false;
    }

    if (!planar_robot_arm_set_target(arm_id, target_x, target_y)) {
        return false;
    }

    g_arm_elbow_up[index] = elbow_up;
    return true;
}

/**
 * @brief 启动归位阶段 — 阻塞式驱动四臂到达 target_x_test 目标
 *
 * 内部循环: 读舵机 → 正运动学 → IK+轨迹+舵机输出,
 * 直到四臂末端均在 tolerance_mm 内到达目标, 或超时.
 *
 * 调用前必须已通过 planar_robot_arm_all_init() 设置目标位置
 * 并完成舵机总线初始化.
 *
 * @param timeout_ms   归位超时 (ms), 通常 3000~5000
 * @param tolerance_mm 到位判定容差 (mm), 通常 15~30
 * @retval true   四臂均到位
 * @retval false  超时 (舵机可能卡死或目标不可达)
 */
bool planar_robot_arm_startup_home(uint32_t timeout_ms, float tolerance_mm)
{
    uint32_t start = HAL_GetTick();

    while (1) {
        /* 读取全部 8 舵机反馈 + 正运动学四臂 */
        // (void)batch_read_all_servo_pos();
        // planar_arm_forward_kinematics_from_cache(&Arm_LF);
        // planar_arm_forward_kinematics_from_cache(&Arm_RF);
        // planar_arm_forward_kinematics_from_cache(&Arm_LB);
        // planar_arm_forward_kinematics_from_cache(&Arm_RB);

        /* IK + 轨迹规划 + 舵机输出 */
        planar_arm_control_loop();

        /* 检查四臂是否均已到达 target_x_test/y_test */
        bool all_homed = true;
        const Planar_Robot_Arm *arms[4] = {
            &Arm_LF, &Arm_RF, &Arm_LB, &Arm_RB
        };
        for (int i = 0; i < 4; i++) {
            float dx = fabsf(arms[i]->end_effector_x - target_x_test[i]);
            float dy = fabsf(arms[i]->end_effector_y - target_y_test[i]);
            if (dx > tolerance_mm || dy > tolerance_mm) {
                all_homed = false;
                break;
            }
        }

        if (all_homed) {
            return true;
        }

        /* 超时保护: 避免舵机卡死或目标不可达导致死循环 */
        if ((uint32_t)(HAL_GetTick() - start) > timeout_ms) {
            return false;
        }

        osDelay(4);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// 计算五次多项式轨迹系数，使轨迹满足起终点的位置/速度/加速度边界条件。
static void solve_quintic_coefficients(float p0,
                                       float v0,
                                       float a0,
                                       float pf,
                                       float vf,
                                       float af,
                                       float t,
                                       float coeffs[6])
{
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;

    coeffs[0] = p0;
    coeffs[1] = v0;
    coeffs[2] = 0.5f * a0;

    coeffs[3] = (20.0f * (pf - p0) - (8.0f * vf + 12.0f * v0) * t - (3.0f * a0 - af) * t2) / (2.0f * t3);
    coeffs[4] = (30.0f * (p0 - pf) + (14.0f * vf + 16.0f * v0) * t + (3.0f * a0 - 2.0f * af) * t2) / (2.0f * t4);
    coeffs[5] = (12.0f * (pf - p0) - (6.0f * vf + 6.0f * v0) * t - (a0 - af) * t2) / (2.0f * t5);
}

// 规划新的轨迹段：在给定时长内，从当前状态平滑过渡到目标状态。
static void plan_new_move(TrajectoryPlanner *planner,
                          uint32_t current_time_ms,
                          float start_pos,
                          float start_vel,
                          float start_acc,
                          float end_pos,
                          float end_vel,
                          float end_acc,
                          float duration_sec)
{
    if (planner == NULL) {
        return;
    }

    planner->start_time = current_time_ms;
    planner->is_running = (duration_sec > 0.0001f);

    planner->current_segment.p0 = start_pos;
    planner->current_segment.v0 = start_vel;
    planner->current_segment.a0 = start_acc;
    planner->current_segment.pf = end_pos;
    planner->current_segment.vf = end_vel;
    planner->current_segment.af = end_acc;
    planner->current_segment.duration = duration_sec;
    planner->current_segment.is_valid = true;

    if (!planner->is_running) {
        for (int i = 0; i < 6; i++) {
            planner->current_segment.coeffs[i] = 0.0f;
        }
        planner->current_segment.coeffs[0] = end_pos;
        return;
    }

    /* 五次多项式插补：满足起终点位置/速度/加速度边界条件 */
    solve_quintic_coefficients(start_pos,
                               start_vel,
                               start_acc,
                               end_pos,
                               end_vel,
                               end_acc,
                               duration_sec,
                               planner->current_segment.coeffs);
}

// 按当前时间采样轨迹，返回当前位置/速度/加速度；轨迹结束时自动钳位到终点。
static bool update_trajectory(TrajectoryPlanner *planner,
                              uint32_t current_time_ms,
                              float *out_pos,
                              float *out_vel,
                              float *out_acc)
{
    if (planner == NULL || !planner->current_segment.is_valid) {
        return false;
    }

    if (!planner->is_running) {
        if (out_pos != NULL) {
            *out_pos = planner->current_segment.pf;
        }
        if (out_vel != NULL) {
            *out_vel = 0.0f;
        }
        if (out_acc != NULL) {
            *out_acc = 0.0f;
        }
        return true;
    }

    /* uint32 subtraction handles HAL_GetTick() wrap-around correctly */
    /* HAL_GetTick() 单位 ms，除以 1000 转换为秒；曾误写 /4000 导致速度 4x 偏慢 */
    float elapsed_time = (float)((uint32_t)(current_time_ms - planner->start_time)) / 1000.0f;
    float t = elapsed_time;
    float T = planner->current_segment.duration;
    float *a = planner->current_segment.coeffs;

    if (t >= T) {
        t = T;
        planner->is_running = false;
    }

    if (out_pos != NULL) {
        *out_pos = a[0] + t * (a[1] + t * (a[2] + t * (a[3] + t * (a[4] + t * a[5]))));
    }
    if (out_vel != NULL) {
        *out_vel = a[1] + t * (2.0f * a[2] + t * (3.0f * a[3] + t * (4.0f * a[4] + t * 5.0f * a[5])));
    }
    if (out_acc != NULL) {
        *out_acc = 2.0f * a[2] + t * (6.0f * a[3] + t * (12.0f * a[4] + t * 20.0f * a[5]));
    }

    return true;
}

// 根据 arm_id 获取对应的轨迹上下文，确保每只机械臂独立规划。
static ArmTrajectoryContext *get_arm_traj_ctx(const Planar_Robot_Arm *arm)
{
    if (arm == NULL) {
        return NULL;
    }

    switch (arm->arm_id) {
        case ARM_ID_LF:
            return &g_ctx_lf;
        case ARM_ID_RF:
            return &g_ctx_rf;
        case ARM_ID_LB:
            return &g_ctx_lb;
        case ARM_ID_RB:
            return &g_ctx_rb;
        default:
            return NULL;
    }
}

// 检查目标是否发生显著变化；若变化超过阈值，触发轨迹重规划。
static bool target_changed(const ArmTrajectoryContext *ctx, float x, float y)
{
    if (ctx == NULL) {
        return false;
    }

    return (fabsf(x - ctx->last_cmd_x) > CONTROLA_REPLAN_EPS_MM) ||
           (fabsf(y - ctx->last_cmd_y) > CONTROLA_REPLAN_EPS_MM);
}

// 对单臂执行“反馈 -> 必要时重规划 -> 轨迹采样 -> 控制输出”。
static bool arm_control_step_with_trajectory(Planar_Robot_Arm *arm, bool elbow_up, uint32_t now_ms)
{
    if (arm == NULL) {
        return false;
    }

    // 获取对应机械臂的轨迹上下文，确保每只臂独立管理轨迹状态。
    ArmTrajectoryContext *ctx = get_arm_traj_ctx(arm);
    if (ctx == NULL) {
        return false;
    }

    /* 原子快照目标位置对：end_aim_x/y 可由 USB 中断（cmd_rx_feed）随时写入，
     * 控制任务须在临界区内一次性读取，防止读到 x 新值 + y 旧值的中间态，
     * 避免 IK 以非法坐标求解触发错误重规划和舵机指令跳变。              */
    taskENTER_CRITICAL();
    float snap_aim_x = arm->end_aim_x;
    float snap_aim_y = arm->end_aim_y;
    taskEXIT_CRITICAL();

    /* 首次进入或目标变化时重规划；其余周期只做轨迹采样。 */
    bool need_replan = (!ctx->initialized) || target_changed(ctx, snap_aim_x, snap_aim_y);

    if (need_replan) {
        /*在启动逆运动学（IK）计算之前，先“冻结”并保存一份机器人的当前状态，以防止在计算过程中因为数据被更新而导致逻辑判断出错 */
        arm->planned_target_x = snap_aim_x;
        arm->planned_target_y = snap_aim_y;
        arm->end_aim_x        = snap_aim_x;
        arm->end_aim_y        = snap_aim_y;

        if (arm->state == ARM_STATE_ERROR) {
            return false;
        }

        /* 逆解 */
        bool solved = planar_arm_inverse_kinematics(arm, elbow_up);
        if (!solved) {
            arm->state = ARM_STATE_ERROR;
            EnableTorque(arm->SERVO_ID1, 0);
            EnableTorque(arm->SERVO_ID2, 0);
            return false;
        }

        float raw_j1 = (arm->target_joint_angles[0] + 180.0f) / 360.0f * 4095.0f;
        float raw_j2 = (arm->target_joint_angles[1] + 180.0f) / 360.0f * 4095.0f;
        float tgt_j1 = clampf(raw_j1,
                              (float)arm->servo_pos_min[0], (float)arm->servo_pos_max[0]);
        float tgt_j2 = clampf(raw_j2,
                              (float)arm->servo_pos_min[1], (float)arm->servo_pos_max[1]);

        if (!isfinite(tgt_j1) || !isfinite(tgt_j2)) {
            arm->state = ARM_STATE_ERROR;
            return false;
        }

        /* ---- IK 可达性检测 (软警告, 不阻断) ----
         *
         * 对比 IK 解算出的原始舵机步进值 (raw_j1/2) 与被 servo_pos_min/max
         * 钳位后的值 (tgt_j1/2)。始终使用钳位后的安全值继续轨迹规划,
         * 机械臂尽可能逼近目标 —— 舵机限位本身就是硬件最后一道防线.
         *
         * 【修复原因】原实现在此处用 goto skip_replan 阻断轨迹生成,
         * 导致两个严重问题:
         *   1. 冷启动 P0 被拒 + 上下文永不初始化 → 上电后不运动
         *   2. 交接 MID 被拒 + last_cmd 锁定 → 永久卡死在原位
         * 修改为: 仅做软警告, 不阻断, 不跳转.                   */
        {
            float clamp1 = fabsf(raw_j1 - tgt_j1);
            float clamp2 = fabsf(raw_j2 - tgt_j2);

            if (clamp1 > (float)IK_CLAMP_WARN_STEP || clamp2 > (float)IK_CLAMP_WARN_STEP) {
                /* 截断量显著: 目标在物理上不完全可达,
                 * 但机械臂仍应以钳位后安全值运动以尽量逼近. */
            }
            /* 始终使用钳位值 tgt_j1/j2, 不阻断, 不跳转 */
        }

        if (!ctx->initialized) {
            /* 冷启动保护：current_servo_positions 初始化为 0，若通信失败仍为 0，
             * 以步进值 0 作为起点规划轨迹会让舵机从 -180° 极端位置高速冲向目标。
             * 当两个位置均为 0（上电默认值）时推迟初始化，等待至少一帧有效反馈。 */
            if (arm->current_servo_positions[0] == 0 && arm->current_servo_positions[1] == 0) {
                return false;
            }
            ctx->cmd_j1 = (float)arm->current_servo_positions[0];
            ctx->cmd_j2 = (float)arm->current_servo_positions[1];            ctx->vel_j1 = 0.0f; ctx->vel_j2 = 0.0f;
            ctx->acc_j1 = 0.0f; ctx->acc_j2 = 0.0f;
        } else {
            update_trajectory(&ctx->planner_j1, now_ms,
                              &ctx->cmd_j1, &ctx->vel_j1, &ctx->acc_j1);
            update_trajectory(&ctx->planner_j2, now_ms,
                              &ctx->cmd_j2, &ctx->vel_j2, &ctx->acc_j2);
        }

        /* Adaptive duration: scale with Euclidean distance in servo-step space */
        float d1 = tgt_j1 - ctx->cmd_j1;
        float d2 = tgt_j2 - ctx->cmd_j2;
        float dist = sqrtf(d1 * d1 + d2 * d2);
        float duration = dist / CONTROLA_MAX_SERVO_SPEED_STEP_PER_S;
        if (duration < CONTROLA_TRAJ_MIN_S) duration = CONTROLA_TRAJ_MIN_S;
        if (duration > CONTROLA_TRAJ_MAX_S) duration = CONTROLA_TRAJ_MAX_S;

        /* Plan quintic polynomial from current traj state to new target */
        plan_new_move(&ctx->planner_j1, now_ms,
                      ctx->cmd_j1, ctx->vel_j1, ctx->acc_j1,
                      tgt_j1, 0.0f, 0.0f,
                      duration);
        plan_new_move(&ctx->planner_j2, now_ms,
                      ctx->cmd_j2, ctx->vel_j2, ctx->acc_j2,
                      tgt_j2, 0.0f, 0.0f,
                      duration);


        ctx->last_cmd_x  = arm->planned_target_x;
        ctx->last_cmd_y  = arm->planned_target_y;
        ctx->initialized = true;
    }

    // 采样轨迹得到本控制周期目标点。若轨迹结束，输出会自动钳位到终点。
    float j1_cmd = ctx->planner_j1.current_segment.pf;
    float j2_cmd = ctx->planner_j2.current_segment.pf;
    update_trajectory(&ctx->planner_j1, now_ms, &j1_cmd, &ctx->vel_j1, &ctx->acc_j1);
    update_trajectory(&ctx->planner_j2, now_ms, &j2_cmd, &ctx->vel_j2, &ctx->acc_j2);
    ctx->cmd_j1 = j1_cmd;
    ctx->cmd_j2 = j2_cmd;


    arm->target_servo_positions[0] = (int16_t)clampf(j1_cmd, (float)arm->servo_pos_min[0], (float)arm->servo_pos_max[0]);
    arm->target_servo_positions[1] = (int16_t)clampf(j2_cmd, (float)arm->servo_pos_min[1], (float)arm->servo_pos_max[1]);

    arm->state = ARM_STATE_MOVING;
    return true;
}



bool controlA_loop(void)
{
    bool ok = true;
    uint32_t now_ms = HAL_GetTick();
    /* ════════════════════════════════════════════════════════════════
     * 注意: DT7 遥控器处理逻辑和 P2→P3 路径点状态机已迁移至:
     *   - input_arbiter.c   (RC 数据处理 + waypoint_smooth_filter)
     *   - action_scheduler.c (交接动作触发)
     *
     * controlA_loop 现仅负责:
     *   1. 将 target_x_test/y_test 写入各臂 end_aim_x/y
     *   2. 读取舵机反馈 → 正运动学 → IK + 轨迹规划 → 舵机输出
     *
     * 输入仲裁由 arm_control_task 中的 input_arbiter_resolve() 完成.
     * ════════════════════════════════════════════════════════════════ */

    /* 将 target_x_test / target_y_test 数组统一应用到各机械臂。
     * 这些数组可由遥控器（USE_DT7_DEBUG 内）或物块交接状态机
     * （associate_run_one_pair）写入，此处统一执行，确保无论
     * 哪种来源都能生效。                                          */
    planar_robot_arm_set_target(ARM_ID_LF, target_x_test[0], target_y_test[0]);
    planar_robot_arm_set_target(ARM_ID_RF, target_x_test[1], target_y_test[1]);
    planar_robot_arm_set_target(ARM_ID_LB, target_x_test[2], target_y_test[2]);
    planar_robot_arm_set_target(ARM_ID_RB, target_x_test[3], target_y_test[3]);


    // 批量读取全部舵机位置（8 次 ReadPos，集中完成）。
    // 读取失败时保留缓存值，不中断后续轨迹计算和指令发送。
    (void)batch_read_all_servo_pos();

    // 每周期基于最新反馈做正运动学，更新末端位置数据。
    planar_arm_forward_kinematics_from_cache(&Arm_LF);
    planar_arm_forward_kinematics_from_cache(&Arm_RF);
    planar_arm_forward_kinematics_from_cache(&Arm_LB);
    planar_arm_forward_kinematics_from_cache(&Arm_RB);

    //四臂轨迹计算（纯计算，无 I/O）
    bool traj_ok = true;
    traj_ok &= arm_control_step_with_trajectory(&Arm_LF, g_arm_elbow_up[0], now_ms);
    traj_ok &= arm_control_step_with_trajectory(&Arm_RF, g_arm_elbow_up[1], now_ms);
    traj_ok &= arm_control_step_with_trajectory(&Arm_LB, g_arm_elbow_up[2], now_ms);
    traj_ok &= arm_control_step_with_trajectory(&Arm_RB, g_arm_elbow_up[3], now_ms);

    /* 仅当四臂轨迹均成功时才发送舵机同步指令.
     * 冷启动首周期若 ReadPos 失败, trajectory 返回 false,
     * target_servo_positions 尚未更新, 跳过写入避免发送危险指令.  */
    if (traj_ok) {
        ok &= planar_arm_all_servo_run(&Arm_LF, &Arm_RF, &Arm_LB, &Arm_RB);
    } else {
        ok = false;
    }
    
    return ok;

}

// 保留原接口，避免现有任务代码改动；内部统一走 controlA_loop。
bool planar_arm_control_loop(void)
{
    return controlA_loop();

}


#ifdef USE_FISH
//此处是使用绞盘的控制逻辑
//此处使用舵机的速度控制逻辑，

const uint8_t FISH_SERVO_ID[4] = {0x15, 0x16, 0x17, 0x18}; //假设使用4个舵机控制绞盘

void fish_servo_init(void)
{
  
    for (int i = 0; i < 4; i++) 
    {
        EnableTorque(FISH_SERVO_ID[i], 1); //使能舵机
    }  
  
}

void fish_servo_set_speed(uint8_t servo_id, int16_t speed)
{
    


}


#endif /* USE_FISH */


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/**
 * @brief  获取前侧交接对状态 (兼容旧 LEFT 全局变量)
 * @note   委托给 action_scheduler 模块
 * @return block_associate_state_e 枚举值
 */
block_associate_state_e action_get_left_state(void);

/**
 * @brief  获取后侧交接对状态 (兼容旧 RIGHT 全局变量)
 * @note   委托给 action_scheduler 模块
 * @return block_associate_state_e 枚举值
 */
block_associate_state_e action_get_right_state(void);

/* ════════════════════════════════════════════════════════════════
 * 交接动作 API 实现说明
 *
 * associate_trigger / associate_abort / associate_get_state /
 * ACTION_loop / ACTION_recvie 等函数的实现已迁移至:
 *   application/Gripper/action_scheduler.c
 *
 * Planar_Robot_Arm.h 仍声明这些接口, 由链接器自动解析到
 * action_scheduler.o 中的实现. 外部模块无需修改调用代码.
 *
 * Planar_Robot_Arm.c 通过 #include "action_scheduler.h" 引入
 * 类型定义 (action_state_e, block_associate_state_e 等),
 * 但不再包含这些函数的重复实现.
 *
 * LEFT / RIGHT 全局变量兼容性:
 *   原 LEFT / RIGHT 变量在 variables.h 中作为 extern 声明.
 *   现在通过 action_get_left_state() / action_get_right_state()
 *   查询, 在需要旧式访问的代码中通过宏或 wrapper 保持兼容.
 * ════════════════════════════════════════════════════════════════ */



