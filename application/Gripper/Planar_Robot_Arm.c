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

    const float TARGET_P0_X[4] = {  180.0f, 600.0f,  -180.0f,  -180.0f };
    const float TARGET_P0_Y[4] = {  240.0f, -330.0f,  240.0f, -240.0f };
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
    else if (arm_type == ARM_TYPE_2)
     {
    // 配置机械臂类型2的参数
        arm->link1_length = 120.0f;
        arm->link2_length = 80.0f;
    }

    arm->current_joint_angles[0] = 0.0f;
    arm->current_joint_angles[1] = 0.0f;
    arm->current_servo_positions[0] = 0;
    arm->current_servo_positions[1] = 0;

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

// // 单臂控制步骤：逆运动学 -> 发送位置指令（反馈由调用方在外部统一完成，避免重复读取）
// static __attribute__((unused)) bool arm_control_step(Planar_Robot_Arm *arm, bool elbow_up ,float x, float y)
// {
//     if (arm == NULL) {
//         return false;
//     }
//     //planar_arm_forward_kinematics(arm); // 获取当前末端位姿，更新arm结构体中的位置信息
//     // 逆运动学求解目标关节角
//     arm->end_aim_x = x;
//     arm->end_aim_y = y;
//     bool solved = planar_arm_inverse_kinematics(arm, elbow_up);
//     if (!solved) 
//     {
//         arm->state = ARM_STATE_ERROR;
//         EnableTorque(arm->SERVO_ID1, 0);
//         EnableTorque(arm->SERVO_ID2, 0);
//         return false;
//     }

//     arm->state = ARM_STATE_MOVING;

//     // 3. 将目标关节角转换为舵机位置并同步写入
//     arm->target_servo_positions[0] =  ((arm->target_joint_angles[0]  + 180.0f) / 360.0f * 4095.0f);
//     arm->target_servo_positions[1] =  ((arm->target_joint_angles[1]  + 180.0f) / 360.0f * 4095.0f);

//     arm->target_servo_positions[0] = (int16_t)clampf(arm->target_servo_positions[0], 1024.0f, 3072.0f);
//     arm->target_servo_positions[1] = (int16_t)clampf(arm->target_servo_positions[1], 1024.0f, 3072.0f);
//     // 保留转换逻辑，发送接口在此路径暂未启用。
//     // (void)arm->target_servo_positions[0];
//     // (void)arm->target_servo_positions[1];
//     uint8_t  ID[2]       = {arm->SERVO_ID1, arm->SERVO_ID2}; // 舵机ID数组
//     int16_t  Position[2] = {arm->target_servo_positions[0], arm->target_servo_positions[1]}; // 目标位置数组
//     uint16_t Speed[2]   = {0, 0}; // 速度数组
//     uint8_t  ACC[2]      = {0, 0}; // 加速度数组

//     // WritePosEx(arm->SERVO_ID1, arm->target_servo_positions[0], Speed[0], ACC[0]);
//     // osDelay(1);
//     // WritePosEx(arm->SERVO_ID2, arm->target_servo_positions[1], Speed[1], ACC[1]);

//     /* 同步写入两个舵机目标位置：SCSerail（USART1 RXNE 中断）统一管理总线，
//      * 此处直接发送，无需额外 osDelay。                                     */
//     //SyncWritePosEx(ID, 2, Position, Speed, ACC);

//     return true;
// }


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
     * 各臂安装于各自象限，限位矩形应包含各臂所有有效目标点。 */
    Arm_LF.workspace_x_min =   60.0f; Arm_LF.workspace_x_max = 800.0f;   /* LF: 前半空间 */
    Arm_LF.workspace_y_min =   180.0f; Arm_LF.workspace_y_max = 900.0f;   /* LF: 左半空间 */

    Arm_RF.workspace_x_min =   60.0f;  Arm_RF.workspace_x_max = 800.0f;   /* RF: 前半空间 */
    Arm_RF.workspace_y_min = -900.0f; Arm_RF.workspace_y_max =  -180.0f;  /* RF: 右半空间 */

    Arm_LB.workspace_x_min = -800.0f; Arm_LB.workspace_x_max =   -60.0f; /* LB: 后半空间 */
    Arm_LB.workspace_y_min =   180.0f;  Arm_LB.workspace_y_max = 900.0f; /* LB: 左半空间 */

    Arm_RB.workspace_x_min = -800.0f; Arm_RB.workspace_x_max =   -60.0f; /* RB: 后半空间 */
    Arm_RB.workspace_y_min = -900.0f; Arm_RB.workspace_y_max =  -180.0f; /* RB: 右半空间 */

    /* 所有臂初始化完成，将末端目标设置为归位点。
     * 控制任务启动后第一个轨迹段将自动规划并执行归位运动。  */
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

        /* Convert target joint angles to servo step counts, clamped by per-joint software limits */
        float tgt_j1 = clampf(
            (arm->target_joint_angles[0] + 180.0f) / 360.0f * 4095.0f,
            (float)arm->servo_pos_min[0], (float)arm->servo_pos_max[0]);
        float tgt_j2 = clampf(
            (arm->target_joint_angles[1] + 180.0f) / 360.0f * 4095.0f,
            (float)arm->servo_pos_min[1], (float)arm->servo_pos_max[1]);

        /* Guard against NaN/Inf from IK numerical failure (e.g. atan2(0,0),
           sqrtf of negative, or out-of-workspace target) */
        if (!isfinite(tgt_j1) || !isfinite(tgt_j2)) {
            arm->state = ARM_STATE_ERROR;
            return false;
        }

        /* Bootstrap on first entry from actual servo feedback (cold start).
           On subsequent replans: sample current trajectory pos/vel/acc as start
           to guarantee command continuity even if previous segment is mid-flight. */
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

        /* Use the snapshot value so last_cmd always reflects the true user
           intent, never a value potentially modified by IK internals. */
        ctx->last_cmd_x  = arm->planned_target_x;
        ctx->last_cmd_y  = arm->planned_target_y;
        ctx->initialized = true;
    }

    // 采样轨迹得到本控制周期目标点。若轨迹结束，输出会自动钳位到终点。
    /* Sample this control cycle; update ctx vel/acc for next replan.
       After trajectory ends the output clamps to the final setpoint. */
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
#ifdef movedebug
    /* ---- 运动调试模式 (movedebug) ----
     * 每 3 秒在两组目标点之间切换，用于验证运动学和轨迹规划的正确性。
     * P0：收拢位（各臂靠近机体）  P1：展开位（各臂向外伸展，与归位点相同）
     * 外部坐标系，单位 mm。若需修改测试点，直接修改下面的坐标数组即可。 */
    static bool     s_dbg_init      = false;
    static uint8_t  s_dbg_idx       = 0U;
    static uint32_t s_dbg_switch_ms = 0U;
    const  uint32_t DBG_INTERVAL_MS = 3000U;

    /* P0：收拢位（各臂向内伸展，与上电归位点一致）*/
    const float TARGET_P0_X[4] = {  160.0f, 160.0f,  -160.0f,  -160.0f };
    const float TARGET_P0_Y[4] = {  120.0f, -120.0f,  120.0f, -120.0f };
    /* P1：展开位*/
    const float TARGET_P1_X[4] = { 250.0f,  250.0f, -250.0f, -250.0f };
    const float TARGET_P1_Y[4] = { 440.0f, -440.0f,  440.0f, -440.0f };

    if (!s_dbg_init) {
        s_dbg_init      = true;
        s_dbg_switch_ms = now_ms;
        s_dbg_idx       = 0U;
    } else if ((now_ms - s_dbg_switch_ms) >= DBG_INTERVAL_MS) {
        s_dbg_switch_ms = now_ms;
        s_dbg_idx ^= 1U;  /* 在 0 和 1 之间交替 */
    }

    if (s_dbg_idx == 0U) {
        planar_robot_arm_set_target(ARM_ID_LF, TARGET_P0_X[0], TARGET_P0_Y[0]);
        planar_robot_arm_set_target(ARM_ID_RF, TARGET_P0_X[1], TARGET_P0_Y[1]);
        planar_robot_arm_set_target(ARM_ID_LB, TARGET_P0_X[2], TARGET_P0_Y[2]);
        planar_robot_arm_set_target(ARM_ID_RB, TARGET_P0_X[3], TARGET_P0_Y[3]);
    } else {
        planar_robot_arm_set_target(ARM_ID_LF, TARGET_P1_X[0], TARGET_P1_Y[0]);
        planar_robot_arm_set_target(ARM_ID_RF, TARGET_P1_X[1], TARGET_P1_Y[1]);
        planar_robot_arm_set_target(ARM_ID_LB, TARGET_P1_X[2], TARGET_P1_Y[2]);
        planar_robot_arm_set_target(ARM_ID_RB, TARGET_P1_X[3], TARGET_P1_Y[3]);
    }
#endif  /* movedebug */
    // const float TARGET_P1_X[4] = { 380.0f,  380.0f, -380.0f, -380.0f };
    // const float TARGET_P1_Y[4] = { 510.0f, -510.0f,  510.0f, -510.0f };

    // const float TARGET_P1_X[4] = { 650.0f,  650.0f, -650.0f, -650.0f };
    // const float TARGET_P1_Y[4] = { 400.0f, -400.0f,  400.0f, -400.0f };
#ifdef USE_DT7_DEBUG
    if(get_remote_control_point()->rc.s[1] == 3) 
    {
        //单独控制一个臂运动
        if (get_remote_control_point()->rc.s[0] == 1)
        {
            if (get_remote_control_point()->rc.ch[0] > 400)
            {
                target_x_test[0] = TARGET_P3_X[0];
                target_y_test[0] = TARGET_P3_Y[0];
            }else {
                target_x_test[0] = TARGET_P2_X[0];
                target_y_test[0] = TARGET_P2_Y[0];
            }
            //LF 放出位置
            if (get_remote_control_point()->rc.ch[0] < -400)
            {
                target_x_test[1] = TARGET_P3_X[1];
                target_y_test[1] = TARGET_P3_Y[1];
            }else {
                target_x_test[1] = TARGET_P2_X[1];
                target_y_test[1] = TARGET_P2_Y[1];
            }
            //LF 放出位置
            //电磁阀控制
            if(get_remote_control_point()->rc.ch[2] > 400)
            {
                    //LB 放出位置
                relay_control(0, 0);
            }else {
                relay_control(0, 1);
            }

            if(get_remote_control_point()->rc.ch[2] < -400)
            {
                    //LB 放出位置
                relay_control(1, 0);
            }else {
                relay_control(1, 1);
            }
        
       }else if (get_remote_control_point()->rc.s[0] == 2)
         {
            if (get_remote_control_point()->rc.ch[0] > 400)
            {
          //RF
          //放置位置
                target_x_test[2] = TARGET_P3_X[2];
                target_y_test[2] = TARGET_P3_Y[2];
            
             }else {
                target_x_test[2] = TARGET_P2_X[2];
                target_y_test[2] = TARGET_P2_Y[2];
            }
            
            if (get_remote_control_point()->rc.ch[0] < -400)
            {
          //RF
          //放置位置
                target_x_test[3] = TARGET_P3_X[3];
                target_y_test[3] = TARGET_P3_Y[3];
            
             }else {
                target_x_test[3] = TARGET_P2_X[3];
                target_y_test[3] = TARGET_P2_Y[3];
            }

            if(get_remote_control_point()->rc.ch[2] > 400)
            {
                    //LB 放出位置
                relay_control(2, 0);
            }else {
                relay_control(2, 1);
            }

            if(get_remote_control_point()->rc.ch[2] < -400)
            {
                    //LB 放出位置
                relay_control(3, 0);
            }else {
                relay_control(3, 1);
            }

         }else if(get_remote_control_point()->rc.s[0] == 3)
         {
            if (get_remote_control_point()->rc.ch[0] > 400)
            {
        //运块的收起位置
        for (int i = 0; i < 4; i++) {
            target_x_test[i] = TARGET_P1_X[i];
            target_y_test[i] = TARGET_P1_Y[i];
        }
            }else {
        //运块的收起位置
                 for (int i = 0; i < 4; i++) {
            target_x_test[i] = TARGET_P2_X[i];
            target_y_test[i] = TARGET_P2_Y[i];
                 }
                }
         }
    }else
    {
        for (int i = 0; i < 4; i++) {
            target_x_test[i] = TARGET_P0_X[i];
            target_y_test[i] = TARGET_P0_Y[i];
        }
    }


    //     if(get_remote_control_point()->rc.s[1] == 3) 
    // {
    //    if (get_remote_control_point()->rc.s[0] == 2)
    // {
    //    if (get_remote_control_point()->rc.ch[0] > 400)
    //    {
    //     //RF
    //         target_x_test[1] = TARGET_P0_X[0];
    //         target_y_test[1] = TARGET_P0_Y[0];
    //     }else {
    //         target_x_test[1] = TARGET_P2_X[0];
    //         target_y_test[1] = TARGET_P2_Y[0];
    //     }
        
    //    }else if (get_remote_control_point()->rc.s[0] == 2)
    //      {
    //       if (get_remote_control_point()->rc.ch[0] < -400)
    //      {
    //       //RF
    //             target_x_test[0] = TARGET_P1_X[0];
    //             target_y_test[0] = TARGET_P1_Y[0];
    //       }else {
    //             target_x_test[0] = TARGET_P3_X[0];
    //             target_y_test[0] = TARGET_P3_Y[0];
    //    }
    // } 

    // if (get_remote_control_point()->rc.s[0] == 1 &&
    //     get_remote_control_point()->rc.s[1] == 3) 
    // {
    //     //归位，放置物块的位置
    //     for (int i = 0; i < 4; i++) {
    //         target_x_test[i] = TARGET_P3_X[i];
    //         target_y_test[i] = TARGET_P3_Y[i];
    //     }
    // }else if (get_remote_control_point()->rc.s[0] == 3 &&
    //           get_remote_control_point()->rc.s[1] == 3) 
    // {
    //     //运块的收起位置
    //     for (int i = 0; i < 4; i++) {
    //         target_x_test[i] = TARGET_P2_X[i];
    //         target_y_test[i] = TARGET_P2_Y[i];
    //     }
    // }else if (get_remote_control_point()->rc.s[0] == 2 &&
    //           get_remote_control_point()->rc.s[1] == 3) 
    // {
    //     //吸取物块的位置
    //     for (int i = 0; i < 4; i++) {
    //         target_x_test[i] = TARGET_P1_X[i];
    //         target_y_test[i] = TARGET_P1_Y[i];
    //     }
    // }else {
    //     for (int i = 0; i < 4; i++) {
    //         target_x_test[i] = TARGET_P0_X[i];
    //         target_y_test[i] = TARGET_P0_Y[i];
    //     }
    // }


    /* ── 路径点状态机：P2→P3 切换时先经过 P4 ──
     * 当目标从 P2（携带位）切换到 P3（伸直放置位）时，
     * 先发往中间路径点 P4，停留 WP_HOLD_MS 后再前进到 P3，
     * 避免末端轨迹突变导致机械结构冲击。            */
    {
        static uint8_t  s_wp_phase[4] = {0,0,0,0}; /* 0=idle/P2, 1=P4, 2=P3 */
        static uint32_t s_wp_tick[4]  = {0,0,0,0};
        const  uint32_t WP_HOLD_MS    = 400U;

        for (int i = 0; i < 4; i++)
        {
            /* 判断当前期望目标是否为 P3（伸直放置位）*/
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
                        /* 仍在 P4，保持目标 */
                        target_x_test[i] = TARGET_P4_X[i];
                        target_y_test[i] = TARGET_P4_Y[i];
                    }
                }
                /* phase == 2: 目标已是 P3，保持不变 */
            }
            else
            {
                /* 目标不是 P3 → 复位状态机 */
                s_wp_phase[i] = 0;
            }
        }
    }

    planar_robot_arm_set_target(ARM_ID_LF, target_x_test[0], target_y_test[0]);
    planar_robot_arm_set_target(ARM_ID_RF, target_x_test[1], target_y_test[1]);
    planar_robot_arm_set_target(ARM_ID_LB, target_x_test[2], target_y_test[2]);
    planar_robot_arm_set_target(ARM_ID_RB, target_x_test[3], target_y_test[3]);

    #endif /* USE_DT7_DEBUG */


    // 批量读取全部舵机位置（8 次 ReadPos，集中完成）。
    // 读取失败时保留缓存值，不中断后续轨迹计算和指令发送。
    (void)batch_read_all_servo_pos();

    // 每周期基于最新反馈做正运动学，更新末端位置数据。
    planar_arm_forward_kinematics_from_cache(&Arm_LF);
    planar_arm_forward_kinematics_from_cache(&Arm_RF);
    planar_arm_forward_kinematics_from_cache(&Arm_LB);
    planar_arm_forward_kinematics_from_cache(&Arm_RB);

    //四臂轨迹计算（纯计算，无 I/O）
    ok &= arm_control_step_with_trajectory(&Arm_LF, g_arm_elbow_up[0], now_ms);
    ok &= arm_control_step_with_trajectory(&Arm_RF, g_arm_elbow_up[1], now_ms);
    ok &= arm_control_step_with_trajectory(&Arm_LB, g_arm_elbow_up[2], now_ms);
    ok &= arm_control_step_with_trajectory(&Arm_RB, g_arm_elbow_up[3], now_ms);

    //8 舵机单次同步写入
    ok &= planar_arm_all_servo_run(&Arm_LF, &Arm_RF, &Arm_LB, &Arm_RB);
    
// #ifdef arm_FKIK_debug
//     ok &= arm_control_step(&Arm_LF, g_arm_elbow_up[0], Arm_LF.end_effector_x, Arm_LF.end_effector_y);
//     ok &= arm_control_step(&Arm_RF, g_arm_elbow_up[1], Arm_RF.end_effector_x, Arm_RF.end_effector_y);
//     ok &= arm_control_step(&Arm_LB, g_arm_elbow_up[2], Arm_LB.end_effector_x, Arm_LB.end_effector_y);
//     ok &= arm_control_step(&Arm_RB, g_arm_elbow_up[3], Arm_RB.end_effector_x, Arm_RB.end_effector_y);
// #endif
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

// 机械臂物块交接状态机
typedef enum 
{

    BLOCK_ASSOCIATE_IDLE,            //空闲状态，等待交接指令
    BLOCK_ASSOCIATE_TO_MIDDLE,       //准备交接，机械臂移动到交接位置
    BLOCK_ASSOCIATE_WAIT,            //等待对方机械臂都到位
    BLOCK_ASSOCIATE_ADSORB,          //等待一定时间吸取物块
    BLOCK_ASSOCIATE_VALVE_CONTROL,   //电磁阀控制启动交接
    BLOCK_ASSOCIATE_COMPLETE,        //交接完成，机械臂移动到目标位置

}block_associate_state_e;

block_associate_state_e LEFT;   //左臂物块交接的状态机
block_associate_state_e RIGHT;  //右臂物块交接的状态机
//此处是机械臂物块交接的控制逻辑

/* ==============================================================
 *  以下为第 1434 行后新增代码：机械臂物块交接状态机完整实现
 *  不能修改此前已有的任何代码
 *
 *  设计说明：
 *    支持两组交接对同时独立运行：
 *      - 前侧 (pair_idx=0): LF(ARM_ID_LF) ↔ RF(ARM_ID_RF)
 *      - 后侧 (pair_idx=1): LB(ARM_ID_LB) ↔ RB(ARM_ID_RB)
 *    每对交接各有独立的状态机上下文，通过 ACTION_loop() 周期驱动。
 *    LEFT / RIGHT 全局变量在 ACTION_loop() 末尾同步刷新，供外部查询。
 *
 *  交接流程：
 *    IDLE → TO_MIDDLE(移动到交接位) → WAIT(等待就绪)
 *    → ADSORB(吸取物块) → VALVE_CONTROL(阀切换) → COMPLETE(完成) → IDLE
 *
 *  使用说明：
 *    1. 在 RTOS 任务或主循环中周期调用 ACTION_loop()（建议 10~20ms 周期）
 *    2. 调用 associate_trigger(pair_idx) 触发指定交接对的移交流程
 *    3. 交接过程中的空间坐标由 ASSOC_xxx 宏定义，可按需修改
 * ============================================================== */

/* ── 交接时序参数（占位符，后续根据实测调整）── */
#define ASSOC_TIMEOUT_MS        5000U   /* 单步最大超时 (ms) */
#define ASSOC_MOVE_DELAY_MS     1500U   /* 机械臂移动到位预估时间 (ms) */
#define ASSOC_WAIT_DELAY_MS     500U    /* 双方到位后的稳定等待 (ms) */
#define ASSOC_ADSORB_TIMEOUT_MS 2000U   /* 吸取物块最大等待时间 (ms) */
#define ASSOC_VALVE_DELAY_MS    300U    /* 电磁阀切换间隔 (ms) */
#define ASSOC_HOLD_MS           500U    /* 交接完成后保持时间 (ms) */

/* ── 交接空间位置（占位坐标，外部坐标系，单位 mm，后续根据实际机械结构调整）── */
#define ASSOC_MID_X         350.0f      /* 交接中间点 X 坐标 */
#define ASSOC_MID_Y         0.0f        /* 交接中间点 Y 坐标 */
#define ASSOC_DONE_LF_X     200.0f      /* LF 完成位 X 坐标 */
#define ASSOC_DONE_LF_Y     100.0f      /* LF 完成位 Y 坐标 */
#define ASSOC_DONE_RF_X     200.0f      /* RF 完成位 X 坐标 */
#define ASSOC_DONE_RF_Y    -100.0f      /* RF 完成位 Y 坐标 */
#define ASSOC_DONE_LB_X    -200.0f      /* LB 完成位 X 坐标 */
#define ASSOC_DONE_LB_Y     100.0f      /* LB 完成位 Y 坐标 */
#define ASSOC_DONE_RB_X    -200.0f      /* RB 完成位 X 坐标 */
#define ASSOC_DONE_RB_Y    -100.0f      /* RB 完成位 Y 坐标 */

/* ── 内部状态机上下文结构体 ──
 *  每对交接臂独立拥有一个实例，封装当前状态、计时信息、吸附标志等。
 */
typedef struct {
    block_associate_state_e state;         /* 当前主状态 */
    uint32_t                enter_tick;    /* 进入当前状态的系统 tick */
    uint32_t                state_timeout; /* 当前状态的超时时间 (ms)，0 表示无超时 */
    bool                    block_grabbed; /* 物块已成功被吸附 */
} AssociateCtx;

/* 两组交接对的内部上下文：
 *   idx 0 = 前侧 (LF ↔ RF)
 *   idx 1 = 后侧 (LB ↔ RB)
 */
static AssociateCtx s_assoc_ctx[2] = {
    { .state = BLOCK_ASSOCIATE_IDLE, .enter_tick = 0, .state_timeout = 0, .block_grabbed = false },
    { .state = BLOCK_ASSOCIATE_IDLE, .enter_tick = 0, .state_timeout = 0, .block_grabbed = false },
};

/* ── assoc_set_state ──
 *  功能：切换到新状态，记录进入时刻和超时时间。
 *  参数：
 *    pair_idx  : 交接对索引（0=前侧，1=后侧）
 *    new_state : 目标状态
 *    timeout_ms: 该状态的超时时间 (ms)，0 表示永不超时
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

/* ── assoc_is_timed_out ──
 *  功能：检查当前状态是否超时。
 *  返回 true 表示超时（仅当 state_timeout > 0 时有效）。
 */
static inline bool assoc_is_timed_out(const AssociateCtx *ctx)
{
    if (ctx == NULL || ctx->state_timeout == 0) {
        return false;
    }
    return ((HAL_GetTick() - ctx->enter_tick) >= ctx->state_timeout);
}

/* ── associate_run_one_pair ──
 *  功能：单对交接臂的状态机核心推进逻辑。
 *  根据当前状态执行相应动作，并在条件满足时切换到下一状态。
 *  所有状态均包含超时保护，防止流程卡死。
 *  参数：
 *    pair_idx: 交接对索引（0=前侧，1=后侧）
 */
static void associate_run_one_pair(uint8_t pair_idx)
{
    if (pair_idx >= 2) {
        return;
    }

    AssociateCtx *ctx = &s_assoc_ctx[pair_idx];

    /* 空闲态不执行任何动作 */
    if (ctx->state == BLOCK_ASSOCIATE_IDLE) {
        return;
    }

    /* 超时保护：任何非空闲状态超时后自动回退到 IDLE，
     * 防止因机械故障、通信中断等异常导致流程死锁。 */
    if (assoc_is_timed_out(ctx)) {
        assoc_set_state(pair_idx, BLOCK_ASSOCIATE_IDLE, 0);
        return;
    }

    /* ── 根据交接对索引选择对应的机械臂 ID 和电磁阀 ID ── */
    arm_id_e armA, armB;       /* armA = 供给方, armB = 接收方 */
    uint8_t  solenoidA, solenoidB;
    float    done_xA, done_yA, done_xB, done_yB;

    if (pair_idx == 0) {
        /* 前侧：LF（供给方）↔ RF（接收方） */
        armA      = ARM_ID_LF;
        armB      = ARM_ID_RF;
        solenoidA = SOLENOID_1;
        solenoidB = SOLENOID_2;
        done_xA   = ASSOC_DONE_LF_X;
        done_yA   = ASSOC_DONE_LF_Y;
        done_xB   = ASSOC_DONE_RF_X;
        done_yB   = ASSOC_DONE_RF_Y;
    } else {
        /* 后侧：LB（供给方）↔ RB（接收方） */
        armA      = ARM_ID_LB;
        armB      = ARM_ID_RB;
        solenoidA = SOLENOID_3;
        solenoidB = SOLENOID_4;
        done_xA   = ASSOC_DONE_LB_X;
        done_yA   = ASSOC_DONE_LB_Y;
        done_xB   = ASSOC_DONE_RB_X;
        done_yB   = ASSOC_DONE_RB_Y;
    }

    /* ── 状态机主分支 ── */
    switch (ctx->state)
    {
        case BLOCK_ASSOCIATE_TO_MIDDLE:
            /* Step 1: 双方机械臂同时向交接中间位置移动。
             * 实际轨迹规划由 controlA_loop 完成，此处仅设置目标点。
             * 等待 MOVE_DELAY 时间后视为到位，转入下一步。 */
            planar_robot_arm_set_target(armA, ASSOC_MID_X, ASSOC_MID_Y);
            planar_robot_arm_set_target(armB, ASSOC_MID_X, ASSOC_MID_Y);
            if ((HAL_GetTick() - ctx->enter_tick) >= ASSOC_MOVE_DELAY_MS) {
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_WAIT, ASSOC_TIMEOUT_MS);
            }
            break;

        case BLOCK_ASSOCIATE_WAIT:
            /* Step 2: 等待双方机械臂稳定就绪。
             * 当前实现采用固定延时，后续可替换为基于末端位置误差的判断逻辑。 */
            if ((HAL_GetTick() - ctx->enter_tick) >= ASSOC_WAIT_DELAY_MS) {
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_ADSORB, ASSOC_ADSORB_TIMEOUT_MS);
            }
            break;

        case BLOCK_ASSOCIATE_ADSORB:
            /* Step 3: 开启供给方电磁阀/真空泵，尝试吸附物块。
             * 通过微动开关（g_switch_input）检测物块是否已吸附。
             * 超时后无论是否吸附成功都继续推进，防止流程卡死。 */
            Solenoid_Valve_control(solenoidA, 1);
            if (g_switch_input.state[armA] != 0) {
                ctx->block_grabbed = true;
            }
            if (ctx->block_grabbed) {
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_VALVE_CONTROL, ASSOC_TIMEOUT_MS);
            }
            /* 超时后即使未吸附也尝试交接（容错处理） */
            if (assoc_is_timed_out(ctx)) {
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_VALVE_CONTROL, ASSOC_TIMEOUT_MS);
            }
            break;

        case BLOCK_ASSOCIATE_VALVE_CONTROL:
            /* Step 4: 电磁阀切换控制。
             * 先关闭供给方电磁阀释放物块，延时后开启接收方电磁阀吸附物块。
             * 两步之间的延时防止气路串扰导致物块掉落。 */
            Solenoid_Valve_control(solenoidA, 0);
            if ((HAL_GetTick() - ctx->enter_tick) >= ASSOC_VALVE_DELAY_MS) {
                Solenoid_Valve_control(solenoidB, 1);
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_COMPLETE, ASSOC_TIMEOUT_MS);
            }
            break;

        case BLOCK_ASSOCIATE_COMPLETE:
            /* Step 5: 交接完成，双方机械臂移动到各自的目标完成位置。
             * 短暂保持后自动回到 IDLE，准备接受下一次交接指令。 */
            planar_robot_arm_set_target(armA, done_xA, done_yA);
            planar_robot_arm_set_target(armB, done_xB, done_yB);
            if ((HAL_GetTick() - ctx->enter_tick) >= ASSOC_HOLD_MS) {
                ctx->block_grabbed = false;
                assoc_set_state(pair_idx, BLOCK_ASSOCIATE_IDLE, 0);
            }
            break;

        default:
            /* 未知状态防御性处理：回退到 IDLE */
            assoc_set_state(pair_idx, BLOCK_ASSOCIATE_IDLE, 0);
            break;
    }
}

/* ── associate_trigger ──
 *  功能：外部触发指定交接对开始物块移交流程。
 *  仅在交接对当前处于 IDLE 状态时才能成功触发。
 *  参数：
 *    pair_idx: 交接对索引（0=前侧 LF↔RF，1=后侧 LB↔RB）
 *  返回：
 *    true  = 成功触发（已切换到 TO_MIDDLE）
 *    false = 触发失败（交接对正忙，流程未完成）
 */
bool associate_trigger(uint8_t pair_idx)
{
    if (pair_idx >= 2) {
        return false;
    }
    if (s_assoc_ctx[pair_idx].state != BLOCK_ASSOCIATE_IDLE) {
        /* 当前交接流程尚未完成，拒绝重复触发 */
        return false;
    }
    assoc_set_state(pair_idx, BLOCK_ASSOCIATE_TO_MIDDLE, ASSOC_TIMEOUT_MS);
    return true;
}

/* ── associate_abort ──
 *  功能：强制中止指定交接对的当前流程，立即回到 IDLE。
 *  可用于紧急停止或故障恢复场景。
 *  参数：
 *    pair_idx: 交接对索引（0=前侧，1=后侧）
 */
void associate_abort(uint8_t pair_idx)
{
    if (pair_idx >= 2) {
        return;
    }
    AssociateCtx *ctx = &s_assoc_ctx[pair_idx];
    ctx->block_grabbed = false;
    assoc_set_state(pair_idx, BLOCK_ASSOCIATE_IDLE, 0);
}

/* ── associate_get_state ──
 *  功能：查询指定交接对的当前状态。
 *  参数：
 *    pair_idx: 交接对索引（0=前侧，1=后侧）
 *  返回：
 *    当前状态枚举值的 uint8_t 表示，非法索引返回 BLOCK_ASSOCIATE_IDLE
 */
uint8_t associate_get_state(uint8_t pair_idx)
{
    if (pair_idx >= 2) {
        return (uint8_t)BLOCK_ASSOCIATE_IDLE;
    }
    return (uint8_t)s_assoc_ctx[pair_idx].state;
}

/* ── ACTION_loop ──
 *  功能：专用动作循环，需在 RTOS 任务或主循环中周期调用（建议 10~20ms 周期）。
 *  负责：
 *    1. 驱动所有交接对的状态机推进
 *    2. 同步更新 LEFT / RIGHT 全局状态变量，供外部模块查询
 *
 *  集成方式示例（在 arm_control_task 或新建任务中调用）：
 *    for (;;) {
 *        planar_arm_control_loop();  // 原有机械臂控制
 *        ACTION_loop();              // 新增动作处理
 *        osDelay(5);
 *    }
 */
void ACTION_loop(void)
{
    /* 驱动前侧交接对（LF ↔ RF）状态机 */
    associate_run_one_pair(0);

    /* 驱动后侧交接对（LB ↔ RB）状态机 */
    associate_run_one_pair(1);

    /* ── 同步外部全局状态变量 ──
     * LEFT  = 前侧交接对的状态（LF 视角）
     * RIGHT = 后侧交接对的状态（RB 视角）
     * 此映射关系可根据实际业务逻辑调整。 */
    LEFT  = s_assoc_ctx[0].state;
    RIGHT = s_assoc_ctx[1].state;
}

