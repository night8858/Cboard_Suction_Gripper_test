#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "Planar_Robot_Arm.h"

//#include "my_feetech.h"
#include "SCSerail.h"
#include "SCS.h"
#include "SMS_STS.h"
#include "SCSCL.h"
#include "usart.h"
#include "variables.h"

#include "cmsis_os.h"

/*
DH参数
*/
//选择规划方式
/* 轨迹插补模式选择：
 *   注释此宏 → 五次多项式插补（默认，平滑加减速）
 *   定义此宏 → 线性插补（匀速，计算量更小）
 */
#define TRAJ_LINEAR_INTERPOLATION


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARM1_LINK1_LONG_EDGE_MM 290.1f
#define ARM1_LINK1_SHORT_EDGE_MM 25.0f
#define ARM2_LINK2_LONG_EDGE_MM 337.27f

//#define arm_FKIK_debug 
#define movedebug 

// 机械臂参数配置和控制函数实现
extern Planar_Robot_Arm Arm_LF;       //左前
extern Planar_Robot_Arm Arm_RF;       //右前
extern Planar_Robot_Arm Arm_LB;       //左后
extern Planar_Robot_Arm Arm_RB;       //右后

static ArmTrajectoryContext g_ctx_lf = {0};
static ArmTrajectoryContext g_ctx_rf = {0};
static ArmTrajectoryContext g_ctx_lb = {0};
static ArmTrajectoryContext g_ctx_rb = {0};

// 每只机械臂的当前肘型配置：LF/RF 默认上肘，LB/RB 默认下肘。
static bool g_arm_elbow_up[4] = {true, false, false, true};




static float deg_to_rad(float deg)
{
    return deg * ((float)M_PI / 180.0f);
}

static float rad_to_deg(float rad)
{
    return rad * (180.0f / (float)M_PI);
}

// 将任意角度统一为“相对 x 正半轴”的有符号夹角，范围 [-180, 180]。
static float normalize_to_x_positive_axis_deg(float deg)
{
    float rad = deg_to_rad(deg);
    return rad_to_deg(atan2f(sinf(rad), cosf(rad)));
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

    planar_arm_forward_kinematics(arm);
    arm->state = ARM_STATE_IDLE;

}

// void planar_robot_arm_move_to_position(Planar_Robot_Arm *arm, float x, float y)
// {
//     if (arm == NULL) {
//         return;
//     }

//     arm->end_aim_x = x;
//     arm->end_aim_y = y;
//     bool solved = planar_arm_inverse_kinematics(arm, true);
//     if (!solved) {
//         arm->state = ARM_STATE_ERROR;
//         EnableTorque(arm->SERVO_ID1, 0); 
//         EnableTorque(arm->SERVO_ID2, 0); // 进入错误状态，禁用舵机扭矩
//         return;
//     }

//     planar_arm_forward_kinematics(arm);

//     arm->target_servo_positions[0] = deg_to_servo_pos(arm->target_joint_angles[0]);
//     arm->target_servo_positions[1] = deg_to_servo_pos(arm->target_joint_angles[1]);

//     uint8_t ID[2]       = {(uint8_t)arm->SERVO_ID1, (uint8_t)arm->SERVO_ID2}; // 舵机ID数组
//     int16_t Position[2] = {arm->target_servo_positions[0], arm->target_servo_positions[1]}; // 目标位置数组
//     uint16_t Speed[2]   = {800, 800}; // 速度数组
//     uint8_t ACC[2]      = {0, 0}; // 加速度数组

//      // 同步写位置指令，控制多个舵机同时运动
//     //SyncWritePosEx(ID, 2, Position, Speed, ACC);
//     // 使用舵机控制器发送位置命令
//     // WritePosEx(1, joint1_pos, 100, 10); // 控制第1个舵机
//     // WritePosEx(2, joint2_pos, 100, 10); // 控制第2个舵机

//}

// 获取机械臂当前状态，更新arm结构体中的状态信息
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

    // arm->servo_move_state[0] = ReadMove(arm->SERVO_ID1); // 更新第1个舵机运动状态
    // arm->servo_move_state[1] = ReadMove(arm->SERVO_ID2); // 更新第2个舵机运动状态

    if (pos1 < 0 || pos2 < 0) {
        arm->state = ARM_STATE_ERROR;
        return;
    }

    arm->current_servo_positions[0] = pos1 ;//* (arm->servo_Reverse_installation[0] ? -1 : 1); // 获取第1个舵机位置
    arm->current_servo_positions[1] = pos2 ;//* (arm->servo_Reverse_installation[1] ? -1 : 1); // 获取第2个舵机位置

        switch (arm->arm_id) {
        case ARM_ID_LF:
            // 左前机械臂的舵机反馈处理
                arm->current_joint_angles[0] = 
                (arm->current_servo_positions[0] / 4095.0 * 360.0 ) - 180.0; // 单位转换为角度
                
                arm->current_joint_angles[1] = 
                (arm->current_servo_positions[1] / 4095.0 * 360.0 ) - 180.0; // 单位转换为角度

            break;
        case ARM_ID_RF:
                arm->current_joint_angles[0] = 
                (arm->current_servo_positions[0] / 4095.0 * 360.0 ) - 180.0; // 单位转换为角度
                
                arm->current_joint_angles[1] = 
                (arm->current_servo_positions[1] / 4095.0 * 360.0 ) - 180.0; // 单位转换为角度

            // 右前机械臂的舵机反馈处理
            break;
        case ARM_ID_LB:
            // 左后机械臂的舵机反馈处理
                arm->current_joint_angles[0] = 
                (arm->current_servo_positions[0] / 4095.0 * 360.0 ) - 180.0; // 单位转换为角度
                
                arm->current_joint_angles[1] = 
                (arm->current_servo_positions[1] / 4095.0 * 360.0 ) - 180.0; // 单位转换为角度

            break;
        case ARM_ID_RB:
            // 右后机械臂的舵机反馈处理
                arm->current_joint_angles[0] = 
                (arm->current_servo_positions[0] / 4095.0 * 360.0 ) - 180.0; // 单位转换为角度
                
                arm->current_joint_angles[1] = 
                (arm->current_servo_positions[1] / 4095.0 * 360.0 ) - 180.0; // 单位转换为角度

            break;
        default:
            // 未知机械臂ID，进入错误状态
            arm->state = ARM_STATE_ERROR;
            return;
    }
    // 根据舵机位置转换为关节角度，更新arm结构体中的关节角度信息

}

// 机械臂正运动学计算函数，根据各关节角度计算末端执行器的位姿[good]
void planar_arm_forward_kinematics(Planar_Robot_Arm *arm)
{
    if (arm == NULL) {
        return;
    }
    get_arm_servo_pos(arm);  // 获取当前舵机位置并更新关节角度信息（初始化时使用）
    // 主控制环中由 batch_read_all_servo_pos() 替代此调用
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

//需要修正偏置
bool planar_arm_inverse_kinematics(Planar_Robot_Arm *arm,
                                   bool elbow_up)
{
    if (arm == NULL) {
        return false;
    }

    // 根据elbow_up参数选择肘部方向，计算关节角度
    float x = 0.0f;
    float y = 0.0f;
    external_to_kinematics_xy(arm, arm->end_aim_x, arm->end_aim_y, &x, &y);

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
    if (arm->target_joint_angles[1] >180.0f) 
    {
    arm->target_joint_angles[1] -= 180.0f;
    }else if(arm->target_joint_angles[1] < -180.0f)
    {
        arm->target_joint_angles[1] += 180.0f;
    }

    // 逆解输出角度统一定义为“相对 x 正半轴”的夹角。
    // arm->target_joint_angles[0] = normalize_to_x_positive_axis_deg(arm->target_joint_angles[0]);
    // arm->target_joint_angles[1] = normalize_to_x_positive_axis_deg(arm->target_joint_angles[1]);
    // // arm->target_joint_angles[0] = clampf(arm->target_joint_angles[0], -90.0f, 90.0f);
    // arm->target_joint_angles[1] = clampf(arm->target_joint_angles[1], -90.0f, 90.0f);

    return true;

}

// 单臂控制步骤：逆运动学 -> 发送位置指令（反馈由调用方在外部统一完成，避免重复读取）
static __attribute__((unused)) bool arm_control_step(Planar_Robot_Arm *arm, bool elbow_up ,float x, float y)
{
    if (arm == NULL) {
        return false;
    }
    planar_arm_forward_kinematics(arm); // 获取当前末端位姿，更新arm结构体中的位置信息
    // 逆运动学求解目标关节角
    arm->end_aim_x = x;
    arm->end_aim_y = y;
    bool solved = planar_arm_inverse_kinematics(arm, elbow_up);
    if (!solved) 
    {
        arm->state = ARM_STATE_ERROR;
        EnableTorque(arm->SERVO_ID1, 0);
        EnableTorque(arm->SERVO_ID2, 0);
        return false;
    }

    arm->state = ARM_STATE_MOVING;

    // 3. 将目标关节角转换为舵机位置并同步写入
    arm->target_servo_positions[0] =  ((arm->target_joint_angles[0]  + 180.0f) / 360.0f * 4095.0f);
    arm->target_servo_positions[1] =  ((arm->target_joint_angles[1]  + 180.0f) / 360.0f * 4095.0f);

    arm->target_servo_positions[0] = (int16_t)clampf(arm->target_servo_positions[0], 1024.0f, 3072.0f);
    arm->target_servo_positions[1] = (int16_t)clampf(arm->target_servo_positions[1], 1024.0f, 3072.0f);
    // 保留转换逻辑，发送接口在此路径暂未启用。
    // (void)arm->target_servo_positions[0];
    // (void)arm->target_servo_positions[1];
    uint8_t  ID[2]       = {arm->SERVO_ID1, arm->SERVO_ID2}; // 舵机ID数组
    int16_t  Position[2] = {arm->target_servo_positions[0], arm->target_servo_positions[1]}; // 目标位置数组
    uint16_t Speed[2]   = {0, 0}; // 速度数组
    uint8_t  ACC[2]      = {0, 0}; // 加速度数组

    // WritePosEx(arm->SERVO_ID1, arm->target_servo_positions[0], Speed[0], ACC[0]);
    // osDelay(1);
    // WritePosEx(arm->SERVO_ID2, arm->target_servo_positions[1], Speed[1], ACC[1]);
    
    SyncWritePosEx(ID, 2, Position, Speed, ACC);

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
    uint16_t Speed[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 速度数组
    uint8_t ACC[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 加速度数组

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
static bool read_one_arm_servo_pos(Planar_Robot_Arm *arm)
{
    if (arm == NULL) return false;
    int16_t pos1 = ReadPos(arm->SERVO_ID1);
    int16_t pos2 = ReadPos(arm->SERVO_ID2);
    if (pos1 < 0 || pos2 < 0) {
        arm->state = ARM_STATE_ERROR;
        return false;
    }
    arm->current_servo_positions[0] = pos1;
    arm->current_servo_positions[1] = pos2;
    update_joint_angles_from_servo_pos(arm);
    return true;
}

// 批量读取全部 4 臂 8 舵机位置，一次完成所有 I/O。
bool batch_read_all_servo_pos(void)
{
    bool ok = true;
    ok &= read_one_arm_servo_pos(&Arm_LF);
    ok &= read_one_arm_servo_pos(&Arm_RF);
    ok &= read_one_arm_servo_pos(&Arm_LB);
    ok &= read_one_arm_servo_pos(&Arm_RB);
    return ok;
}

bool planar_robot_arm_all_init(void)
{
    SCS_SetUART(&huart1);
    setEnd(0);
    // 配置四个机械臂参数，分别为左前、右前、左后、右后
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_LF, 1, 2, 
                                 ARM_ID_LF, 0.0f, 0.0f,  -90.0f, 90.0f);
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_RF, 3, 4,
                                 ARM_ID_RF, 0.0f, 0.0f, 90.0f,  -90.0f);
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_LB, 5, 6,
                                 ARM_ID_LB, 0.0f, 0.0f, -90.0f, -90.0f);
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_RB, 7, 8,
                                 ARM_ID_RB, 0.0f, 0.0f, 90.0f, 90.0f);
    // 初始位置设置，测试用
        return true;

}

// int16_t* planar_arm_control_cmd(int16_t* cmd_array, size_t array_size)
// {
//     static int16_t cmd_buffer[4] = {0}; // 静态缓冲区存储命令数据

//     if (cmd_array == NULL || array_size < 4) {
//         return NULL; // 输入无效，返回NULL
//     }

//     // 从输入数组中提取命令数据，假设前4个元素分别对应四个机械臂的目标位置
//     for (size_t i = 0; i < 4; i++) {
//         cmd_buffer[i] = cmd_array[i];
//     }

//     return cmd_buffer; // 返回指向命令数据的指针
// }

// bool planar_robot_arm_process_cmd(int16_t* cmd_array, size_t array_size)
// {
//     int16_t* cmd_buffer = planar_arm_control_cmd(cmd_array, array_size);
//     if (cmd_buffer == NULL) {
//         return false; // 命令处理失败
//     }

//     // 解析命令数据，更新四个机械臂的目标位置
//     Arm_LF.end_aim_x = (float)cmd_buffer[0];
//     Arm_RF.end_aim_x = (float)cmd_buffer[1];
//     Arm_LB.end_aim_x = (float)cmd_buffer[2];
//     Arm_RB.end_aim_x = (float)cmd_buffer[3];

//     // 可以添加更多的命令解析逻辑，例如根据命令类型设置不同的目标位置或执行不同的动作

//     return true; // 命令处理成功
// }





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
bool planar_robot_arm_set_target(arm_id_e arm_id, float target_x, float target_y)
{
    Planar_Robot_Arm *arm = planar_robot_arm_get_by_id(arm_id);
    if (arm == NULL) {
        return false;
    }

    arm->end_aim_x = target_x;
    arm->end_aim_y = target_y;
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

#ifndef TRAJ_LINEAR_INTERPOLATION
    /* 五次多项式插补：满足起终点位置/速度/加速度边界条件 */
    solve_quintic_coefficients(start_pos,
                               start_vel,
                               start_acc,
                               end_pos,
                               end_vel,
                               end_acc,
                               duration_sec,
                               planner->current_segment.coeffs);
#else
    /* 线性插补：p(t) = p0 + (pf - p0) / T * t，忽略初速度边界条件 */
    planner->current_segment.coeffs[0] = start_pos;
    planner->current_segment.coeffs[1] = (end_pos - start_pos) / duration_sec;
    planner->current_segment.coeffs[2] = 0.0f;
    planner->current_segment.coeffs[3] = 0.0f;
    planner->current_segment.coeffs[4] = 0.0f;
    planner->current_segment.coeffs[5] = 0.0f;
#endif
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

/* 目标偶差超过此阈値即强制重规划，防止轨迹执行期间目标漂移过大导致卡死。 */
#define CONTROLA_EMERGENCY_REPLAN_MM 200.0f
static bool target_emergency(const ArmTrajectoryContext *ctx, float x, float y)
{
    if (ctx == NULL) {
        return false;
    }
    return (fabsf(x - ctx->last_cmd_x) > CONTROLA_EMERGENCY_REPLAN_MM) ||
           (fabsf(y - ctx->last_cmd_y) > CONTROLA_EMERGENCY_REPLAN_MM);
}

// 对单臂执行“反馈 -> 必要时重规划 -> 轨迹采样 -> 控制输出”。
static bool arm_control_step_with_trajectory(Planar_Robot_Arm *arm, bool elbow_up, uint32_t now_ms)
{
    if (arm == NULL) {
        return false;
    }

    ArmTrajectoryContext *ctx = get_arm_traj_ctx(arm);
    if (ctx == NULL) {
        return false;
    }

    /* 当前轨迹是否仍在运行 */
    bool traj_running = ctx->initialized &&
                        (ctx->planner_j1.is_running || ctx->planner_j2.is_running);

    /* 重规划策略：
     *  1. 首次初始化
     *  2. 轨迹已自然结束 且 目标发生变化 → 执行下一段规划
     *  3. 轨迹运行中 但 目标偏差超出紧急阈値 → 强制重规划防卡死
     * 轨迹执行期间忽略小幅目标更新，保持当前轨迹平滑运行。 */
    bool need_replan = !ctx->initialized ||
                       (!traj_running && target_changed(ctx, arm->end_aim_x, arm->end_aim_y)) ||
                       (traj_running  && target_emergency(ctx, arm->end_aim_x, arm->end_aim_y));

    if (need_replan) {
        /* Snapshot the user target BEFORE IK to prevent end_aim_x/y corruption
           from polluting the replan trigger check in subsequent cycles. */
        arm->planned_target_x = arm->end_aim_x;
        arm->planned_target_y = arm->end_aim_y;

        read_one_arm_servo_pos(arm);

        if (arm->state == ARM_STATE_ERROR) {
            return false;
        }
        float cur_j1 = (float)arm->current_servo_positions[0];
        float cur_j2 = (float)arm->current_servo_positions[1];

        /* Solve IK once for the new target */
        bool solved = planar_arm_inverse_kinematics(arm, elbow_up);
        if (!solved) {
            arm->state = ARM_STATE_ERROR;
            EnableTorque(arm->SERVO_ID1, 0);
            EnableTorque(arm->SERVO_ID2, 0);
            return false;
        }

        /* Convert target joint angles to servo step counts */
        float tgt_j1 = clampf(
            (arm->target_joint_angles[0] + 180.0f) / 360.0f * 4095.0f,
            1024.0f, 3072.0f);
        float tgt_j2 = clampf(
            (arm->target_joint_angles[1] + 180.0f) / 360.0f * 4095.0f,
            1024.0f, 3072.0f);

        /* Guard against NaN/Inf from IK numerical failure (e.g. atan2(0,0),
           sqrtf of negative, or out-of-workspace target) */
        if (!isfinite(tgt_j1) || !isfinite(tgt_j2)) {
            arm->state = ARM_STATE_ERROR;
            return false;
        }

        // 重规划时继承当前轨迹速度，避免发生速度突变导致卡顿
        float cur_vel_j1 = 0.0f, cur_vel_j2 = 0.0f;
        if (ctx->initialized) {
            update_trajectory(&ctx->planner_j1, now_ms, NULL, &cur_vel_j1, NULL);
            update_trajectory(&ctx->planner_j2, now_ms, NULL, &cur_vel_j2, NULL);
        }
        /* Cap inherited velocity: max-speed quintic over 2048 steps in
           CONTROLA_TRAJ_DURATION_S peaks at ~1.875*(2048/T) steps/s ~= 2400.           Clamp tighter to prevent overshooting on mid-motion replan. */
        {
            const float max_replan_vel = 1500.0f;
            cur_vel_j1 = clampf(cur_vel_j1, -max_replan_vel, max_replan_vel);
            cur_vel_j2 = clampf(cur_vel_j2, -max_replan_vel, max_replan_vel);
        }

        /* Plan quintic polynomial trajectories in joint (servo-step) space */
        plan_new_move(&ctx->planner_j1, now_ms,
                      cur_j1, cur_vel_j1, 0.0f,
                      tgt_j1, 0.0f, 0.0f,
                      CONTROLA_TRAJ_DURATION_S);
        plan_new_move(&ctx->planner_j2, now_ms,
                      cur_j2, cur_vel_j2, 0.0f,
                      tgt_j2, 0.0f, 0.0f,
                      CONTROLA_TRAJ_DURATION_S);

        /* Use the snapshot value so last_cmd always reflects the true user
           intent, never a value potentially modified by IK internals. */
        ctx->last_cmd_x  = arm->planned_target_x;
        ctx->last_cmd_y  = arm->planned_target_y;
        ctx->initialized = true                                                                                                 ;
    }

    // 采样轨迹得到本控制周期目标点。若轨迹结束，输出会自动钳位到终点。
    /* Sample joint-space trajectories for this control cycle */
    float j1_cmd = ctx->planner_j1.current_segment.pf;
    float j2_cmd = ctx->planner_j2.current_segment.pf;
    update_trajectory(&ctx->planner_j1, now_ms, &j1_cmd, NULL, NULL);
    update_trajectory(&ctx->planner_j2, now_ms, &j2_cmd, NULL, NULL);

    // arm_control_step 会把 end_aim 写成当前插值点。这里在调用后恢复外部目标点，
    // 避免下一周期误判“目标已变更”并把轨迹重规划回当前位置。
    arm->target_servo_positions[0] = (int16_t)clampf(j1_cmd, 1024.0f, 3072.0f);
    arm->target_servo_positions[1] = (int16_t)clampf(j2_cmd, 1024.0f, 3072.0f);

    arm->state = ARM_STATE_MOVING;
    return true;
}

bool controlA_loop(void)
{
    bool ok = true;
    uint32_t now_ms = HAL_GetTick();
#ifdef movedebug
    // LF单臂调试：每3秒在两个占位符目标点之间切换。
    static bool lf_toggle_initialized = false;
    static uint8_t lf_target_index = 0U;
    static uint32_t lf_last_switch_ms = 0U;
    const uint32_t lf_switch_interval_ms = 3000U;

    // TODO: 将下面4个占位符替换成你的实际目标点坐标（单位:mm）。
    const float TARGET_P0_X[4]= {-50.0f , -50.0f , 50.0f ,50.0f  }; // 占位符点0 X
    const float TARGET_P0_Y[4]= {20.0f , - 20.0f , 20.0f ,-20.0f }; // 占位符点0 Y
    const float TARGET_P1_X[4]= {50.0f , 50.0f , -50.0f , -50.0f }; // 占位符点1 X
    const float TARGET_P1_Y[4]= {540.0f , -540.0f , 540.0f , -540.0f }; // 占位符点1 Y

    if (!lf_toggle_initialized) {
        lf_toggle_initialized = true;
        lf_last_switch_ms = (uint32_t)now_ms;
        lf_target_index = 0U;
    } else if (((uint32_t)now_ms - lf_last_switch_ms) >= lf_switch_interval_ms) {
        lf_last_switch_ms = (uint32_t)now_ms;
        lf_target_index ^= 1U;
    }

    if (lf_target_index == 0U) {
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
#endif
    // Phase 1: 批量读取全部舵机位置（8 次 ReadPos，集中完成）
    //batch_read_all_servo_pos();

    // Phase 2: 四臂轨迹计算（纯计算，无 I/O）
    ok &= arm_control_step_with_trajectory(&Arm_LF, g_arm_elbow_up[0], now_ms);
    ok &= arm_control_step_with_trajectory(&Arm_RF, g_arm_elbow_up[1], now_ms);
    ok &= arm_control_step_with_trajectory(&Arm_LB, g_arm_elbow_up[2], now_ms);
    ok &= arm_control_step_with_trajectory(&Arm_RB, g_arm_elbow_up[3], now_ms);

    // Phase 3: 8 舵机单次同步写入
    planar_arm_all_servo_run(&Arm_LF, &Arm_RF, &Arm_LB, &Arm_RB);
    
#ifdef arm_FKIK_debug
    ok &= arm_control_step(&Arm_LF, g_arm_elbow_up[0], Arm_LF.end_effector_x, Arm_LF.end_effector_y);
    ok &= arm_control_step(&Arm_RF, g_arm_elbow_up[1], Arm_RF.end_effector_x, Arm_RF.end_effector_y);
    ok &= arm_control_step(&Arm_LB, g_arm_elbow_up[2], Arm_LB.end_effector_x, Arm_LB.end_effector_y);
    ok &= arm_control_step(&Arm_RB, g_arm_elbow_up[3], Arm_RB.end_effector_x, Arm_RB.end_effector_y);
#endif

    return ok;

}

// 保留原接口，避免现有任务代码改动；内部统一走 controlA_loop。
bool planar_arm_control_loop(void)
{
    return controlA_loop();
}