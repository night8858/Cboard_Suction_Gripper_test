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


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARM1_LINK1_LONG_EDGE_MM 290.1f
#define ARM1_LINK1_SHORT_EDGE_MM 25.0f
#define ARM2_LINK2_LONG_EDGE_MM 337.27f


// 机械臂参数配置和控制函数实现
extern Planar_Robot_Arm Arm_LF;       //左前
extern Planar_Robot_Arm Arm_RF;       //右前
extern Planar_Robot_Arm Arm_LB;       //左后
extern Planar_Robot_Arm Arm_RB;       //右后

static int read_servo_pos_with_retry(uint8_t id);


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
    return -y_external;
}

static float y_kinematics_to_external(float y_kinematics)
{
    return -y_kinematics;
}



static float arm_link1_theta_bias_rad(const Planar_Robot_Arm *arm)
{
    if (arm == NULL) {
        return 0.0f;
    }

    // ARM_TYPE_1 第一连杆是斜边，第二关节相对斜边主方向存在固定偏角。
    if (arm->arm_id == ARM_TYPE_1) {
        return atan2f(ARM1_LINK1_SHORT_EDGE_MM, ARM1_LINK1_LONG_EDGE_MM);
    }else if (arm->arm_id == ARM_TYPE_2) {
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

static int16_t deg_to_servo_pos(float deg)
{
    // float normalized = fmodf(deg, 360.0f);
    // if (normalized < 0.0f) 
    // {
    //     normalized += 360.0f;
    // }
    return (int16_t)((deg  + 180.0f) / 360.0f * 4095.0f);
}

void planar_robot_arm_config_init(int arm_type , Planar_Robot_Arm *arm ,
                                  uint8_t SERVO_ID1 , uint8_t SERVO_ID2,
                                  uint8_t ARM_ID,
                                  float origin_point_x_offset , 
                                  float origin_point_y_offset , 
                                  float servo_angle_offset1   ,
                                  float servo_angle_offset2)
{

    if (arm == NULL) {
        return;
    }

    arm->arm_id = (uint8_t)arm_type;
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
    osDelay(1); // 延时1毫秒
    int16_t pos2 = ReadPos(arm->SERVO_ID2);
    osDelay(1); // 延时1毫秒

    arm->servo_move_state[0] = ReadMove(arm->SERVO_ID1); // 更新第1个舵机运动状态
    arm->servo_move_state[1] = ReadMove(arm->SERVO_ID2); // 更新第2个舵机运动状态

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
    get_arm_servo_pos(arm);  // 获取当前舵机位置并更新关节角度信息

    switch (arm->arm_id) 
    {
        case ARM_ID_LF:
            // 左前机械臂的正运动学计算
            float theta1 =  -deg_to_rad(arm->current_joint_angles[0] + arm->servo_angle_offset[0]);
            float theta2 =   deg_to_rad(arm->current_joint_angles[1] + arm->servo_angle_offset[1]); 

            float l1 = arm->link1_length;
            float l2 = arm->link2_length;

            float theta1_geom = theta1 + arm_link1_theta_bias_rad(arm);

            // 第一连杆几何主方向相对舵机角存在固定偏角，末端位姿按真实几何计算。
            float x_kinematics = l1 * cosf(theta1_geom) + l2 * cosf(theta1_geom + theta2);
            float y_kinematics = l1 * sinf(theta1_geom) + l2 * sinf(theta1_geom + theta2);

            break;
        case ARM_ID_RF:
            // 右前机械臂的正运动学计算
            float theta1 =  deg_to_rad(arm->current_joint_angles[0] + arm->servo_angle_offset[0]);
            float theta2 =  deg_to_rad(arm->current_joint_angles[1] + arm->servo_angle_offset[1]); 
            
            float l1 = arm->link1_length;
            float l2 = arm->link2_length;
            
            float theta1_geom = theta1 + arm_link1_theta_bias_rad(arm);
            
            // 第一连杆几何主方向相对舵机角存在固定偏角，末端位姿按真实几何计算。
            float x_kinematics = l1 * cosf(theta1_geom) + l2 * cosf(theta1_geom + theta2);
            float y_kinematics = l1 * sinf(theta1_geom) + l2 * sinf(theta1_geom + theta2);

            break;
        case ARM_ID_LB:
            // 左后机械臂的正运动学计算
            float theta1 =  -deg_to_rad(arm->current_joint_angles[0] + arm->servo_angle_offset[0]);
            float theta2 =   deg_to_rad(arm->current_joint_angles[1] + arm->servo_angle_offset[1]); 
            
            float l1 = arm->link1_length;
            float l2 = arm->link2_length; 
            
            float theta1_geom = theta1 + arm_link1_theta_bias_rad(arm);
            
            // 第一连杆几何主方向相对舵机角存在固定偏角，末端位姿按真实几何计算。
            float x_kinematics = l1 * cosf(theta1_geom) + l2 * cosf(theta1_geom + theta2);
            float y_kinematics = l1 * sinf(theta1_geom) + l2 * sinf(theta1_geom + theta2);

            break;
        case ARM_ID_RB:
            // 右后机械臂的正运动学计算
            float theta1 =  deg_to_rad(arm->current_joint_angles[0] + arm->servo_angle_offset[0]);
            float theta2 =  deg_to_rad(arm->current_joint_angles[1] + arm->servo_angle_offset[1]); 
            
            float l1 = arm->link1_length;
            float l2 = arm->link2_length;
            
            float theta1_geom = theta1 + arm_link1_theta_bias_rad(arm);
            
            // 第一连杆几何主方向相对舵机角存在固定偏角，末端位姿按真实几何计算。
            float x_kinematics = l1 * cosf(theta1_geom) + l2 * cosf(theta1_geom + theta2);
            float y_kinematics = l1 * sinf(theta1_geom) + l2 * sinf(theta1_geom + theta2);

            break;
        default:
            // 未知机械臂ID，进入错误状态
            arm->state = ARM_STATE_ERROR;
            return;
    }
    //实际角度值映射到运动学计算的关节角度值，考虑舵机安装方向和角度偏移

    arm->end_effector_x = x_kinematics;
    arm->end_effector_y = y_kinematics;
}

//需要修正偏置
bool planar_arm_inverse_kinematics(Planar_Robot_Arm *arm,
                                   bool elbow_up)
{
    if (arm == NULL) {
        return false;
    }

    // 根据elbow_up参数选择肘部方向，计算关节角度
    float x = arm->end_aim_x;
    float y = arm->end_aim_y;
    //external_to_kinematics_xy(arm, arm->end_aim_x, arm->end_aim_y, &x, &y);

    float l1 = arm->link1_length;
    float l2 = arm->link2_length;
    float theta_bias = arm_link1_theta_bias_rad(arm);

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

    float alpha = atan2f(y, x);
    float cos_beta = (l1 * l1 + r2 - l2 * l2) / (2.0f * l1 * sqrtf(r2));
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acosf(cos_beta);
    float theta1_geom = alpha - beta;

    // float beta = acosf(cos_beta);
    // float k1 = l1 + l2 * cos_theta2;
    // float k2 = l2 * sin_theta2;
    // float theta1_geom = atan2f(y, x) - atan2f(k2, k1);
    float theta1 = theta1_geom - theta_bias;

    
    arm->target_joint_angles[0] = theta1 ;
    arm->target_joint_angles[1] = theta2 ;
//处理舵机安装方向和角度偏移

    arm->target_joint_angles[0] = -rad_to_deg(arm->target_joint_angles[0]) - arm->servo_angle_offset[0];
    arm->target_joint_angles[1] =  -rad_to_deg(arm->target_joint_angles[1]) -  arm->servo_angle_offset[1];

    if(arm->target_joint_angles[0] < -180.0f)
     {arm->target_joint_angles[0] += 360.0f;
    }else if (arm->target_joint_angles[0] > 180.0f) {
            arm->target_joint_angles[0] -= 360.0f;
        }
    
    if (arm->target_joint_angles[1] < -180.0f) {
        arm->target_joint_angles[1] += 360.0f;
    }else if(arm->target_joint_angles[1] > 180.0f) {
        arm->target_joint_angles[1] -= 360.0f;
    }
    // arm->target_joint_angles[0] = clampf(arm->target_joint_angles[0], -90.0f, 90.0f);
    // arm->target_joint_angles[1] = clampf(arm->target_joint_angles[1], -90.0f, 90.0f);

    return true;

}

// 单臂控制步骤：反馈更新 -> 逆运动学 -> 发送位置指令
static bool arm_control_step(Planar_Robot_Arm *arm, bool elbow_up ,float x, float y)
{
    if (arm == NULL) {
        return false;
    }

    // 1. 读取舵机反馈，更新当前关节角和末端位姿
    planar_robot_arm_feedback(arm);


    // 2. 逆运动学求解目标关节角
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
    arm->target_servo_positions[1] =  ((arm->target_joint_angles[1]  + 180.0f) / 360.0f * 4095.0f );

    arm->target_servo_positions[0] = (int16_t)clampf(arm->target_servo_positions[0], 1024.0f, 3072.0f);
    arm->target_servo_positions[1] = (int16_t)clampf(arm->target_servo_positions[1], 1024.0f, 3072.0f);
    // 保留转换逻辑，发送接口在此路径暂未启用。
    // (void)arm->target_servo_positions[0];
    // (void)arm->target_servo_positions[1];
    uint8_t  ID[2]       = {arm->SERVO_ID1, arm->SERVO_ID2}; // 舵机ID数组
    int16_t  Position[2] = {arm->target_servo_positions[0], arm->target_servo_positions[1]}; // 目标位置数组
    uint16_t Speed[2]   = {1500, 1500}; // 速度数组
    uint8_t  ACC[2]      = {0, 0}; // 加速度数组

    // WritePosEx(arm->SERVO_ID1, arm->target_servo_positions[0], Speed[0], ACC[0]);
    // osDelay(1);
    // WritePosEx(arm->SERVO_ID2, arm->target_servo_positions[1], Speed[1], ACC[1]);
    // //SyncWritePosEx(ID, 2, Position, Speed, ACC);
    // osDelay(1);

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
    uint16_t Speed[8] = {800, 800, 800, 800, 800, 800, 800, 800}; // 速度数组
    uint8_t ACC[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 加速度数组

    SyncWritePosEx(ID, 8, Position, Speed, ACC);
    return true;
}


static int read_servo_pos_with_retry(uint8_t id)
{
    int pos = -1;
    int retry = 0;

    for (retry = 0; retry < 3; retry++) {
        pos = ReadPos((int)id);
        if (pos >= 0) {
            return pos;
        }

        if (FeedBack((int)id) > 0) {
            pos = ReadPos(-1);
            if (pos >= 0) {
                return pos;
            }
        }
    }

    return -1;
}

bool planar_robot_arm_all_init(void)
{
    SCS_SetUART(&huart1);
    setEnd(0);
    // 配置四个机械臂参数，分别为左前、右前、左后、右后
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_LF, 1, 2, ARM_ID_LF, 0.0f, 0.0f,  -90.0f, 90.0f);
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_RF, 3, 4, ARM_ID_RF, 0.0f, 0.0f, 90.0f,  -90.0f);
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_LB, 5, 6, ARM_ID_LB, 0.0f, 0.0f, 90.0f, -90.0f);
    planar_robot_arm_config_init(ARM_TYPE_1, &Arm_RB, 7, 8, ARM_ID_RB, 0.0f, 0.0f, -90.0f, 90.0f);

    return true;

}

int16_t* planar_arm_control_cmd(int16_t* cmd_array, size_t array_size)
{
    static int16_t cmd_buffer[4] = {0}; // 静态缓冲区存储命令数据

    if (cmd_array == NULL || array_size < 4) {
        return NULL; // 输入无效，返回NULL
    }

    // 从输入数组中提取命令数据，假设前4个元素分别对应四个机械臂的目标位置
    for (size_t i = 0; i < 4; i++) {
        cmd_buffer[i] = cmd_array[i];
    }

    return cmd_buffer; // 返回指向命令数据的指针
}

bool planar_robot_arm_process_cmd(int16_t* cmd_array, size_t array_size)
{
    int16_t* cmd_buffer = planar_arm_control_cmd(cmd_array, array_size);
    if (cmd_buffer == NULL) {
        return false; // 命令处理失败
    }

    // 解析命令数据，更新四个机械臂的目标位置
    Arm_LF.end_aim_x = (float)cmd_buffer[0];
    Arm_RF.end_aim_x = (float)cmd_buffer[1];
    Arm_LB.end_aim_x = (float)cmd_buffer[2];
    Arm_RB.end_aim_x = (float)cmd_buffer[3];

    // 可以添加更多的命令解析逻辑，例如根据命令类型设置不同的目标位置或执行不同的动作

    return true; // 命令处理成功
}

// 四臂控制循环：左前/右前为上肘型，左后/右后为下肘型
bool planar_arm_control_loop(void)
{
    bool ok = true;

    // static bool lf_toggle_inited = false;
    // static uint8_t lf_target_idx = 0U;
    // static uint32_t lf_last_switch_ms = 0U;
    // const uint32_t switch_interval_ms = 3000U;

    // const float lf_target_x[2] = {-46.0f, 450.0f};
    // const float lf_target_y[2] = {0.0f, 300.0f};

    // uint32_t now_ms = HAL_GetTick();
    // if (!lf_toggle_inited) {
    //     lf_toggle_inited = true;
    //     lf_last_switch_ms = now_ms;
    //     lf_target_idx = 0U;
    // } else if ((now_ms - lf_last_switch_ms) >= switch_interval_ms) {
    //     lf_last_switch_ms = now_ms;
    //     lf_target_idx ^= 1U;
    // }

    // // 每个控制周期都执行，持续读取反馈并进行正逆解；仅目标点每2秒切换一次。
    // ok &= arm_control_step(&Arm_LF,
    //                        true,
    //                        lf_target_x[lf_target_idx],
    //                        lf_target_y[lf_target_idx]);
    ok &= arm_control_step(&Arm_LF, false, Arm_LF.end_effector_x, Arm_LF.end_effector_y);
    ok &= arm_control_step(&Arm_RF, false, Arm_RF.end_effector_x, Arm_RF.end_effector_y);
    // 左后、右后：下肘型 (elbow_up = false)
    ok &= arm_control_step(&Arm_LB, false, Arm_LB.end_effector_x, Arm_LB.end_effector_y);
    ok &= arm_control_step(&Arm_RB, true, Arm_RB.end_effector_x, Arm_RB.end_effector_y);

    return ok;
}
