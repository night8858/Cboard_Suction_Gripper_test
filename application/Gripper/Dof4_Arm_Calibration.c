/**
 * @file Dof4_Arm_Calibration.c
 * @brief 4DOF 机械臂现场调参集中配置。
 *
 * 实机调参优先修改本文件。单位约定：
 * - 位置、长度、偏置：m
 * - 角度：rad
 * - 舵机位置：0..4095 position
 * - 时间：ms
 */

#include "Dof4_Arm_Calibration.h"

/* ── 便捷宏：构造关节角度点（J1,J2,J3,J4）── */
#define CALIB_JOINT_POINT(j1_, j2_, j3_, j4_) \
    { .q = {(j1_), (j2_), (j3_), (j4_)} }

/* ── 便捷宏：四关节统一使用最小/最大舵机位置 ── */
#define CALIB_SERVO_ALL_MIN \
    { DOF4_SERVO_MIN_POS, DOF4_SERVO_MIN_POS, DOF4_SERVO_MIN_POS, DOF4_SERVO_MIN_POS }

#define CALIB_SERVO_ALL_MAX \
    { DOF4_SERVO_MAX_POS, DOF4_SERVO_MAX_POS, DOF4_SERVO_MAX_POS, DOF4_SERVO_MAX_POS }

/* PC 动态取放目标正上方悬停高度。现场调参只需修改此值，单位 m。 */
#define CALIB_PC_DYNAMIC_HOVER_CLEARANCE_M 0.12f

/* ========================================================================
 *  机械臂本体标定参数（左臂 & 右臂）
 *  包含：舵机ID、几何尺寸、关节限位、舵机映射、工作空间等
 * ======================================================================== */

static const Dof4ArmCalibration s_arm_calibration = {
    .arm = {
        /* ──────────── 左臂（DOF4_ARM_LEFT）──────────── */
        [DOF4_ARM_LEFT] = {
            .servo_id = {1U, 2U, 3U, 4U},          /* 舵机总线 ID */
            .base = {0.0f, 0.0f, 0.0f},             /* 基座世界坐标 */
            .base_offset = {0.0f, 0.0f, 0.0f},      /* 基座微调偏置 */
            .shoulder_r = 0.02716f,                  /* 肩关节水平偏移半径 */
            .shoulder_z = 0.022319f,                 /* 肩关节高度 */
            .link_len = {0.35548f, 0.27130f, 0.03600f}, /* 连杆长度 L1/L2/L3 */
            .tcp_offset = {0.0f, 0.0f, -0.0f},      /* 工具中心点偏置 */
            .pitch_offset = 0.0f,                     /* 末端俯仰角偏置 */
            .joint_min = {-3.90f, 2.67f - 3.25f, -1.57f - 4.60f, -0.00f - 2.35f}, /* 关节下限 */
            .joint_max = {1.90f, 2.67f + 2.25f, 0.50f, -0.00f + 2.35f},           /* 关节上限 */
            .servo_min = CALIB_SERVO_ALL_MIN,        /* 舵机位置下限 */
            .servo_max = CALIB_SERVO_ALL_MAX,        /* 舵机位置上限 */
            .servo_zero = {                          /* 舵机零点（中位） */
                DOF4_SERVO_CENTER_POS,
                DOF4_SERVO_CENTER_POS,
                DOF4_SERVO_CENTER_POS,
                DOF4_SERVO_CENTER_POS,
            },
            .servo_offset = {1.57f, 1.57f, -1.57f, -0.00f}, /* 舵机零位对应的关节角 */
            .servo_sign = {-1, 1, -1, -1},           /* 舵机转向符号（+1同向/-1反向） */
            .servo_reverse = {0U, 0U, 0U, 0U},       /* 是否反转舵机输出 */
            .servo_speed = 8000U,                     /* 舵机默认速度 */
            .servo_acc = 40U,                         /* 舵机默认加速度 */
            .ws_min = {-0.35f, -0.65f, -0.65f},       /* 工作空间下限 (x,y,z) */
            .ws_max = {0.90f, 0.90f, 0.65f},          /* 工作空间上限 (x,y,z) */
            .cart_vel_mps = DOF4_DEFAULT_CART_VEL_MPS,   /* 默认笛卡尔线速度 */
            .pitch_vel_rps = DOF4_DEFAULT_PITCH_VEL_RPS, /* 默认俯仰角速度 */
        },
        /* ──────────── 右臂（DOF4_ARM_RIGHT）──────────── */
        [DOF4_ARM_RIGHT] = {
            .servo_id = {5U, 6U, 7U, 8U},          /* 舵机总线 ID */
            .base = {0.0f, 0.0f, 0.00f},            /* 基座世界坐标 */
            .base_offset = {0.0f, 0.0f, 0.0f},      /* 基座微调偏置 */
            .shoulder_r = 0.02716f,                  /* 肩关节水平偏移半径 */
            .shoulder_z = 0.022319f,                 /* 肩关节高度 */
            .link_len = {0.35548f, 0.27130f, 0.03600f}, /* 连杆长度 L1/L2/L3 */
            .tcp_offset = {0.0f, 0.0f, -0.0f},      /* 工具中心点偏置 */
            .pitch_offset = 0.0f,                     /* 末端俯仰角偏置 */
            .joint_min = {-1.90f, 2.67f - 3.25f, -1.57f - 4.60f, 0.0f - 2.35f}, /* 关节下限 */
            .joint_max = {3.90f, 2.67f + 2.25f, 0.50f, 0.0f + 2.35f},           /* 关节上限 */
            .servo_min = CALIB_SERVO_ALL_MIN,        /* 舵机位置下限 */
            .servo_max = CALIB_SERVO_ALL_MAX,        /* 舵机位置上限 */
            .servo_zero = {                          /* 舵机零点（中位） */
                DOF4_SERVO_CENTER_POS,
                DOF4_SERVO_CENTER_POS,
                DOF4_SERVO_CENTER_POS,
                DOF4_SERVO_CENTER_POS,
            },
            .servo_offset = {-1.57f, 1.57f, -1.57f, 0.0f}, /* 舵机零位对应的关节角 */
            .servo_sign = {-1, 1, -1, 1},            /* 舵机转向符号（+1同向/-1反向） */
            .servo_reverse = {0U, 0U, 0U, 0U},       /* 是否反转舵机输出 */
            .servo_speed = 8000U,                     /* 舵机默认速度 */
            .servo_acc = 40U,                         /* 舵机默认加速度 */
            .ws_min = {-0.35f, -0.90f, -0.65f},       /* 工作空间下限 (x,y,z) */
            .ws_max = {0.90f, 0.65f, 0.65f},          /* 工作空间上限 (x,y,z) */
            .cart_vel_mps = DOF4_DEFAULT_CART_VEL_MPS,   /* 默认笛卡尔线速度 */
            .pitch_vel_rps = DOF4_DEFAULT_PITCH_VEL_RPS, /* 默认俯仰角速度 */
        },
    },
    .world_offset = {0.0f, 0.0f, 0.0f},              /* 世界坐标系整体偏移 */
    .target_bias = {                                 
         /* 目标点偏置（左右臂独立） */
        [DOF4_ARM_LEFT]  = {0.0f, 0.0f, 0.0f},
        [DOF4_ARM_RIGHT] = {0.0f, 0.0f, 0.0f},
    },
};

/* ========================================================================
 *  PC 端动作标定参数
 *  定义抓取(pick)、放置(place)、归位(put_back/get_back)等动作的
 *  进入/退出偏移、目标俯仰角、关节路径点及超时/容差/延迟等
 * ======================================================================== */

static const Dof4PcActionCalibration s_pc_action_calibration = {
    /* 动态模板的 exit_offset 当前仅启用 y：左臂 +Y、右臂 -Y；其余分量保留。 */
    /* ── 单臂抓取 (pick)：entry=接近目标, exit=离开目标, target_pitch=抓取俯仰角 ── */
    .pick = {
        [DOF4_ARM_LEFT] = {
            .entry_offset = {-0.10f, -0.15f, 0.38f, -0.60f},
            .exit_offset = {-0.10f, 0.05f, 0.38f, -0.30f},
            .target_pitch = -1.57f,
            .vertical_clearance_m = CALIB_PC_DYNAMIC_HOVER_CLEARANCE_M,
        },
        [DOF4_ARM_RIGHT] = {
            .entry_offset = {0.10f, 0.15f, 0.40f, -0.60f},
            .exit_offset = {0.10f, -0.05f, 0.40f, -0.30f},
            .target_pitch = -1.57f,
            .vertical_clearance_m = CALIB_PC_DYNAMIC_HOVER_CLEARANCE_M,
        },
    },
    /* ── 双臂协同抓取 (dual_pick) ── */
    .dual_pick = {
        [DOF4_ARM_LEFT] = {
            .entry_offset = {0.10f, -0.15f, 0.28f, -0.60f},
            .exit_offset = {0.10f, 0.05f, 0.365f, -0.30f},
            .target_pitch = -1.57f,
            .vertical_clearance_m = CALIB_PC_DYNAMIC_HOVER_CLEARANCE_M,
        },
        [DOF4_ARM_RIGHT] = {
            .entry_offset = {0.10f, 0.15f, 0.30f, -0.60f},
            .exit_offset = {0.10f, -0.05f, 0.385f, -0.30f},
            .target_pitch = -1.57f,
            .vertical_clearance_m = CALIB_PC_DYNAMIC_HOVER_CLEARANCE_M,
        },
    },
    /* ── 单臂放置 (place) ── */
    .place = {
        [DOF4_ARM_LEFT] = {
            .entry_offset = {-0.075f, -0.10f, 0.49f, -0.30f},
            .exit_offset = {-0.175f, 0.05f, 0.44f, -0.50f},
            .target_pitch = -1.57f,
            .vertical_clearance_m = CALIB_PC_DYNAMIC_HOVER_CLEARANCE_M,
        },
        [DOF4_ARM_RIGHT] = {
            .entry_offset = {-0.075f, 0.10f, 0.52f, -1.00f},
            .exit_offset = {-0.175f, -0.05f, 0.43f, -0.50f},
            .target_pitch = -1.57f,
            .vertical_clearance_m = CALIB_PC_DYNAMIC_HOVER_CLEARANCE_M,
        },
    },
    /* ── 单臂放回 (put_back)：pre=放回前路径点, post=放回后路径点 ── */
    .put_back = {
        [DOF4_ARM_LEFT] = {
            .pre = {
                CALIB_JOINT_POINT(2.500f, 1.50f, -1.20f, -1.680f),
                CALIB_JOINT_POINT(2.760f, 1.57f, -1.82f, -1.30f),
            },
            .post = {
                CALIB_JOINT_POINT(2.000f, 1.50f, -1.20f, -1.463f),
                CALIB_JOINT_POINT(1.0f, 1.50f, -1.70f, -1.463f),
            },
            .pre_count = 2U,
            .post_count = 2U,
        },
        [DOF4_ARM_RIGHT] = {
            .pre = {
                CALIB_JOINT_POINT(-2.500f, 1.50f, -1.20f, -1.680f),
                CALIB_JOINT_POINT(-2.760f, 1.57f, -1.82f, -1.30f),
            },
            .post = {
                CALIB_JOINT_POINT(-2.000f, 1.50f, -1.20f, -1.463f),
                CALIB_JOINT_POINT(-1.0f, 1.50f, -1.70f, -1.463f),
            },
            .pre_count = 2U,
            .post_count = 2U,
        },
    },
    /* ── 双臂协同放回 (dual_put_back) ── */
    .dual_put_back = {
        [DOF4_ARM_LEFT] = {
            .pre = {
                CALIB_JOINT_POINT(2.500f, 1.50f, -1.20f, -1.680f),
                CALIB_JOINT_POINT(2.760f, 1.57f, -1.82f, -1.30f),
            },
            .post = {
                CALIB_JOINT_POINT(2.500f, 1.50f, -1.20f, -1.463f),
                CALIB_JOINT_POINT(1.0f, 1.50f, -1.70f, -1.463f),
            },
            .pre_count = 2U,
            .post_count = 2U,
        },
        [DOF4_ARM_RIGHT] = {
            .pre = {
                CALIB_JOINT_POINT(-2.500f, 1.50f, -1.20f, -1.680f),
                CALIB_JOINT_POINT(-2.760f, 1.57f, -1.82f, -1.30f),
            },
            .post = {
                CALIB_JOINT_POINT(-2.500f, 1.50f, -1.20f, -1.463f),
                CALIB_JOINT_POINT(-1.0f, 1.50f, -1.70f, -1.463f),
            },
            .pre_count = 2U,
            .post_count = 2U,
        },
    },
    /* ── 取回 (get_back)：从回收区取回物块 ── */
    .get_back = {
        [DOF4_ARM_LEFT] = {
            .pre = {
                CALIB_JOINT_POINT(2.500f, 1.50f, -1.00f, -1.680f),
                CALIB_JOINT_POINT(2.760f, 1.57f, -1.82f, -1.30f),
            },
            .post = {
                CALIB_JOINT_POINT(2.500f, 1.50f, -1.48f, -1.463f),
                CALIB_JOINT_POINT(1.042f, 1.50f, -1.70f, -1.463f),
            },
            .pre_count = 2U,
            .post_count = 2U,
        },
        [DOF4_ARM_RIGHT] = {
            .pre = {
                CALIB_JOINT_POINT(-2.500f, 1.50f, -1.00f, -1.680f),
                CALIB_JOINT_POINT(-2.760f, 1.57f, -1.82f, -1.30f),
            },
            .post = {
                CALIB_JOINT_POINT(-2.500f, 1.50f, -1.48f, -1.463f),
                CALIB_JOINT_POINT(-1.042f, 1.50f, -1.70f, -1.463f),
            },
            .pre_count = 2U,
            .post_count = 2U,
        },
    },
    /* ── 通用超时与容差 ── */
    .move_timeout_ms = 2000U,                         /* 单次移动超时 */
    .pose_pos_tol_m = 0.03f,                          /* 位姿位置容差 */
    .pose_pitch_tol_rad = 0.02f,                      /* 位姿俯仰容差 */
    .path_point_max_adjust_m = 0.10f,                 /* 动态路径中间点最大邻近可达修正量 */
    .joint_tol_rad = 0.06f,                           /* 关节角度容差 */
    .back_servo_speed = 8000U,                        /* 兼容旧字段：背部动作高速段舵机速度 */
    .back_fast_servo_speed = 8000U,                   /* 背部动作非交接段高速速度 */
    .back_final_servo_speed = 2800U,                  /* 背部交接点低速精定位速度 */
    .back_fast_servo_acc = 40U,                       /* 背部动作非交接段加速度 */
    .back_final_servo_acc = 14U,                      /* 背部交接点低速精定位加速度 */
    .back_final_joint_tol_rad = 0.04f,                /* 背部交接点严格关节容差 */
    .back_final_stable_frames = 6U,                   /* 背部交接点连续稳定帧数，200Hz 下约 30ms */
    .dynamic_hold_pre_index = 0U,                     /* 动态抓取预保持索引 */
    /* ── 各阶段延迟 (ms) ── */
    .delay_dynamic_pick_hold_ms = 1500U,              /* 动态抓取后保持 */
    .delay_dynamic_place_release_ms = 1600U,          /* 动态放置后释放 */
    .delay_dynamic_target_settle_ms = 800U,           /* 动态目标稳定等待 */
    .delay_back_pre_release_ms = 1600U,               /* 归位前释放 */
    .delay_back_post_release_ms = 1600U,              /* 归位后释放 */
    .delay_back_get_arm_hold_ms = 1600U,              /* 取回后机械臂保持 */
    .delay_back_source_release_ms = 800U,             /* 源端释放 */
    .delay_idle_hold_ms = 100U,                       /* 空闲保持 */
    .delay_dynamic_hover_hold_ms = 60U,               /* 悬停保持 */
};

/* ========================================================================
 *  动作执行标定参数
 *  定义闲置位姿、避让位姿、各阶段超时/保持时间、轨迹混合参数等
 * ======================================================================== */
static const Dof4ActionCalibration s_action_calibration = {
    .idle_offset = {                                  /* 闲置姿态偏移 */
        [DOF4_ARM_LEFT]  = {0.15f, 0.10f, 0.25f, -0.02f},
        [DOF4_ARM_RIGHT] = {0.15f, -0.10f, 0.25f, -0.02f},
    },
    .default_back_avoid = {                           /* 默认归位避让点 */
        [DOF4_ARM_LEFT]  = {0.22f,  0.06f,  0.34f, 0.45f},
        [DOF4_ARM_RIGHT] = {0.22f, -0.06f, 0.34f, 0.45f},
    },
    .current_back_avoid = {                           /* 当前归位避让点 */
        [DOF4_ARM_LEFT] = {0.10f, 0.15f, 0.28f, 0.45f},
        [DOF4_ARM_RIGHT] = {0.10f, -0.15f, 0.28f, 0.45f},
    },
    /* ── 超时与保持时间 (ms) ── */
    .move_timeout_ms = 2500U,                         /* 移动超时 */
    .suction_timeout_ms = 1500U,                      /* 吸盘吸附超时 */
    .place_hold_ms = 1000U,                           /* 放置后保持 */
    .back_release_hold_ms = 500U,                     /* 归位释放保持 */
    .place_pre_release_back_ms = 2000U,               /* 放置前释放回退 */
    .place_post_release_back_ms = 2000U,              /* 放置后释放回退 */
    .place_post_release_ext_ms = 2000U,               /* 放置后释放伸出 */
    .release_timeout_ms = 1000U,                      /* 释放超时 */
    .hold_ms = 100U,                                  /* 通用保持时间 */
    .waypoint_hold_ms = 100U,                         /* 路径点停留 */
    /* ── 轨迹混合参数 ── */
    .default_blend_dist_m = 0.08f,                    /* 默认混合距离 */
    .default_via_speed_factor = 0.75f,                /* 默认途经点速度系数 */
    .chain_final_blend_dist_m = 0.03f,                /* 链末段混合距离 */
    /* ── 到位判定容差 ── */
    .reach_pos_tol_m = 0.03f,                         /* 位置到位容差 */
    .reach_pitch_tol_rad = 0.03f,                     /* 俯仰到位容差 */
    .joint_reach_tol_rad = 0.05f,                     /* 关节到位容差 */
    .joint_blend_tol_rad = 0.15f,                     /* 关节混合容差 */
};

/* ───────────────────────────────────────────────────────────────────────
 *  获取函数：返回各标定结构体的只读指针
 * ─────────────────────────────────────────────────────────────────────── */

/** 获取机械臂本体标定参数 */
const Dof4ArmCalibration *Dof4_calibration_get_arm(void)
{
    return &s_arm_calibration;
}

/** 获取 PC 端动作标定参数 */
const Dof4PcActionCalibration *Dof4_calibration_get_pc_action(void)
{
    return &s_pc_action_calibration;
}

/** 获取动作执行标定参数 */
const Dof4ActionCalibration *Dof4_calibration_get_action(void)
{
    return &s_action_calibration;
}

/**
 * @brief 将标定中的运行时偏置应用到左右臂实例
 * @param left  左臂指针（可为 NULL）
 * @param right 右臂指针（可为 NULL）
 *
 * 将 base_offset、tcp_offset、pitch_offset 和 world_offset
 * 写入对应的 Dof4_Arm 实例，使运动学计算使用最新标定值。
 */

void Dof4_calibration_apply_runtime_offsets(Dof4_Arm *left, Dof4_Arm *right)
{
    const Dof4ArmCalibration *cal = Dof4_calibration_get_arm();

    /* 应用左臂运行时偏置 */
    if (left != 0) {
        const Dof4_SingleArmCalibration *left_cal = &cal->arm[DOF4_ARM_LEFT];
        (void)Dof4_arm_set_base_offset(left,
                                       left_cal->base_offset[0],
                                       left_cal->base_offset[1],
                                       left_cal->base_offset[2]);
        (void)Dof4_arm_set_tcp_offset(left,
                                      left_cal->tcp_offset[0],
                                      left_cal->tcp_offset[1],
                                      left_cal->tcp_offset[2]);
        (void)Dof4_arm_set_pitch_offset(left, left_cal->pitch_offset);
    }

    /* 应用右臂运行时偏置 */
    if (right != 0) {
        const Dof4_SingleArmCalibration *right_cal = &cal->arm[DOF4_ARM_RIGHT];
        (void)Dof4_arm_set_base_offset(right,
                                       right_cal->base_offset[0],
                                       right_cal->base_offset[1],
                                       right_cal->base_offset[2]);
        (void)Dof4_arm_set_tcp_offset(right,
                                      right_cal->tcp_offset[0],
                                      right_cal->tcp_offset[1],
                                      right_cal->tcp_offset[2]);
        (void)Dof4_arm_set_pitch_offset(right, right_cal->pitch_offset);
    }

    /* 应用世界坐标系整体偏移 */
    (void)Dof4_set_world_offset(cal->world_offset.x,
                                cal->world_offset.y,
                                cal->world_offset.z);
}
