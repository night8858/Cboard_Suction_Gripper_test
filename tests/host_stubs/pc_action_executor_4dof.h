#ifndef HOST_STUB_PC_ACTION_EXECUTOR_4DOF_H
#define HOST_STUB_PC_ACTION_EXECUTOR_4DOF_H

#include <stdbool.h>
#include "Dof4_Arm.h"

#define PC_ACTION_4DOF_VERTICAL_CLEARANCE_M 0.05f

typedef enum {
    PC_ACTION_4DOF_REJECT_NONE = 0,
    PC_ACTION_4DOF_REJECT_INVALID_ARM = 1,
    PC_ACTION_4DOF_REJECT_BAD_TARGET = 2,
    PC_ACTION_4DOF_REJECT_TARGET_UNREACHABLE = 3,
    PC_ACTION_4DOF_REJECT_TARGET_ABOVE_UNREACHABLE = 4,
    PC_ACTION_4DOF_REJECT_JOINT_PATH_INVALID = 5,
    PC_ACTION_4DOF_REJECT_BUSY = 6,
    PC_ACTION_4DOF_REJECT_COMPLETION_PENDING = 7,
    PC_ACTION_4DOF_REJECT_ACTION_ACTIVE = 8,
    PC_ACTION_4DOF_REJECT_PENDING_FULL = 9,
    PC_ACTION_4DOF_REJECT_PATH_POINT_UNREACHABLE = 10,
} PcAction4DOF_RejectReason;

bool pc_action_4dof_start_pick(Dof4_ArmId arm_id, const Dof4_Pose *target_world);
bool pc_action_4dof_start_place(Dof4_ArmId arm_id, const Dof4_Pose *target_world);
bool pc_action_4dof_start_put_back(Dof4_ArmId arm_id);
bool pc_action_4dof_start_get_back(Dof4_ArmId arm_id);
bool pc_action_4dof_start_dual_pick(const Dof4_Pose *left_target_world,
                                    const Dof4_Pose *right_target_world);
bool pc_action_4dof_start_dual_place(const Dof4_Pose *left_target_world,
                                     const Dof4_Pose *right_target_world);
bool pc_action_4dof_start_dual_put_back(void);
bool pc_action_4dof_start_dual_get_back(void);
void pc_action_4dof_init(void);
void pc_action_4dof_loop(void);
bool pc_action_4dof_is_active(void);
void pc_action_4dof_record_reject(Dof4_ArmId arm_id,
                                  PcAction4DOF_RejectReason reason,
                                  const Dof4_Pose *target_world);
bool pc_action_4dof_completion_pending(void);
void pc_action_4dof_completion_acknowledge(void);

#endif
