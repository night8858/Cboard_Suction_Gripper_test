#ifndef HOST_STUB_ACTION_SCHEDULER_4DOF_H
#define HOST_STUB_ACTION_SCHEDULER_4DOF_H

#include <stdbool.h>

#include "Dof4_Arm.h"

typedef enum {
    ACTION_4DOF_IDLE = 0,
    ACTION_DANCE = 13,
} action_state_4dof_e;

bool action_4dof_is_active(void);
bool action_4dof_trigger(action_state_4dof_e action);
void action_4dof_abort(void);
Dof4_Pose action_4dof_get_idle_pose(Dof4_ArmId arm_id);
void action_4dof_set_back_occupied(Dof4_ArmId arm_id, bool occupied);

#endif
