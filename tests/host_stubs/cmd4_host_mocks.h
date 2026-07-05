#ifndef CMD4_HOST_MOCKS_H
#define CMD4_HOST_MOCKS_H

#include <stdbool.h>
#include <stdint.h>

#include "Dof4_Arm.h"
#include "action_scheduler_4dof.h"
#include "pc_action_executor_4dof.h"
#include "pneumatic_control.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned single_pick_calls;
    unsigned single_place_calls;
    unsigned single_put_back_calls;
    unsigned single_get_back_calls;
    unsigned dual_pick_calls;
    unsigned dual_place_calls;
    unsigned dual_put_back_calls;
    unsigned dual_get_back_calls;
    unsigned pose_set_calls;
    unsigned action_trigger_calls;
    unsigned action_abort_calls;
    unsigned pc_action_abort_calls;
    unsigned relay_calls;
    unsigned answer_calls;
    unsigned pump_speed_calls;
    unsigned world_offset_calls;
    unsigned arm_start_calls;
    unsigned reject_calls;
    Dof4_ArmId last_arm_id;
    action_state_4dof_e last_action;
    PcAction4DOF_RejectReason last_reject_reason;
    uint8_t last_relay_id;
    uint8_t last_relay_state;
    uint8_t relay_state[4];
    uint8_t last_answer;
    float last_pump_speed;
    float last_offset_x;
    float last_offset_y;
    float last_offset_z;
    Dof4_Pose last_single_target;
    Dof4_Pose last_left_target;
    Dof4_Pose last_right_target;
    Dof4_Pose last_pose_target;
} Cmd4HostMockState;

void cmd4_host_mocks_reset(void);
const Cmd4HostMockState *cmd4_host_mocks_state(void);
void cmd4_host_mocks_set_action_active(bool active);
void cmd4_host_mocks_set_pc_action_active(bool active);

#ifdef __cplusplus
}
#endif

#endif
