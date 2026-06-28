#include "command_decode_4dof.h"
#include "pc_action_executor_4dof.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void require_true(bool value, const char *message)
{
    if (!value) {
        printf("FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    require_true(CMD4_PICK_BLOCK == 0x11U, "single pick command");
    require_true(CMD4_PLACE_BLOCK == 0x12U, "single place command");
    require_true(CMD4_PUT_BLOCK_BACK == 0x14U, "single put-back command");
    require_true(CMD4_GET_BLOCK_BACK == 0x15U, "single get-back command");
    require_true(CMD4_PICK_BLOCK_ALL == 0x21U, "dual pick command");
    require_true(CMD4_PUT_BLOCK_BACK_ALL == 0x22U, "dual put-back command");
    require_true(CMD4_PLACE_BLOCK_ALL == 0x23U, "dual place command");
    require_true(CMD4_GET_BLOCK_BACK_ALL == 0x24U, "dual get-back command");

    require_true(CMD4_FRAME_SINGLE_TARGET_ACTION_LEN == 18U,
                 "single target frame length");
    require_true(CMD4_FRAME_SINGLE_BACK_ACTION_LEN == 6U,
                 "single back frame length");
    require_true(CMD4_FRAME_DUAL_TARGET_ACTION_LEN == 29U,
                 "dual target frame length");
    require_true(CMD4_FRAME_DUAL_BACK_ACTION_LEN == 5U,
                 "dual back frame length");
    require_true(fabsf(PC_ACTION_4DOF_VERTICAL_CLEARANCE_M - 0.05f) < 1.0e-6f,
                 "dynamic vertical clearance");
    printf("pc action protocol test passed\n");
    return 0;
}
