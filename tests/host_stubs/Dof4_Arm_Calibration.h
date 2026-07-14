#ifndef HOST_STUB_DOF4_ARM_CALIBRATION_H
#define HOST_STUB_DOF4_ARM_CALIBRATION_H

#include "Dof4_Arm.h"

typedef struct {
    float x;
    float y;
    float z;
} Dof4_CalibrationOffset3;

typedef struct {
    Dof4_CalibrationOffset3 target_bias[2];
} Dof4ArmCalibration;

const Dof4ArmCalibration *Dof4_calibration_get_arm(void);

#endif
