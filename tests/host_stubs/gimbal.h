#ifndef HOST_STUB_GIMBAL_H
#define HOST_STUB_GIMBAL_H

typedef struct {
    float j1;
    float pitch;
    float yaw;
} Gimbal_s;

extern Gimbal_s Gimbal;

void gimbal_init(void);
void gimbal_start(void);
void gimbal_set_target_position(Gimbal_s *gimbal,
                                float j1,
                                float pitch,
                                float yaw);

#endif
