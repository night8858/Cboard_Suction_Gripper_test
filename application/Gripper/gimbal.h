#ifndef GIMBAL_H
#define GIMBAL_H    

#include <stdbool.h>
#include <stdint.h>

typedef enum 
{
    GIMBAL_STATE_IDLE,
    GIMBAL_STATE_MOVING,
    GIMBAL_STATE_ERROR
} GimbalState;


typedef struct {

    uint8_t SERVO_ID1;
    uint8_t SERVO_ID2;
    uint8_t SERVO_ID3;

    GimbalState state;
    
    int16_t current_servo_positions[3];

    float current_J1;       /**< 当前关节1角度，单位：度 */
    float target_J1;        /**< 目标关节1角度，单位：度 */

    float current_YAW;      /**< 当前水平旋转角度，单位：度 */
    float current_PITCH;    /**< 当前垂直旋转角度，单位：度 */

    float target_YAW;       /**< 目标水平旋转角度，单位：度 */
    float target_PITCH;     /**< 目标垂直旋转角度，单位：度 */

} Gimbal_s;

void gimbal_init(void);
void gimbal_start(void);
void gimbal_control_loop(void);
void gimbal_set_target_position(Gimbal_s *Gimbal, float j1, float pitch, float yaw);


#endif // GIMBAL_H
