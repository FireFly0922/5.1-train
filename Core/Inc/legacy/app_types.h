#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include "app_config.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error;
    float last_error;
    float integral;
    float output;
} PID_TypeDef;

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_RUN_1 = 1,
    APP_STATE_STOP_1 = 2,
    APP_STATE_TURN_1 = 3,
    APP_STATE_RUN_2 = 4,
    APP_STATE_STOP_2 = 5,
    APP_STATE_TURN_2 = 6,
    APP_STATE_RUN_3 = 7,
    APP_STATE_FINISH = 8
} AppRunState;

typedef struct {
    uint8_t rx1_buffer[APP_RX_BUF_SIZE];
    uint8_t rx3_buffer[APP_RX_BUF_SIZE];

    float current_yaw;
    float target_yaw;
    float vision_target;

    volatile float base_speed;
    volatile long long total_pulses;
    volatile AppRunState run_state;

    PID_TypeDef pid_left;
    PID_TypeDef pid_right;
    PID_TypeDef pid_angle;
} AppControlContext;

#endif /* APP_TYPES_H */
