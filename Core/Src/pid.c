#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd) {
    if (!pid) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->p = 0.0f;
    pid->i = 0.0f;
    pid->d = 0.0f;

    pid->error = 0.0f;
    pid->last_error = 0.0f;

    pid->is_first = 1u;
}

float PID_Update(PID_t *pid, float error, float dt) {
    if (!pid || dt <= 0.0f) {
        return 0.0f;
    }

    pid->error = error;

    pid->p = pid->error;
    pid->i += pid->error * dt;

    if (pid->is_first) {
        pid->d = 0.0f;
        pid->is_first = 0u;
    } else {
        pid->d = (pid->error - pid->last_error) / dt;
    }

    pid->last_error = pid->error;

    return (pid->kp * pid->p) + (pid->ki * pid->i) + (pid->kd * pid->d);
}
