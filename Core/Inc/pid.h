#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float p;
    float i;
    float d;

    float error;
    float last_error;
    float integral_limit;

    uint8_t is_first;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
void PID_Reset(PID_t *pid);
void PID_SetIntegralLimit(PID_t *pid, float limit);
float PID_Update(PID_t *pid, float error, float dt);

#endif
