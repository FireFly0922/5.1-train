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

    uint8_t is_first;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
float PID_Update(PID_t *pid, float error, float dt);

#endif
