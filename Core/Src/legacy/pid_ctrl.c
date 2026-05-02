#include "pid_ctrl.h"
#include "app_config.h"

/* 娓呯┖ PID 鐨勫姩鎬侀」锛屼繚鐣欏弬鏁伴」銆?*/
void PID_Reset(PID_TypeDef *pid)
{
    if (pid == 0) {
        return;
    }

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float PID_Calc(PID_TypeDef *pid, float target, float current)
{
    float derivative;

    if (pid == 0) {
        return 0.0f;
    }

    pid->error = target - current;
    pid->integral += pid->error;

    if (pid->integral > APP_PID_INTEGRAL_LIMIT) {
        pid->integral = APP_PID_INTEGRAL_LIMIT;
    } else if (pid->integral < -APP_PID_INTEGRAL_LIMIT) {
        pid->integral = -APP_PID_INTEGRAL_LIMIT;
    }

    derivative = pid->error - pid->last_error;
    pid->output = (pid->Kp * pid->error)
                + (pid->Ki * pid->integral)
                + (pid->Kd * derivative);
    pid->last_error = pid->error;

    return pid->output;
}
