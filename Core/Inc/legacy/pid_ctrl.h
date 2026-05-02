#ifndef PID_CTRL_H
#define PID_CTRL_H

#include "app_types.h"

void PID_Reset(PID_TypeDef *pid);
float PID_Calc(PID_TypeDef *pid, float target, float current);

#endif /* PID_CTRL_H */
