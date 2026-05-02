#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#include "main.h"
#include "tim.h"

void Motor_SetSpeed(uint8_t motor_id, int pwm_val);
int Motor_ReadSpeed(TIM_HandleTypeDef *htim);

#endif /* MOTOR_DRV_H */
