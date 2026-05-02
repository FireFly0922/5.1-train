#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetSpeed(uint8_t motor_id, int32_t pwm_val);
int32_t Motor_ReadSpeed(uint8_t motor_id);

#endif /* MOTOR_DRV_H */
