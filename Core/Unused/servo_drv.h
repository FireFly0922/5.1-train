#ifndef SERVO_DRV_H
#define SERVO_DRV_H

#include <stdint.h>

void Servo_Init(void);
void Servo_SetPulse(uint8_t servo_id, uint16_t pulse);
void Servo_SetAngle(uint8_t servo_id, float angle);

#endif /* SERVO_DRV_H */
