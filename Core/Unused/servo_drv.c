#include "servo_drv.h"

#include "app_config.h"
#include "status.h"

void Servo_Init(void) {
    uint32_t index;

    for (index = 0U; index < STATUS_SERVO_COUNT; ++index) {
        STATUS.motor.servo[index].current_angle = 0.0f;
        STATUS.motor.servo[index].target_angle = 0.0f;
        STATUS.motor.servo[index].current_pulse = APP_SERVO_CENTER_PULSE;
        STATUS.motor.servo[index].target_pulse = APP_SERVO_CENTER_PULSE;
    }
}

void Servo_SetPulse(uint8_t servo_id, uint16_t pulse) {
    uint32_t index = (uint32_t)servo_id;

    if (index >= STATUS_SERVO_COUNT) {
        return;
    }

    STATUS.motor.servo[index].target_pulse = pulse;
    STATUS.motor.servo[index].current_pulse = pulse;
}

void Servo_SetAngle(uint8_t servo_id, float angle) {
    uint32_t index = (uint32_t)servo_id;

    if (index >= STATUS_SERVO_COUNT) {
        return;
    }

    STATUS.motor.servo[index].target_angle = angle;
    STATUS.motor.servo[index].current_angle = angle;
}
