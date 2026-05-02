#ifndef STATUS_H
#define STATUS_H

#include <stdint.h>

#include "main.h"
#include "pid.h"

#define STATUS_WHEEL_COUNT 4U
#define STATUS_SERVO_COUNT 2U

typedef struct {
    PID_t pid;
    float current_speed;
    float target_speed;
    int32_t pwm_duty;
} Wheel_t;

typedef struct {
    float current_angle;
    float target_angle;
} Servo_t;

typedef struct {
    float yaw;
    float pitch;
    float roll;
} Gyro_t;

typedef struct {
    uint8_t is_on;
    uint8_t is_bound;
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_state;
} Led_t;

typedef struct {
    Wheel_t wheel[STATUS_WHEEL_COUNT];
    Servo_t servo[STATUS_SERVO_COUNT];
} Motor_t;

typedef struct {
    Gyro_t gyro;
} Sensor_t;

typedef struct {
    Led_t led;
} Device_t;

typedef struct {
    int32_t main_mode;
    int32_t sub_mode;
} State_t;

typedef struct {
    float demo_setpoint;
    float measurement;
    float control_out;
    uint32_t last_tick;
    uint32_t last_log_tick;
    uint32_t last_hello_tick;
} Runtime_t;

typedef struct {
    Motor_t motor;
    Sensor_t sensor;
    Device_t device;
    State_t state;
    Runtime_t runtime;
} Status_t;

extern Status_t STATUS;

#endif
