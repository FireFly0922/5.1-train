#include "wheel.h"

#include <string.h>

#include "app_config.h"
#include "log.h"
#include "motor_drv.h"
#include "status.h"

enum {
    WHEEL_LEFT_ID = 1U,
    WHEEL_RIGHT_ID = 2U,
    WHEEL_LEFT_INDEX = 0U,
    WHEEL_RIGHT_INDEX = 1U,
};

static void Wheel_InitMotor(void);
static void Wheel_InitState(void);
static void Wheel_InitRuntime(uint32_t now_tick);
static void Wheel_UpdatePID(uint32_t now_tick, float dt);
static float Wheel_UpdateLegacyPid(PID_t *pid, float target, float current, float dt);

void Wheel_Init(void) {
    Wheel_t *left_wheel = &STATUS.motor.wheel[WHEEL_LEFT_INDEX];
    Wheel_t *right_wheel = &STATUS.motor.wheel[WHEEL_RIGHT_INDEX];

    memset(&STATUS, 0, sizeof(STATUS));

    Wheel_InitMotor();
    Wheel_InitState();
    Wheel_InitRuntime(HAL_GetTick());

    STATUS.runtime.demo_setpoint = APP_BASE_SPEED_FAST;

    left_wheel->target_speed = APP_BASE_SPEED_FAST;
    PID_Init(&left_wheel->pid,
             APP_PID_LEFT_KP,
             APP_PID_LEFT_KI,
             APP_PID_LEFT_KD);
    PID_SetIntegralLimit(&left_wheel->pid, APP_PID_INTEGRAL_LIMIT);

    right_wheel->target_speed = APP_BASE_SPEED_FAST;
    PID_Init(&right_wheel->pid,
             APP_PID_RIGHT_KP,
             APP_PID_RIGHT_KI,
             APP_PID_RIGHT_KD);
    PID_SetIntegralLimit(&right_wheel->pid, APP_PID_INTEGRAL_LIMIT);

    PRINTLN("hello world");
    PRINTLN("config: max_pwm=%d deadband=%d feedforward=%d",
            APP_MAX_PWM,
            APP_PWM_DEADBAND,
            APP_PWM_FEEDFORWARD);
    PRINTLN("wheel PID: left %.2f/%.2f/%.2f right %.2f/%.2f/%.2f speed=%.2f",
            left_wheel->pid.kp,
            left_wheel->pid.ki,
            left_wheel->pid.kd,
            right_wheel->pid.kp,
            right_wheel->pid.ki,
            right_wheel->pid.kd,
            left_wheel->target_speed);
}

void Wheel_Update(void) {
    uint32_t now_tick = HAL_GetTick();
    float dt = (now_tick - STATUS.runtime.last_tick) / 1000.0f;

    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    STATUS.runtime.last_tick = now_tick;

    Wheel_UpdatePID(now_tick, dt);

    HAL_Delay(APP_MAIN_LOOP_DELAY_MS);
}

static void Wheel_InitMotor(void) {
    Motor_Init();
}

static void Wheel_InitState(void) {
    STATUS.state.main_mode = 0;
    STATUS.state.sub_mode = 0;
    STATUS.state.car_run_state = 0;
}

static void Wheel_InitRuntime(uint32_t now_tick) {
    STATUS.runtime.demo_setpoint = 0.0f;
    STATUS.runtime.measurement = 0.0f;
    STATUS.runtime.control_out = 0.0f;
    STATUS.runtime.total_pulses = 0;
    STATUS.runtime.last_tick = now_tick;
    STATUS.runtime.last_log_tick = now_tick;
    STATUS.runtime.last_hello_tick = now_tick;
}

// Drive both wheels with the same speed loop parameters used by legacy.
static void Wheel_UpdatePID(uint32_t now_tick, float dt) {
    Wheel_t *left_wheel = &STATUS.motor.wheel[WHEEL_LEFT_INDEX];
    Wheel_t *right_wheel = &STATUS.motor.wheel[WHEEL_RIGHT_INDEX];
    float left_pwm;
    float right_pwm;
    int32_t speed_left;
    int32_t speed_right;

    (void)dt;

    left_wheel->target_speed = APP_BASE_SPEED_FAST;
    right_wheel->target_speed = APP_BASE_SPEED_FAST;

    speed_left = Motor_ReadSpeed(WHEEL_LEFT_ID);
    speed_right = Motor_ReadSpeed(WHEEL_RIGHT_ID);
    STATUS.runtime.total_pulses += (speed_left + speed_right) / 2;

    left_pwm = Wheel_UpdateLegacyPid(&left_wheel->pid,
                                     left_wheel->target_speed,
                                     (float)speed_left,
                                     dt);
    right_pwm = Wheel_UpdateLegacyPid(&right_wheel->pid,
                                      right_wheel->target_speed,
                                      (float)speed_right,
                                      dt);

    Motor_SetSpeed(WHEEL_LEFT_ID, (int32_t)left_pwm);
    Motor_SetSpeed(WHEEL_RIGHT_ID, (int32_t)right_pwm);

    STATUS.runtime.demo_setpoint = left_wheel->target_speed;
    STATUS.runtime.measurement = (left_wheel->current_speed + right_wheel->current_speed) * 0.5f;
    STATUS.runtime.control_out = (left_pwm + right_pwm) * 0.5f;

    if ((now_tick - STATUS.runtime.last_log_tick) >= APP_LOG_PERIOD_MS) {
        PRINTLN("L sp=%.2f speed=%ld pwm=%ld | R sp=%.2f speed=%ld pwm=%ld dt=%.3f",
                left_wheel->target_speed,
                (long)speed_left,
                (long)left_wheel->pwm_duty,
                right_wheel->target_speed,
                (long)speed_right,
                (long)right_wheel->pwm_duty,
                dt);
        STATUS.runtime.last_log_tick = now_tick;
    }

    if ((now_tick - STATUS.runtime.last_hello_tick) >= APP_HELLO_PERIOD_MS) {
        PRINTLN("hello world");
        STATUS.runtime.last_hello_tick = now_tick;
    }
}

static float Wheel_UpdateLegacyPid(PID_t *pid, float target, float current, float dt) {
    float output;

    output = PID_Update(pid, target - current, dt);

    return output;
}
