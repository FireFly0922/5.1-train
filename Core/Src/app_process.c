#include "app_process.h"

#include "log.h"
#include "status.h"

enum {
    APP_DEMO_WHEEL_INDEX = 0,
    APP_LOG_PERIOD_MS = 200U,
    APP_HELLO_PERIOD_MS = 1000U,
    APP_LOOP_DELAY_MS = 10U,
};

static void App_ProcessControl(uint32_t now_tick, float dt);
static void App_ProcessDevice(void);

void App_Process(void) {
    uint32_t now_tick = HAL_GetTick();
    float dt = (now_tick - STATUS.runtime.last_tick) / 1000.0f;

    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    STATUS.runtime.last_tick = now_tick;

    App_ProcessControl(now_tick, dt);
    App_ProcessDevice();

    HAL_Delay(APP_LOOP_DELAY_MS);
}

static void App_ProcessControl(uint32_t now_tick, float dt) {
    Wheel_t *demo_wheel = &STATUS.motor.wheel[APP_DEMO_WHEEL_INDEX];
    float error = demo_wheel->target_speed - STATUS.runtime.measurement;

    STATUS.runtime.demo_setpoint = demo_wheel->target_speed;
    STATUS.runtime.control_out = PID_Update(&demo_wheel->pid, error, dt);
    STATUS.runtime.measurement +=
        (STATUS.runtime.control_out - STATUS.runtime.measurement) * 0.1f;

    demo_wheel->current_speed = STATUS.runtime.measurement;
    demo_wheel->pwm_duty = (int32_t)STATUS.runtime.control_out;

    if ((now_tick - STATUS.runtime.last_log_tick) >= APP_LOG_PERIOD_MS) {
        PRINTLN("sp=%.2f meas=%.2f err=%.2f out=%.2f dt=%.3f",
                demo_wheel->target_speed,
                demo_wheel->current_speed,
                error,
                STATUS.runtime.control_out,
                dt);
        STATUS.runtime.last_log_tick = now_tick;
    }

    if ((now_tick - STATUS.runtime.last_hello_tick) >= APP_HELLO_PERIOD_MS) {
        PRINTLN("hello world");
        STATUS.runtime.last_hello_tick = now_tick;
    }
}

static void App_ProcessDevice(void) {
    Led_t *led = &STATUS.device.led;
    GPIO_PinState inactive_state = GPIO_PIN_RESET;

    if ((led->is_bound == 0U) || (led->port == NULL)) {
        return;
    }

    if (led->active_state == GPIO_PIN_RESET) {
        inactive_state = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(led->port,
                      led->pin,
                      (led->is_on != 0U) ? led->active_state : inactive_state);
}
