#include "app_init.h"

#include <string.h>

#include "log.h"
#include "status.h"

enum {
    APP_DEMO_WHEEL_INDEX = 0,
};

static void App_InitMotor(void);
static void App_InitSensor(void);
static void App_InitDevice(void);
static void App_InitState(void);
static void App_InitRuntime(uint32_t now_tick);

void App_Init(void) {
    Wheel_t *demo_wheel = &STATUS.motor.wheel[APP_DEMO_WHEEL_INDEX];

    memset(&STATUS, 0, sizeof(STATUS));

    App_InitMotor();
    App_InitSensor();
    App_InitDevice();
    App_InitState();
    App_InitRuntime(HAL_GetTick());

    STATUS.runtime.demo_setpoint = 50.0f;
    demo_wheel->target_speed = STATUS.runtime.demo_setpoint;
    PID_Init(&demo_wheel->pid, 1.0f, 0.2f, 0.01f);

    PRINTLN("hello world");
    PRINTLN("PID start: kp=%.2f ki=%.2f kd=%.2f setpoint=%.2f",
            demo_wheel->pid.kp,
            demo_wheel->pid.ki,
            demo_wheel->pid.kd,
            demo_wheel->target_speed);
}

static void App_InitMotor(void) {
    uint32_t index = 0U;

    for (index = 0U; index < STATUS_WHEEL_COUNT; ++index) {
        STATUS.motor.wheel[index].current_speed = 0.0f;
        STATUS.motor.wheel[index].target_speed = 0.0f;
        STATUS.motor.wheel[index].pwm_duty = 0;
    }

    for (index = 0U; index < STATUS_SERVO_COUNT; ++index) {
        STATUS.motor.servo[index].current_angle = 0.0f;
        STATUS.motor.servo[index].target_angle = 0.0f;
    }
}

static void App_InitSensor(void) {
    STATUS.sensor.gyro.yaw = 0.0f;
    STATUS.sensor.gyro.pitch = 0.0f;
    STATUS.sensor.gyro.roll = 0.0f;
}

static void App_InitDevice(void) {
    STATUS.device.led.is_on = 0U;
    STATUS.device.led.is_bound = 0U;
    STATUS.device.led.port = NULL;
    STATUS.device.led.pin = 0U;
    STATUS.device.led.active_state = GPIO_PIN_SET;
}

static void App_InitState(void) {
    STATUS.state.main_mode = 0;
    STATUS.state.sub_mode = 0;
}

static void App_InitRuntime(uint32_t now_tick) {
    STATUS.runtime.demo_setpoint = 0.0f;
    STATUS.runtime.measurement = 0.0f;
    STATUS.runtime.control_out = 0.0f;
    STATUS.runtime.last_tick = now_tick;
    STATUS.runtime.last_log_tick = now_tick;
    STATUS.runtime.last_hello_tick = now_tick;
}
