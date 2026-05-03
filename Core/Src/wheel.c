#include "wheel.h"

#include <string.h>

#include "adc.h"
#include "app_config.h"
#include "GW.h"
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
static void Wheel_InitGw(void);
static void Wheel_InitState(void);
static void Wheel_InitRuntime(uint32_t now_tick);
static void Wheel_UpdatePID(uint32_t now_tick, float dt);
static void Wheel_UpdateGwTargets(Wheel_t *left_wheel, Wheel_t *right_wheel, float dt);
static float Wheel_UpdateLegacyPid(PID_t *pid, float target, float current, float dt);
static float Wheel_Clamp(float value, float min_value, float max_value);
static void Wheel_CopyGwStatus(GW_Status status);
static void Wheel_PrintGwSamples(void);

static GW_Sensor g_gw_sensor;
static PID_t g_gw_line_pid;
static uint32_t g_motor_arm_tick;

void Wheel_Init(void) {
    Wheel_t *left_wheel = &STATUS.motor.wheel[WHEEL_LEFT_INDEX];
    Wheel_t *right_wheel = &STATUS.motor.wheel[WHEEL_RIGHT_INDEX];

    memset(&STATUS, 0, sizeof(STATUS));

    Wheel_InitMotor();
    Wheel_InitState();
    Wheel_InitRuntime(HAL_GetTick());
    Wheel_InitGw();

    STATUS.runtime.demo_setpoint = APP_GW_BASE_SPEED;

    left_wheel->target_speed = APP_GW_BASE_SPEED;
    PID_Init(&left_wheel->pid,
             APP_PID_LEFT_KP,
             APP_PID_LEFT_KI,
             APP_PID_LEFT_KD);
    PID_SetIntegralLimit(&left_wheel->pid, APP_PID_INTEGRAL_LIMIT);

    right_wheel->target_speed = APP_GW_BASE_SPEED;
    PID_Init(&right_wheel->pid,
             APP_PID_RIGHT_KP,
             APP_PID_RIGHT_KI,
             APP_PID_RIGHT_KD);
    PID_SetIntegralLimit(&right_wheel->pid, APP_PID_INTEGRAL_LIMIT);

    PRINTLN("config: max_pwm=%d deadband=%d feedforward=%d",
            APP_MAX_PWM,
            APP_PWM_DEADBAND,
            APP_PWM_FEEDFORWARD);
    PRINTLN("motor: arm=%lu left_sign=%d right_sign=%d enc_left=%d enc_right=%d",
            (unsigned long)APP_MOTOR_ARM_DELAY_MS,
            APP_MOTOR_LEFT_SIGN,
            APP_MOTOR_RIGHT_SIGN,
            APP_ENCODER_LEFT_SIGN,
            APP_ENCODER_RIGHT_SIGN);
    PRINTLN("GW: base=%.2f kp=%.2f max_corr=%.2f steer=%.1f low=%u black=%u white=%u",
            APP_GW_BASE_SPEED,
            APP_GW_LINE_KP,
            APP_GW_MAX_CORRECTION,
            APP_GW_STEER_SIGN,
            APP_GW_LOW_THRESHOLD,
            APP_GW_CALIBRATION_BLACK,
            APP_GW_CALIBRATION_WHITE);
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

static void Wheel_InitGw(void) {
    static const uint16_t white[GW_CHANNELS] = {
        APP_GW_CALIBRATION_WHITE,
        APP_GW_CALIBRATION_WHITE,
        APP_GW_CALIBRATION_WHITE,
        APP_GW_CALIBRATION_WHITE,
        APP_GW_CALIBRATION_WHITE,
        APP_GW_CALIBRATION_WHITE,
        APP_GW_CALIBRATION_WHITE,
        APP_GW_CALIBRATION_WHITE,
    };
    static const uint16_t black[GW_CHANNELS] = {
        APP_GW_CALIBRATION_BLACK,
        APP_GW_CALIBRATION_BLACK,
        APP_GW_CALIBRATION_BLACK,
        APP_GW_CALIBRATION_BLACK,
        APP_GW_CALIBRATION_BLACK,
        APP_GW_CALIBRATION_BLACK,
        APP_GW_CALIBRATION_BLACK,
        APP_GW_CALIBRATION_BLACK,
    };
    GW_Config config;
    GW_Status status;

    memset(&config, 0, sizeof(config));
    config.hadc = &hadc2;
    config.adc_channel = ADC_CHANNEL_11;
    config.adc_rank = ADC_REGULAR_RANK_1;
    config.adc_sampling_time = ADC_SAMPLETIME_64CYCLES_5;
    config.adc_single_diff = ADC_SINGLE_ENDED;
    config.adc_timeout_ms = GW_DEFAULT_ADC_TIMEOUT;
    config.address[0].port = AD0_GPIO_Port;
    config.address[0].pin = AD0_Pin;
    config.address[1].port = AD1_GPIO_Port;
    config.address[1].pin = AD1_Pin;
    config.address[2].port = AD2_GPIO_Port;
    config.address[2].pin = AD2_Pin;

    status = GW_InitCalibrated(&g_gw_sensor, &config, white, black);
    GW_SetThreshold(&g_gw_sensor, APP_GW_LOW_THRESHOLD);
    Wheel_CopyGwStatus(status);

    PID_Init(&g_gw_line_pid,
             APP_GW_LINE_KP,
             APP_GW_LINE_KI,
             APP_GW_LINE_KD);
    PID_SetIntegralLimit(&g_gw_line_pid, APP_PID_INTEGRAL_LIMIT);
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
    g_motor_arm_tick = now_tick;
}

// Drive both wheels with the same speed loop parameters used by legacy.
static void Wheel_UpdatePID(uint32_t now_tick, float dt) {
    Wheel_t *left_wheel = &STATUS.motor.wheel[WHEEL_LEFT_INDEX];
    Wheel_t *right_wheel = &STATUS.motor.wheel[WHEEL_RIGHT_INDEX];
    float left_pwm;
    float right_pwm;
    int32_t speed_left;
    int32_t speed_right;
    uint8_t motor_armed;

    Wheel_UpdateGwTargets(left_wheel, right_wheel, dt);

    speed_left = Motor_ReadSpeed(WHEEL_LEFT_ID);
    speed_right = Motor_ReadSpeed(WHEEL_RIGHT_ID);
    STATUS.runtime.total_pulses += (speed_left + speed_right) / 2;

    left_pwm = 0.0f;
    right_pwm = 0.0f;
    motor_armed = ((now_tick - g_motor_arm_tick) >= APP_MOTOR_ARM_DELAY_MS) ? 1u : 0u;

    if (motor_armed == 0u) {
        left_wheel->target_speed = 0.0f;
        right_wheel->target_speed = 0.0f;
        PID_Reset(&left_wheel->pid);
        PID_Reset(&right_wheel->pid);
        PID_Reset(&g_gw_line_pid);
    } else {
        left_pwm = Wheel_UpdateLegacyPid(&left_wheel->pid,
                                         left_wheel->target_speed,
                                         (float)speed_left,
                                         dt);
        right_pwm = Wheel_UpdateLegacyPid(&right_wheel->pid,
                                          right_wheel->target_speed,
                                          (float)speed_right,
                                          dt);
    }

    Motor_SetSpeed(WHEEL_LEFT_ID, (int32_t)left_pwm);
    Motor_SetSpeed(WHEEL_RIGHT_ID, (int32_t)right_pwm);

    STATUS.runtime.demo_setpoint = left_wheel->target_speed;
    STATUS.runtime.measurement = (left_wheel->current_speed + right_wheel->current_speed) * 0.5f;
    STATUS.runtime.control_out = (left_pwm + right_pwm) * 0.5f;

    if ((now_tick - STATUS.runtime.last_log_tick) >= APP_LOG_PERIOD_MS) {
        PRINTLN("GW bits=0x%02X pos=%.2f lost=%u cross=%u st=%ld armed=%u | L sp=%.2f speed=%ld pwm=%ld | R sp=%.2f speed=%ld pwm=%ld dt=%.3f",
                STATUS.sensor.gw.line_bits,
                STATUS.sensor.gw.line_position,
                STATUS.sensor.gw.lost,
                STATUS.sensor.gw.cross,
                (long)STATUS.sensor.gw.last_status,
                motor_armed,
                left_wheel->target_speed,
                (long)speed_left,
                (long)left_wheel->pwm_duty,
                right_wheel->target_speed,
                (long)speed_right,
                (long)right_wheel->pwm_duty,
                dt);
        Wheel_PrintGwSamples();
        STATUS.runtime.last_log_tick = now_tick;
    }
}

static void Wheel_UpdateGwTargets(Wheel_t *left_wheel, Wheel_t *right_wheel, float dt) {
    GW_Status status;
    float correction;

    status = GW_Update(&g_gw_sensor);
    Wheel_CopyGwStatus(status);

    if (status != GW_OK) {
        left_wheel->target_speed = 0.0f;
        right_wheel->target_speed = 0.0f;
        PID_Reset(&g_gw_line_pid);
        return;
    }

    if (GW_IsLost(&g_gw_sensor) != 0u) {
        left_wheel->target_speed = 0.0f;
        right_wheel->target_speed = 0.0f;
        PID_Reset(&g_gw_line_pid);
        return;
    }

    if (GW_IsCross(&g_gw_sensor) != 0u) {
        left_wheel->target_speed = APP_GW_BASE_SPEED;
        right_wheel->target_speed = APP_GW_BASE_SPEED;
        PID_Reset(&g_gw_line_pid);
        return;
    }

    correction = PID_Update(&g_gw_line_pid,
                            APP_GW_STEER_SIGN * GW_GetPosition(&g_gw_sensor),
                            dt);
    correction = Wheel_Clamp(correction,
                             -APP_GW_MAX_CORRECTION,
                             APP_GW_MAX_CORRECTION);

    left_wheel->target_speed = APP_GW_BASE_SPEED + correction;
    right_wheel->target_speed = APP_GW_BASE_SPEED - correction;
}

static float Wheel_UpdateLegacyPid(PID_t *pid, float target, float current, float dt) {
    float output;

    output = PID_Update(pid, target - current, dt);

    return output;
}

static float Wheel_Clamp(float value, float min_value, float max_value) {
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

static void Wheel_CopyGwStatus(GW_Status status) {
    GW_CopyAnalog(&g_gw_sensor, STATUS.sensor.gw.analog);
    GW_CopyNormalized(&g_gw_sensor, STATUS.sensor.gw.normalized);
    STATUS.sensor.gw.line_bits = GW_GetLineBits(&g_gw_sensor);
    STATUS.sensor.gw.line_position = GW_GetPosition(&g_gw_sensor);
    STATUS.sensor.gw.normal = GW_IsNormal(&g_gw_sensor);
    STATUS.sensor.gw.lost = GW_IsLost(&g_gw_sensor);
    STATUS.sensor.gw.cross = GW_IsCross(&g_gw_sensor);
    STATUS.sensor.gw.last_status = (int32_t)status;
}

static void Wheel_PrintGwSamples(void) {
    PRINTLN("GW analog: %u %u %u %u %u %u %u %u",
            (unsigned int)STATUS.sensor.gw.analog[0],
            (unsigned int)STATUS.sensor.gw.analog[1],
            (unsigned int)STATUS.sensor.gw.analog[2],
            (unsigned int)STATUS.sensor.gw.analog[3],
            (unsigned int)STATUS.sensor.gw.analog[4],
            (unsigned int)STATUS.sensor.gw.analog[5],
            (unsigned int)STATUS.sensor.gw.analog[6],
            (unsigned int)STATUS.sensor.gw.analog[7]);
    PRINTLN("GW norm: %u %u %u %u %u %u %u %u",
            (unsigned int)STATUS.sensor.gw.normalized[0],
            (unsigned int)STATUS.sensor.gw.normalized[1],
            (unsigned int)STATUS.sensor.gw.normalized[2],
            (unsigned int)STATUS.sensor.gw.normalized[3],
            (unsigned int)STATUS.sensor.gw.normalized[4],
            (unsigned int)STATUS.sensor.gw.normalized[5],
            (unsigned int)STATUS.sensor.gw.normalized[6],
            (unsigned int)STATUS.sensor.gw.normalized[7]);
}
