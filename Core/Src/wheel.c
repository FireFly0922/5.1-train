#include "wheel.h"

#include <stdint.h>
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
static void Wheel_UpdateMotorTest(uint32_t now_tick);
static void Wheel_UpdatePID(uint32_t now_tick, float dt);
static uint8_t Wheel_UpdateGwOpenLoop(float *left_pwm, float *right_pwm, float dt);
static void Wheel_UpdateGwTargets(Wheel_t *left_wheel, Wheel_t *right_wheel, float dt);
static float Wheel_UpdateLegacyPid(PID_t *pid, float target, float current, float dt);
static int32_t Wheel_AddPidFeedforward(float target, float pid_output);
static float Wheel_Clamp(float value, float min_value, float max_value);
static void Wheel_CopyGwStatus(GW_Status status);
static void Wheel_PrintGwSamples(void);
static int32_t Wheel_FloatToScaled(float value, float scale);

static GW_Sensor g_gw_sensor;
static PID_t g_gw_line_pid;
static uint32_t g_motor_arm_tick;

#if APP_WHEEL_TEST_ENABLE
typedef struct {
    const char *name;
    int32_t left_pwm;
    int32_t right_pwm;
    uint32_t duration_ms;
} Wheel_TestStep;

static const Wheel_TestStep g_wheel_test_steps[] = {
    {"left forward", APP_WHEEL_TEST_PWM, 0, APP_WHEEL_TEST_RUN_MS},
    {"stop", 0, 0, APP_WHEEL_TEST_STOP_MS},
    {"left reverse", -APP_WHEEL_TEST_PWM, 0, APP_WHEEL_TEST_RUN_MS},
    {"stop", 0, 0, APP_WHEEL_TEST_STOP_MS},
    {"right forward", 0, APP_WHEEL_TEST_PWM, APP_WHEEL_TEST_RUN_MS},
    {"stop", 0, 0, APP_WHEEL_TEST_STOP_MS},
    {"right reverse", 0, -APP_WHEEL_TEST_PWM, APP_WHEEL_TEST_RUN_MS},
    {"stop", 0, 0, APP_WHEEL_TEST_STOP_MS},
};

static uint32_t g_wheel_test_start_tick;
static uint32_t g_wheel_test_last_step;
static uint32_t g_wheel_test_last_log_tick;
#endif

void Wheel_Init(void) {
    Wheel_t *left_wheel = &STATUS.motor.wheel[WHEEL_LEFT_INDEX];
    Wheel_t *right_wheel = &STATUS.motor.wheel[WHEEL_RIGHT_INDEX];

    memset(&STATUS, 0, sizeof(STATUS));

    Wheel_InitMotor();
    Wheel_InitState();
    Wheel_InitRuntime(HAL_GetTick());

#if APP_WHEEL_TEST_ENABLE
    g_wheel_test_start_tick = HAL_GetTick();
    g_wheel_test_last_step = UINT32_MAX;
    g_wheel_test_last_log_tick = 0U;
    PRINTLN("wheel test enabled: pwm=%d run=%lu stop=%lu",
            APP_WHEEL_TEST_PWM,
            (unsigned long)APP_WHEEL_TEST_RUN_MS,
            (unsigned long)APP_WHEEL_TEST_STOP_MS);
    PRINTLN("sequence: left forward, left reverse, right forward, right reverse");
    return;
#endif

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
    PRINTLN("mapping: motor_out L=%u R=%u enc_timer L=%u R=%u",
            (unsigned int)APP_MOTOR_LEFT_OUTPUT,
            (unsigned int)APP_MOTOR_RIGHT_OUTPUT,
            (unsigned int)APP_ENCODER_LEFT_TIMER,
            (unsigned int)APP_ENCODER_RIGHT_TIMER);
    PRINTLN("drive: mode=%u straight_speed_x100=%ld",
            (unsigned int)APP_DRIVE_MODE,
            (long)Wheel_FloatToScaled(APP_STRAIGHT_TEST_SPEED, 100.0f));
    PRINTLN("drive: straight_pwm=%d",
            APP_STRAIGHT_TEST_PWM);
    PRINTLN("motor: arm=%lu motor_sign L=%d R=%d enc_sign L=%d R=%d",
            (unsigned long)APP_MOTOR_ARM_DELAY_MS,
            APP_MOTOR_LEFT_SIGN,
            APP_MOTOR_RIGHT_SIGN,
            APP_ENCODER_LEFT_SIGN,
            APP_ENCODER_RIGHT_SIGN);
    PRINTLN("GW: base_x100=%ld kp_x100=%ld max_corr_x100=%ld steer_x10=%ld low=%u black=%u white=%u",
            (long)Wheel_FloatToScaled(APP_GW_BASE_SPEED, 100.0f),
            (long)Wheel_FloatToScaled(APP_GW_LINE_KP, 100.0f),
            (long)Wheel_FloatToScaled(APP_GW_MAX_CORRECTION, 100.0f),
            (long)Wheel_FloatToScaled(APP_GW_STEER_SIGN, 10.0f),
            APP_GW_LOW_THRESHOLD,
            APP_GW_CALIBRATION_BLACK,
            APP_GW_CALIBRATION_WHITE);
    PRINTLN("wheel PID x100: left %ld/%ld/%ld right %ld/%ld/%ld speed=%ld",
            (long)Wheel_FloatToScaled(left_wheel->pid.kp, 100.0f),
            (long)Wheel_FloatToScaled(left_wheel->pid.ki, 100.0f),
            (long)Wheel_FloatToScaled(left_wheel->pid.kd, 100.0f),
            (long)Wheel_FloatToScaled(right_wheel->pid.kp, 100.0f),
            (long)Wheel_FloatToScaled(right_wheel->pid.ki, 100.0f),
            (long)Wheel_FloatToScaled(right_wheel->pid.kd, 100.0f),
            (long)Wheel_FloatToScaled(left_wheel->target_speed, 100.0f));
}

void Wheel_Update(void) {
    uint32_t now_tick = HAL_GetTick();
    float dt = (now_tick - STATUS.runtime.last_tick) / 1000.0f;

    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    STATUS.runtime.last_tick = now_tick;

#if APP_WHEEL_TEST_ENABLE
    Wheel_UpdateMotorTest(now_tick);
    HAL_Delay(APP_MAIN_LOOP_DELAY_MS);
    return;
#endif

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
    STATUS.sensor.vision_class_id = 0xFFU;
    STATUS.sensor.vision_prob = 0U;
    STATUS.sensor.vision_valid = 0U;
    STATUS.state.route_id = ROUTE_ID_NONE;
    STATUS.state.route_stage = ROUTE_STAGE_WAIT_VISION;
    STATUS.state.route_first_action = ROUTE_ACTION_NONE;
    STATUS.state.route_second_action = ROUTE_ACTION_NONE;
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

static void Wheel_UpdateMotorTest(uint32_t now_tick) {
#if APP_WHEEL_TEST_ENABLE
    const uint32_t step_count = (uint32_t)(sizeof(g_wheel_test_steps) / sizeof(g_wheel_test_steps[0]));
    uint32_t elapsed = now_tick - g_wheel_test_start_tick;
    uint32_t cycle_ms = 0U;
    uint32_t step_index = 0U;
    uint32_t step_elapsed;
    uint32_t index;
    const Wheel_TestStep *step;
    int32_t speed_left;
    int32_t speed_right;

    for (index = 0U; index < step_count; ++index) {
        cycle_ms += g_wheel_test_steps[index].duration_ms;
    }

    if (cycle_ms == 0U) {
        Motor_SetSpeed(WHEEL_LEFT_ID, 0);
        Motor_SetSpeed(WHEEL_RIGHT_ID, 0);
        return;
    }

    elapsed %= cycle_ms;
    step_elapsed = elapsed;

    for (index = 0U; index < step_count; ++index) {
        if (step_elapsed < g_wheel_test_steps[index].duration_ms) {
            step_index = index;
            break;
        }

        step_elapsed -= g_wheel_test_steps[index].duration_ms;
    }

    step = &g_wheel_test_steps[step_index];
    Motor_SetSpeed(WHEEL_LEFT_ID, step->left_pwm);
    Motor_SetSpeed(WHEEL_RIGHT_ID, step->right_pwm);

    speed_left = Motor_ReadSpeed(WHEEL_LEFT_ID);
    speed_right = Motor_ReadSpeed(WHEEL_RIGHT_ID);

    if ((step_index != g_wheel_test_last_step) ||
        ((now_tick - g_wheel_test_last_log_tick) >= APP_LOG_PERIOD_MS)) {
        PRINTLN("wheel test: %s | cmd L=%ld R=%ld | pwm L=%ld R=%ld | enc L=%ld R=%ld",
                step->name,
                (long)step->left_pwm,
                (long)step->right_pwm,
                (long)STATUS.motor.wheel[WHEEL_LEFT_INDEX].pwm_duty,
                (long)STATUS.motor.wheel[WHEEL_RIGHT_INDEX].pwm_duty,
                (long)speed_left,
                (long)speed_right);
        g_wheel_test_last_step = step_index;
        g_wheel_test_last_log_tick = now_tick;
    }
#else
    (void)now_tick;
#endif
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
#if APP_DRIVE_MODE == APP_DRIVE_MODE_STRAIGHT_OPEN_PWM
        left_wheel->target_speed = 0.0f;
        right_wheel->target_speed = 0.0f;
        left_pwm = (float)APP_STRAIGHT_TEST_PWM;
        right_pwm = (float)APP_STRAIGHT_TEST_PWM;
        left_pwm = (float)Wheel_AddPidFeedforward(1.0f, left_pwm);
        right_pwm = (float)Wheel_AddPidFeedforward(1.0f, right_pwm);
        PID_Reset(&left_wheel->pid);
        PID_Reset(&right_wheel->pid);
        PID_Reset(&g_gw_line_pid);
#elif APP_DRIVE_MODE == APP_DRIVE_MODE_STRAIGHT_PID
        left_wheel->target_speed = APP_STRAIGHT_TEST_SPEED;
        right_wheel->target_speed = APP_STRAIGHT_TEST_SPEED;
        left_pwm = Wheel_UpdateLegacyPid(&left_wheel->pid,
                                         left_wheel->target_speed,
                                         (float)speed_left,
                                         dt);
        right_pwm = Wheel_UpdateLegacyPid(&right_wheel->pid,
                                          right_wheel->target_speed,
                                          (float)speed_right,
                                          dt);
        left_pwm = (float)Wheel_AddPidFeedforward(left_wheel->target_speed, left_pwm);
        right_pwm = (float)Wheel_AddPidFeedforward(right_wheel->target_speed, right_pwm);
#else
        if (Wheel_UpdateGwOpenLoop(&left_pwm, &right_pwm, dt) != 0u) {
            PID_Reset(&left_wheel->pid);
            PID_Reset(&right_wheel->pid);
        } else {
            left_wheel->target_speed = 0.0f;
            right_wheel->target_speed = 0.0f;
        }
#endif
    }

    Motor_SetRawPwm(WHEEL_LEFT_ID, (int32_t)left_pwm);
    Motor_SetRawPwm(WHEEL_RIGHT_ID, (int32_t)right_pwm);

    STATUS.runtime.demo_setpoint = left_wheel->target_speed;
    STATUS.runtime.measurement = (left_wheel->current_speed + right_wheel->current_speed) * 0.5f;
    STATUS.runtime.control_out = (left_pwm + right_pwm) * 0.5f;

    if ((now_tick - STATUS.runtime.last_log_tick) >= APP_LOG_PERIOD_MS) {
        PRINTLN("GW bits=0x%02X pos_x100=%ld corr_pwm=%ld lost=%u cross=%u st=%ld armed=%u | L target_pwm=%ld speed=%ld pwm=%ld | R target_pwm=%ld speed=%ld pwm=%ld dt_x1000=%ld",
                STATUS.sensor.gw.line_bits,
                (long)Wheel_FloatToScaled(STATUS.sensor.gw.line_position, 100.0f),
                (long)((left_wheel->target_speed - right_wheel->target_speed) * 0.5f),
                STATUS.sensor.gw.lost,
                STATUS.sensor.gw.cross,
                (long)STATUS.sensor.gw.last_status,
                motor_armed,
                (long)left_wheel->target_speed,
                (long)speed_left,
                (long)left_wheel->pwm_duty,
                (long)right_wheel->target_speed,
                (long)speed_right,
                (long)right_wheel->pwm_duty,
                (long)Wheel_FloatToScaled(dt, 1000.0f));
        Wheel_PrintGwSamples();
        STATUS.runtime.last_log_tick = now_tick;
    }
}

static uint8_t Wheel_UpdateGwOpenLoop(float *left_pwm, float *right_pwm, float dt) {
    GW_Status status;
    uint8_t bits;
    uint8_t left_trigger;
    uint8_t right_trigger;
    float position;
    float base_pwm;
    float turn_pwm;
    float slow_pwm;

    (void)dt;

    status = GW_Update(&g_gw_sensor);
    Wheel_CopyGwStatus(status);

    if (status != GW_OK) {
        *left_pwm = 0.0f;
        *right_pwm = 0.0f;
        PID_Reset(&g_gw_line_pid);
        return 0u;
    }

    bits = GW_GetLineBits(&g_gw_sensor);
    if (bits == 0x00u) {
        *left_pwm = 0.0f;
        *right_pwm = 0.0f;
        PID_Reset(&g_gw_line_pid);
        return 0u;
    }

    PID_Reset(&g_gw_line_pid);

    base_pwm = Wheel_Clamp((float)APP_GW_OPEN_PWM, 0.0f, (float)APP_MAX_PWM);
    turn_pwm = (float)APP_GW_OPEN_TURN_PWM;
    if ((bits & 0x81u) != 0u) {
        turn_pwm = (float)APP_GW_OPEN_SHARP_TURN_PWM;
    }

    slow_pwm = Wheel_Clamp(base_pwm - turn_pwm,
                           (float)APP_GW_MIN_FORWARD_PWM,
                           base_pwm);

    *left_pwm = base_pwm;
    *right_pwm = base_pwm;

    if ((bits != 0xFFu) &&
        (GW_IsCross(&g_gw_sensor) == 0u) &&
        ((bits & 0x18u) == 0u)) {
        left_trigger = (uint8_t)(bits & 0x07u);
        right_trigger = (uint8_t)(bits & 0xE0u);
        position = GW_GetPosition(&g_gw_sensor);

        if ((left_trigger != 0u) && (right_trigger == 0u)) {
            *left_pwm = slow_pwm;
        } else if ((right_trigger != 0u) && (left_trigger == 0u)) {
            *right_pwm = slow_pwm;
        } else if (position < 0.0f) {
            *left_pwm = slow_pwm;
        } else if (position > 0.0f) {
            *right_pwm = slow_pwm;
        }
    }

    *left_pwm = Wheel_Clamp(*left_pwm, 0.0f, (float)APP_MAX_PWM);
    *right_pwm = Wheel_Clamp(*right_pwm, 0.0f, (float)APP_MAX_PWM);

    STATUS.motor.wheel[WHEEL_LEFT_INDEX].target_speed = *left_pwm;
    STATUS.motor.wheel[WHEEL_RIGHT_INDEX].target_speed = *right_pwm;

    return 1u;
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

#if APP_GW_STOP_ON_CROSS
    if (GW_IsCross(&g_gw_sensor) != 0u) {
        left_wheel->target_speed = 0.0f;
        right_wheel->target_speed = 0.0f;
        PID_Reset(&g_gw_line_pid);
        return;
    }
#endif

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

static int32_t Wheel_AddPidFeedforward(float target, float pid_output) {
    float output;

    if (target > 0.0f) {
        output = pid_output + (float)APP_PWM_FEEDFORWARD;
        if (output < 0.0f) {
            output = 0.0f;
        }
    } else if (target < 0.0f) {
        output = pid_output - (float)APP_PWM_FEEDFORWARD;
        if (output > 0.0f) {
            output = 0.0f;
        }
    } else {
        output = 0.0f;
    }

    return (int32_t)Wheel_Clamp(output, (float)-APP_MAX_PWM, (float)APP_MAX_PWM);
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

static int32_t Wheel_FloatToScaled(float value, float scale) {
    float scaled = value * scale;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    return (int32_t)scaled;
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
