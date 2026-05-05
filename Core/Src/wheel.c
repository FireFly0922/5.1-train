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
static uint8_t Wheel_UpdateRoute(float *left_pwm, float *right_pwm, float dt,
                                 uint32_t now_tick, uint8_t *direct_pwm);
static uint8_t Wheel_UpdateGwTargets(Wheel_t *left_wheel, Wheel_t *right_wheel, float dt,
                                     uint32_t now_tick);
static float Wheel_UpdateLegacyPid(PID_t *pid, float target, float current, float dt);
static float Wheel_AddStaticDrive(float target, float pid_output);
static float Wheel_ApplyPwmSlew(float target_pwm, int32_t last_pwm);
static float Wheel_Clamp(float value, float min_value, float max_value);
static void Wheel_CopyGwStatus(GW_Status status);
static void Wheel_PrintGwSamples(void);
static int32_t Wheel_FloatToScaled(float value, float scale);
static const char *Wheel_RouteName(int32_t route_id);
static const char *Wheel_ActionName(int32_t action);
static const char *Wheel_StageName(int32_t stage);
static int32_t Wheel_RouteActionForCross(uint8_t cross_index);
static int32_t Wheel_StageForAction(int32_t action);
static uint8_t Wheel_IsRouteLocked(void);
static uint8_t Wheel_IsReacquired(void);
static uint8_t Wheel_RouteIsDone(void);
static void Wheel_SetRouteStage(int32_t stage, uint32_t now_tick);
static uint8_t Wheel_CountBits(uint8_t value);

static GW_Sensor g_gw_sensor;
static PID_t g_gw_line_pid;
static uint32_t g_motor_arm_tick;
static uint32_t g_gw_ramp_start_tick;
static uint32_t g_route_stage_tick;
static uint8_t g_route_cross_count;
static uint8_t g_route_logged_wait;

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
#if APP_MOTOR_DRIVER_TB6612_2PWM
    PRINTLN("motor driver: TB6612 2PWM wiring, tie PWMA/PWMB/STBY high");
#endif
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
    PID_SetIntegralLimit(&g_gw_line_pid, APP_GW_LINE_INTEGRAL_LIMIT);
}

static void Wheel_InitState(void) {
    STATUS.state.main_mode = 0;
    STATUS.state.sub_mode = 0;
    STATUS.state.car_run_state = 0;
#if APP_ROUTE_FIXED_ENABLE
    STATUS.sensor.vision_class_id = APP_ROUTE_FIXED_CLASS_ID;
    STATUS.sensor.vision_prob = 100U;
    STATUS.sensor.vision_valid = 1U;
    STATUS.sensor.vision_target = (float)APP_ROUTE_FIXED_CLASS_ID;
    STATUS.state.route_id = APP_ROUTE_FIXED_CLASS_ID;
    STATUS.state.route_stage = ROUTE_STAGE_ROUTE_LOCKED;
    if (APP_ROUTE_FIXED_CLASS_ID == ROUTE_ID_A) {
        STATUS.state.route_first_action = ROUTE_ACTION_LEFT;
        STATUS.state.route_second_action = ROUTE_ACTION_NONE;
    } else if (APP_ROUTE_FIXED_CLASS_ID == ROUTE_ID_B) {
        STATUS.state.route_first_action = ROUTE_ACTION_RIGHT;
        STATUS.state.route_second_action = ROUTE_ACTION_NONE;
    } else if (APP_ROUTE_FIXED_CLASS_ID == ROUTE_ID_C) {
        STATUS.state.route_first_action = ROUTE_ACTION_STRAIGHT;
        STATUS.state.route_second_action = ROUTE_ACTION_LEFT;
    } else {
        STATUS.state.route_first_action = ROUTE_ACTION_STRAIGHT;
        STATUS.state.route_second_action = ROUTE_ACTION_RIGHT;
    }
    PRINTLN("route fixed: id=%u first=%s second=%s",
            (unsigned int)APP_ROUTE_FIXED_CLASS_ID,
            Wheel_ActionName(STATUS.state.route_first_action),
            Wheel_ActionName(STATUS.state.route_second_action));
#else
    STATUS.sensor.vision_class_id = 0xFFU;
    STATUS.sensor.vision_prob = 0U;
    STATUS.sensor.vision_valid = 0U;
    STATUS.state.route_id = ROUTE_ID_NONE;
    STATUS.state.route_stage = ROUTE_STAGE_WAIT_VISION;
    STATUS.state.route_first_action = ROUTE_ACTION_NONE;
    STATUS.state.route_second_action = ROUTE_ACTION_NONE;
#endif
    g_route_stage_tick = HAL_GetTick();
    g_route_cross_count = 0U;
    g_route_logged_wait = 0U;
}

static void Wheel_InitRuntime(uint32_t now_tick) {
    STATUS.runtime.demo_setpoint = 0.0f;
    STATUS.runtime.measurement = 0.0f;
    STATUS.runtime.control_out = 0.0f;
    STATUS.runtime.total_pulses = 0;
    STATUS.runtime.last_tick = now_tick;
    STATUS.runtime.last_log_tick = now_tick;
    g_motor_arm_tick = now_tick;
    g_gw_ramp_start_tick = now_tick + APP_MOTOR_ARM_DELAY_MS;
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
    int32_t left_cmd;
    int32_t right_cmd;
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
    left_cmd = step->left_pwm;
    right_cmd = step->right_pwm;
    if (step_elapsed < APP_MOTOR_START_BOOST_MS) {
        if (left_cmd > 0) {
            left_cmd = APP_MOTOR_START_BOOST_PWM;
        } else if (left_cmd < 0) {
            left_cmd = -APP_MOTOR_START_BOOST_PWM;
        }

        if (right_cmd > 0) {
            right_cmd = APP_MOTOR_START_BOOST_PWM;
        } else if (right_cmd < 0) {
            right_cmd = -APP_MOTOR_START_BOOST_PWM;
        }
    }

    Motor_SetRawPwm(WHEEL_LEFT_ID, left_cmd);
    Motor_SetRawPwm(WHEEL_RIGHT_ID, right_cmd);

    speed_left = Motor_ReadSpeed(WHEEL_LEFT_ID);
    speed_right = Motor_ReadSpeed(WHEEL_RIGHT_ID);

    if ((step_index != g_wheel_test_last_step) ||
        ((now_tick - g_wheel_test_last_log_tick) >= APP_LOG_PERIOD_MS)) {
        PRINTLN("wheel test: %s | cmd L=%ld R=%ld | pwm L=%ld R=%ld | enc L=%ld R=%ld",
                step->name,
                (long)left_cmd,
                (long)right_cmd,
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

static uint8_t Wheel_UpdateRoute(float *left_pwm, float *right_pwm, float dt,
                                 uint32_t now_tick, uint8_t *direct_pwm) {
#if APP_ROUTE_ENABLE
    Wheel_t *left_wheel = &STATUS.motor.wheel[WHEEL_LEFT_INDEX];
    Wheel_t *right_wheel = &STATUS.motor.wheel[WHEEL_RIGHT_INDEX];
    int32_t action;
    GW_Status status;
    uint8_t gw_ok;

    *left_pwm = 0.0f;
    *right_pwm = 0.0f;
    *direct_pwm = 0U;

    if (Wheel_IsRouteLocked() == 0u) {
        status = GW_Update(&g_gw_sensor);
        Wheel_CopyGwStatus(status);
        *left_pwm = 0.0f;
        *right_pwm = 0.0f;
        *direct_pwm = 1U;
        left_wheel->target_speed = 0.0f;
        right_wheel->target_speed = 0.0f;
        PID_Reset(&g_gw_line_pid);
        if (STATUS.state.route_stage != ROUTE_STAGE_WAIT_ROUTE) {
            Wheel_SetRouteStage(ROUTE_STAGE_WAIT_ROUTE, now_tick);
        } else if (g_route_logged_wait == 0U) {
            PRINTLN("route wait: need valid MaixCam ABCD frame");
            g_route_logged_wait = 1U;
        }
        return 1u;
    }

    g_route_logged_wait = 0U;
    if (STATUS.state.route_stage == ROUTE_STAGE_ROUTE_LOCKED) {
        g_route_cross_count = 0U;
        Wheel_SetRouteStage(ROUTE_STAGE_LINE_FOLLOW, now_tick);
    }

    switch (STATUS.state.route_stage) {
        case ROUTE_STAGE_WAIT_ROUTE:
            Wheel_SetRouteStage(ROUTE_STAGE_LINE_FOLLOW, now_tick);
            break;

        case ROUTE_STAGE_LINE_FOLLOW:
            gw_ok = Wheel_UpdateGwTargets(left_wheel, right_wheel, dt, now_tick);
            if (gw_ok == 0U) {
                if ((STATUS.sensor.gw.lost != 0U) &&
                    (g_route_cross_count >= 1U) &&
                    (STATUS.state.route_second_action != ROUTE_ACTION_NONE) &&
                    ((now_tick - g_route_stage_tick) >= APP_ROUTE_CROSS_EXIT_MS)) {
                    g_route_cross_count = 2U;
                    Wheel_SetRouteStage(Wheel_StageForAction(STATUS.state.route_second_action),
                                        now_tick);
                    return 1u;
                }
                return 0u;
            }

            if (STATUS.sensor.gw.cross != 0U) {
                Wheel_SetRouteStage(ROUTE_STAGE_CROSS_DETECTED, now_tick);
            }
            break;

        case ROUTE_STAGE_CROSS_DETECTED:
            gw_ok = Wheel_UpdateGwTargets(left_wheel, right_wheel, dt, now_tick);
            if (gw_ok == 0U) {
                return 0u;
            }

            if ((now_tick - g_route_stage_tick) >= APP_ROUTE_CROSS_DEBOUNCE_MS) {
                g_route_cross_count++;
                action = Wheel_RouteActionForCross(g_route_cross_count);
                Wheel_SetRouteStage(Wheel_StageForAction(action), now_tick);
            }
            break;

        case ROUTE_STAGE_STRAIGHT_THROUGH:
            *left_pwm = (float)APP_GW_OPEN_PWM;
            *right_pwm = (float)APP_GW_OPEN_PWM;
            *direct_pwm = 1U;
            left_wheel->target_speed = *left_pwm;
            right_wheel->target_speed = *right_pwm;
            PID_Reset(&g_gw_line_pid);
            if ((now_tick - g_route_stage_tick) >= APP_ROUTE_STRAIGHT_MS) {
                Wheel_SetRouteStage(ROUTE_STAGE_REACQUIRE_LINE, now_tick);
            }
            break;

        case ROUTE_STAGE_TURN_LEFT:
            *left_pwm = -(float)APP_ROUTE_TURN_LEFT_PWM;
            *right_pwm = (float)APP_ROUTE_TURN_LEFT_PWM;
            *direct_pwm = 1U;
            left_wheel->target_speed = *left_pwm;
            right_wheel->target_speed = *right_pwm;
            PID_Reset(&g_gw_line_pid);
            status = GW_Update(&g_gw_sensor);
            Wheel_CopyGwStatus(status);
            if (((now_tick - g_route_stage_tick) >= APP_ROUTE_TURN_MIN_MS) &&
                (Wheel_IsReacquired() != 0U)) {
                if (Wheel_RouteIsDone() != 0U) {
                    Wheel_SetRouteStage(ROUTE_STAGE_DONE, now_tick);
                } else {
                    Wheel_SetRouteStage(ROUTE_STAGE_REACQUIRE_LINE, now_tick);
                }
            } else if ((now_tick - g_route_stage_tick) >= APP_ROUTE_TURN_TIMEOUT_MS) {
                if (Wheel_RouteIsDone() != 0U) {
                    Wheel_SetRouteStage(ROUTE_STAGE_DONE, now_tick);
                } else {
                    Wheel_SetRouteStage(ROUTE_STAGE_REACQUIRE_LINE, now_tick);
                }
            }
            break;

        case ROUTE_STAGE_TURN_RIGHT:
            *left_pwm = (float)APP_ROUTE_TURN_RIGHT_PWM;
            *right_pwm = -(float)APP_ROUTE_TURN_RIGHT_PWM;
            *direct_pwm = 1U;
            left_wheel->target_speed = *left_pwm;
            right_wheel->target_speed = *right_pwm;
            PID_Reset(&g_gw_line_pid);
            status = GW_Update(&g_gw_sensor);
            Wheel_CopyGwStatus(status);
            if (((now_tick - g_route_stage_tick) >= APP_ROUTE_TURN_MIN_MS) &&
                (Wheel_IsReacquired() != 0U)) {
                if (Wheel_RouteIsDone() != 0U) {
                    Wheel_SetRouteStage(ROUTE_STAGE_DONE, now_tick);
                } else {
                    Wheel_SetRouteStage(ROUTE_STAGE_REACQUIRE_LINE, now_tick);
                }
            } else if ((now_tick - g_route_stage_tick) >= APP_ROUTE_TURN_TIMEOUT_MS) {
                if (Wheel_RouteIsDone() != 0U) {
                    Wheel_SetRouteStage(ROUTE_STAGE_DONE, now_tick);
                } else {
                    Wheel_SetRouteStage(ROUTE_STAGE_REACQUIRE_LINE, now_tick);
                }
            }
            break;

        case ROUTE_STAGE_REACQUIRE_LINE:
            gw_ok = Wheel_UpdateGwTargets(left_wheel, right_wheel, dt, now_tick);
            if (gw_ok == 0U) {
                return 0u;
            }

            if (((now_tick - g_route_stage_tick) >= APP_ROUTE_CROSS_EXIT_MS) &&
                (Wheel_IsReacquired() != 0U)) {
                if (Wheel_RouteIsDone() != 0U) {
                    Wheel_SetRouteStage(ROUTE_STAGE_DONE, now_tick);
                } else {
                    Wheel_SetRouteStage(ROUTE_STAGE_LINE_FOLLOW, now_tick);
                }
            }
            break;

        case ROUTE_STAGE_DONE:
        case ROUTE_STAGE_INVALID:
        default:
            *left_pwm = 0.0f;
            *right_pwm = 0.0f;
            *direct_pwm = 1U;
            left_wheel->target_speed = 0.0f;
            right_wheel->target_speed = 0.0f;
            PID_Reset(&g_gw_line_pid);
            break;
    }

    if (*direct_pwm != 0U) {
        *left_pwm = Wheel_Clamp(*left_pwm, (float)-APP_MAX_PWM, (float)APP_MAX_PWM);
        *right_pwm = Wheel_Clamp(*right_pwm, (float)-APP_MAX_PWM, (float)APP_MAX_PWM);
    }

    return 1u;
#else
    Wheel_t *left_wheel = &STATUS.motor.wheel[WHEEL_LEFT_INDEX];
    Wheel_t *right_wheel = &STATUS.motor.wheel[WHEEL_RIGHT_INDEX];

    *direct_pwm = 0U;
    *left_pwm = 0.0f;
    *right_pwm = 0.0f;
    return Wheel_UpdateGwTargets(left_wheel, right_wheel, dt, HAL_GetTick());
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
    uint8_t direct_pwm;

    speed_left = Motor_ReadSpeed(WHEEL_LEFT_ID);
    speed_right = Motor_ReadSpeed(WHEEL_RIGHT_ID);
    STATUS.runtime.total_pulses += (speed_left + speed_right) / 2;

    left_pwm = 0.0f;
    right_pwm = 0.0f;
    direct_pwm = 0U;
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
        left_pwm = Wheel_AddStaticDrive(left_wheel->target_speed, left_pwm);
        right_pwm = Wheel_AddStaticDrive(right_wheel->target_speed, right_pwm);
#else
        if (Wheel_UpdateRoute(&left_pwm, &right_pwm, dt, now_tick, &direct_pwm) != 0u) {
            if (direct_pwm == 0U) {
                left_pwm = Wheel_UpdateLegacyPid(&left_wheel->pid,
                                                 left_wheel->target_speed,
                                                 (float)speed_left,
                                                 dt);
                right_pwm = Wheel_UpdateLegacyPid(&right_wheel->pid,
                                                  right_wheel->target_speed,
                                                  (float)speed_right,
                                                  dt);
                left_pwm = Wheel_AddStaticDrive(left_wheel->target_speed, left_pwm);
                right_pwm = Wheel_AddStaticDrive(right_wheel->target_speed, right_pwm);
            } else {
                PID_Reset(&left_wheel->pid);
                PID_Reset(&right_wheel->pid);
            }
        } else {
            left_wheel->target_speed = 0.0f;
            right_wheel->target_speed = 0.0f;
            PID_Reset(&left_wheel->pid);
            PID_Reset(&right_wheel->pid);
        }
#endif
    }

    left_pwm = Wheel_ApplyPwmSlew(left_pwm, left_wheel->pwm_duty);
    right_pwm = Wheel_ApplyPwmSlew(right_pwm, right_wheel->pwm_duty);

    Motor_SetRawPwm(WHEEL_LEFT_ID, (int32_t)left_pwm);
    Motor_SetRawPwm(WHEEL_RIGHT_ID, (int32_t)right_pwm);

    STATUS.runtime.demo_setpoint = left_wheel->target_speed;
    STATUS.runtime.measurement = (left_wheel->current_speed + right_wheel->current_speed) * 0.5f;
    STATUS.runtime.control_out = (left_pwm + right_pwm) * 0.5f;

    if ((now_tick - STATUS.runtime.last_log_tick) >= APP_LOG_PERIOD_MS) {
        PRINTLN("route=%s stage=%s cross_n=%u action=%s out=%s | GW bits=0x%02X pos_x100=%ld corr_x100=%ld lost=%u cross=%u st=%ld armed=%u | L target=%ld speed=%ld pwm=%ld | R target=%ld speed=%ld pwm=%ld dt_x1000=%ld",
                Wheel_RouteName(STATUS.state.route_id),
                Wheel_StageName(STATUS.state.route_stage),
                (unsigned int)g_route_cross_count,
                Wheel_ActionName(Wheel_RouteActionForCross(g_route_cross_count)),
                (direct_pwm != 0U) ? "PWM" : "SPD",
                STATUS.sensor.gw.line_bits,
                (long)Wheel_FloatToScaled(STATUS.sensor.gw.line_position, 100.0f),
                (long)Wheel_FloatToScaled((left_wheel->target_speed - right_wheel->target_speed) * 0.5f, 100.0f),
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

static uint8_t Wheel_UpdateGwTargets(Wheel_t *left_wheel, Wheel_t *right_wheel, float dt,
                                     uint32_t now_tick) {
    GW_Status status;
    float correction;
    float base_speed;
    float left_target;
    float right_target;

    status = GW_Update(&g_gw_sensor);
    Wheel_CopyGwStatus(status);

    if (status != GW_OK) {
        left_wheel->target_speed = 0.0f;
        right_wheel->target_speed = 0.0f;
        PID_Reset(&g_gw_line_pid);
        return 0u;
    }

    if (GW_IsLost(&g_gw_sensor) != 0u) {
        left_wheel->target_speed = 0.0f;
        right_wheel->target_speed = 0.0f;
        PID_Reset(&g_gw_line_pid);
        return 0u;
    }

#if APP_GW_STOP_ON_CROSS
    if (GW_IsCross(&g_gw_sensor) != 0u) {
        left_wheel->target_speed = 0.0f;
        right_wheel->target_speed = 0.0f;
        PID_Reset(&g_gw_line_pid);
        return 0u;
    }
#endif

    (void)dt;
    correction = PID_Update(&g_gw_line_pid,
                            APP_GW_STEER_SIGN * GW_GetPosition(&g_gw_sensor),
                            APP_GW_LINE_T_MS);
    correction = Wheel_Clamp(correction,
                             -APP_GW_MAX_CORRECTION,
                             APP_GW_MAX_CORRECTION);

    base_speed = APP_GW_BASE_SPEED;
    if (APP_GW_SPEED_RAMP_MS > 0U) {
        uint32_t run_ms = now_tick - g_gw_ramp_start_tick;

        if (run_ms < APP_GW_SPEED_RAMP_MS) {
            base_speed *= (float)run_ms / (float)APP_GW_SPEED_RAMP_MS;
        }
    }

    left_target = base_speed + correction;
    right_target = base_speed - correction;

    left_wheel->target_speed = Wheel_Clamp(left_target, 0.0f, APP_BASE_SPEED_FAST);
    right_wheel->target_speed = Wheel_Clamp(right_target, 0.0f, APP_BASE_SPEED_FAST);

    return 1u;
}

static float Wheel_UpdateLegacyPid(PID_t *pid, float target, float current, float dt) {
    float output;

    (void)dt;
    output = -PID_Update(pid, current - target, APP_WHEEL_PID_T_MS);

    return output;
}

static float Wheel_AddStaticDrive(float target, float pid_output) {
    float output = pid_output;

    if (target > 0.0f) {
        output += (float)APP_PWM_STATIC_DRIVE;
        if (output < 0.0f) {
            output = 0.0f;
        }
    } else if (target < 0.0f) {
        output -= (float)APP_PWM_STATIC_DRIVE;
        if (output > 0.0f) {
            output = 0.0f;
        }
    } else {
        output = 0.0f;
    }

    return Wheel_Clamp(output, (float)-APP_MAX_PWM, (float)APP_MAX_PWM);
}

static float Wheel_ApplyPwmSlew(float target_pwm, int32_t last_pwm) {
    float min_pwm = (float)last_pwm - (float)APP_PWM_SLEW_STEP;
    float max_pwm = (float)last_pwm + (float)APP_PWM_SLEW_STEP;

    return Wheel_Clamp(target_pwm, min_pwm, max_pwm);
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

static const char *Wheel_RouteName(int32_t route_id) {
    switch (route_id) {
        case ROUTE_ID_A:
            return "A";
        case ROUTE_ID_B:
            return "B";
        case ROUTE_ID_C:
            return "C";
        case ROUTE_ID_D:
            return "D";
        default:
            return "?";
    }
}

static const char *Wheel_ActionName(int32_t action) {
    switch (action) {
        case ROUTE_ACTION_LEFT:
            return "LEFT";
        case ROUTE_ACTION_RIGHT:
            return "RIGHT";
        case ROUTE_ACTION_STRAIGHT:
            return "STRAIGHT";
        default:
            return "NONE";
    }
}

static const char *Wheel_StageName(int32_t stage) {
    switch (stage) {
        case ROUTE_STAGE_WAIT_VISION:
            return "WAIT_VISION";
        case ROUTE_STAGE_ROUTE_LOCKED:
            return "ROUTE_LOCKED";
        case ROUTE_STAGE_WAIT_ROUTE:
            return "WAIT_ROUTE";
        case ROUTE_STAGE_LINE_FOLLOW:
            return "LINE_FOLLOW";
        case ROUTE_STAGE_CROSS_DETECTED:
            return "CROSS_DETECTED";
        case ROUTE_STAGE_TURN_LEFT:
            return "TURN_LEFT";
        case ROUTE_STAGE_TURN_RIGHT:
            return "TURN_RIGHT";
        case ROUTE_STAGE_STRAIGHT_THROUGH:
            return "STRAIGHT_THROUGH";
        case ROUTE_STAGE_REACQUIRE_LINE:
            return "REACQUIRE_LINE";
        case ROUTE_STAGE_DONE:
            return "DONE";
        case ROUTE_STAGE_INVALID:
            return "INVALID";
        default:
            return "UNKNOWN";
    }
}

static int32_t Wheel_RouteActionForCross(uint8_t cross_index) {
    if (cross_index == 0U) {
        return STATUS.state.route_first_action;
    }

    if (cross_index == 1U) {
        return STATUS.state.route_first_action;
    }

    if (cross_index == 2U) {
        return STATUS.state.route_second_action;
    }

    return ROUTE_ACTION_NONE;
}

static int32_t Wheel_StageForAction(int32_t action) {
    switch (action) {
        case ROUTE_ACTION_LEFT:
            return ROUTE_STAGE_TURN_LEFT;
        case ROUTE_ACTION_RIGHT:
            return ROUTE_STAGE_TURN_RIGHT;
        case ROUTE_ACTION_STRAIGHT:
            return ROUTE_STAGE_STRAIGHT_THROUGH;
        default:
            return ROUTE_STAGE_DONE;
    }
}

static uint8_t Wheel_IsRouteLocked(void) {
    return ((STATUS.sensor.vision_valid == 1U) &&
            (STATUS.state.route_id >= ROUTE_ID_A) &&
            (STATUS.state.route_id <= ROUTE_ID_D)) ? 1U : 0U;
}

static uint8_t Wheel_IsReacquired(void) {
    uint8_t bits = STATUS.sensor.gw.line_bits;

    if (STATUS.sensor.gw.lost != 0U) {
        return 0U;
    }

    if (STATUS.sensor.gw.cross != 0U) {
        return 0U;
    }

    if (bits == 0U) {
        return 0U;
    }

    return (Wheel_CountBits(bits) >= APP_ROUTE_REACQUIRE_MIN_BITS) ? 1U : 0U;
}

static uint8_t Wheel_RouteIsDone(void) {
    if (STATUS.state.route_second_action == ROUTE_ACTION_NONE) {
        return (g_route_cross_count >= 1U) ? 1U : 0U;
    }

    return (g_route_cross_count >= 2U) ? 1U : 0U;
}

static void Wheel_SetRouteStage(int32_t stage, uint32_t now_tick) {
    if (STATUS.state.route_stage == stage) {
        return;
    }

    STATUS.state.route_stage = stage;
    STATUS.state.car_run_state = stage;
    g_route_stage_tick = now_tick;

    PRINTLN("route stage=%s route=%s cross_n=%u first=%s second=%s bits=0x%02X cross=%u lost=%u",
            Wheel_StageName(stage),
            Wheel_RouteName(STATUS.state.route_id),
            (unsigned int)g_route_cross_count,
            Wheel_ActionName(STATUS.state.route_first_action),
            Wheel_ActionName(STATUS.state.route_second_action),
            STATUS.sensor.gw.line_bits,
            STATUS.sensor.gw.cross,
            STATUS.sensor.gw.lost);
}

static uint8_t Wheel_CountBits(uint8_t value) {
    uint8_t count = 0U;

    while (value != 0U) {
        if ((value & 0x01U) != 0U) {
            count++;
        }
        value >>= 1;
    }

    return count;
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
