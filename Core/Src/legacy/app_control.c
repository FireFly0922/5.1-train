#include "app_control.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "motor_drv.h"
#include "pid_ctrl.h"
#include "protocol_parser.h"
#include "tim.h"
#include "usart.h"

static AppControlContext g_app_ctx;

/* 灏嗚搴︾害鏉熷埌 [-180, 180]锛屼笌 IMU 杈撳嚭鍧愭爣绯讳繚鎸佷竴鑷淬€?*/
static float normalize_angle(float angle)
{
    if (angle > 180.0f) {
        angle -= 360.0f;
    } else if (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float calc_angle_error(float target, float current)
{
    float err;
    err = target - current;
    if (err > 180.0f) {
        err -= 360.0f;
    } else if (err < -180.0f) {
        err += 360.0f;
    }
    return err;
}

static void reset_drive_pid(void)
{
    PID_Reset(&g_app_ctx.pid_left);
    PID_Reset(&g_app_ctx.pid_right);
}

static void reset_all_pid(void)
{
    reset_drive_pid();
    PID_Reset(&g_app_ctx.pid_angle);
}

static void advance_turn_target(void)
{
    g_app_ctx.target_yaw += APP_TURN_ANGLE_DEG;
    g_app_ctx.target_yaw = normalize_angle(g_app_ctx.target_yaw);
}

static void setup_pid_params(void)
{
    g_app_ctx.pid_left.Kp = APP_PID_LEFT_KP;
    g_app_ctx.pid_left.Ki = APP_PID_LEFT_KI;
    g_app_ctx.pid_left.Kd = APP_PID_LEFT_KD;
    PID_Reset(&g_app_ctx.pid_left);

    g_app_ctx.pid_right.Kp = APP_PID_RIGHT_KP;
    g_app_ctx.pid_right.Ki = APP_PID_RIGHT_KI;
    g_app_ctx.pid_right.Kd = APP_PID_RIGHT_KD;
    PID_Reset(&g_app_ctx.pid_right);

    g_app_ctx.pid_angle.Kp = APP_PID_ANGLE_KP;
    g_app_ctx.pid_angle.Ki = APP_PID_ANGLE_KI;
    g_app_ctx.pid_angle.Kd = APP_PID_ANGLE_KD;
    PID_Reset(&g_app_ctx.pid_angle);
}

/* 鍚姩杩愯鏃朵緷璧栫殑澶栬锛歅WM銆佺紪鐮佸櫒銆佷覆鍙?DMA銆佷簯鍙?PWM銆?*/
static void start_runtime_peripherals(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_app_ctx.rx1_buffer, APP_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_app_ctx.rx3_buffer, APP_RX_BUF_SIZE);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void AppControl_Init(void)
{
    memset(&g_app_ctx, 0, sizeof(g_app_ctx));
    setup_pid_params();

    start_runtime_peripherals();

    HAL_Delay(APP_STARTUP_DELAY_MS);

    g_app_ctx.target_yaw = g_app_ctx.current_yaw;
    reset_all_pid();

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, APP_SERVO_CENTER);
    g_app_ctx.run_state = APP_STATE_RUN_1;

    HAL_TIM_Base_Start_IT(&htim6);
}

void AppControl_Step(void)
{
    float yaw_diff;

    switch (g_app_ctx.run_state) {
    case APP_STATE_RUN_1:
        g_app_ctx.base_speed = APP_BASE_SPEED_FAST;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, APP_SERVO_LOOK_RIGHT);
        if (g_app_ctx.total_pulses > APP_STATE1_PULSE_THRESHOLD) {
            reset_drive_pid();
            g_app_ctx.run_state = APP_STATE_STOP_1;
        }
        break;

    case APP_STATE_STOP_1:
        g_app_ctx.base_speed = 0.0f;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, APP_SERVO_CENTER);
        HAL_Delay(APP_STOP_DELAY_MS);
        advance_turn_target();
        reset_all_pid();
        g_app_ctx.run_state = APP_STATE_TURN_1;
        break;

    case APP_STATE_TURN_1:
        g_app_ctx.base_speed = 0.0f;
        yaw_diff = calc_angle_error(g_app_ctx.target_yaw, g_app_ctx.current_yaw);
        if (fabsf(yaw_diff) < APP_TURN_LOCK_THRESHOLD_DEG) {
            HAL_Delay(APP_TURN_SETTLE_DELAY_MS);
            reset_all_pid();
            g_app_ctx.total_pulses = 0;
            g_app_ctx.run_state = APP_STATE_RUN_2;
        }
        break;

    case APP_STATE_RUN_2:
        g_app_ctx.base_speed = APP_BASE_SPEED_SLOW;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, APP_SERVO_CENTER);
        if (g_app_ctx.total_pulses > APP_STATE4_PULSE_THRESHOLD) {
            reset_drive_pid();
            g_app_ctx.run_state = APP_STATE_STOP_2;
        }
        break;

    case APP_STATE_STOP_2:
        g_app_ctx.base_speed = 0.0f;
        HAL_Delay(APP_STOP_DELAY_MS);
        advance_turn_target();
        reset_all_pid();
        g_app_ctx.run_state = APP_STATE_TURN_2;
        break;

    case APP_STATE_TURN_2:
        g_app_ctx.base_speed = 0.0f;
        yaw_diff = calc_angle_error(g_app_ctx.target_yaw, g_app_ctx.current_yaw);
        if (fabsf(yaw_diff) < APP_TURN_LOCK_THRESHOLD_DEG) {
            HAL_Delay(APP_TURN_SETTLE_DELAY_MS);
            reset_all_pid();
            g_app_ctx.total_pulses = 0;
            g_app_ctx.run_state = APP_STATE_RUN_3;
        }
        break;

    case APP_STATE_RUN_3:
        g_app_ctx.base_speed = APP_BASE_SPEED_FAST;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, APP_SERVO_LOOK_RIGHT);
        if (g_app_ctx.total_pulses > APP_STATE7_PULSE_THRESHOLD) {
            reset_drive_pid();
            g_app_ctx.run_state = APP_STATE_FINISH;
        }
        break;

    case APP_STATE_FINISH:
        g_app_ctx.base_speed = 0.0f;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, APP_SERVO_CENTER);
        break;

    default:
        g_app_ctx.base_speed = 0.0f;
        g_app_ctx.run_state = APP_STATE_IDLE;
        break;
    }
}

void AppControl_On10msTick(void)
{
    float angle_err;
    float turn_adjust;
    float max_turn;
    float target_l;
    float target_r;
    float pwm_l;
    float pwm_r;
    int speed_left;
    int speed_right;

    speed_left = Motor_ReadSpeed(&htim2);
    speed_right = -Motor_ReadSpeed(&htim4);
    g_app_ctx.total_pulses += (speed_left + speed_right) / 2;

    angle_err = calc_angle_error(g_app_ctx.target_yaw, g_app_ctx.current_yaw);

    if ((g_app_ctx.run_state == APP_STATE_TURN_1) || (g_app_ctx.run_state == APP_STATE_TURN_2)) {
        g_app_ctx.pid_angle.Kp = 1.0f;
        g_app_ctx.pid_angle.Kd = 0.8f;
        max_turn = APP_TURN_MAX_ROTATE;
    } else {
        g_app_ctx.pid_angle.Kp = 1.0f;
        g_app_ctx.pid_angle.Kd = 0.5f;
        max_turn = APP_TURN_MAX_DRIVE;
    }

    turn_adjust = PID_Calc(&g_app_ctx.pid_angle, 0.0f, -angle_err);
    if (turn_adjust > max_turn) {
        turn_adjust = max_turn;
    } else if (turn_adjust < -max_turn) {
        turn_adjust = -max_turn;
    }

    target_l = g_app_ctx.base_speed - turn_adjust;
    target_r = g_app_ctx.base_speed + turn_adjust;

    pwm_l = PID_Calc(&g_app_ctx.pid_left, target_l, (float)speed_left);
    pwm_r = PID_Calc(&g_app_ctx.pid_right, target_r, (float)speed_right);

    Motor_SetSpeed(1U, (int)pwm_l);
    Motor_SetSpeed(2U, (int)pwm_r);
}

void AppControl_OnUartRx(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == 0) {
        return;
    }

    if (huart->Instance == USART1) {
        Protocol_ParseUartData(&g_app_ctx, huart, g_app_ctx.rx1_buffer, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_app_ctx.rx1_buffer, APP_RX_BUF_SIZE);
    } else if (huart->Instance == USART3) {
        Protocol_ParseUartData(&g_app_ctx, huart, g_app_ctx.rx3_buffer, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_app_ctx.rx3_buffer, APP_RX_BUF_SIZE);
    }
}

void AppControl_OnUartError(UART_HandleTypeDef *huart)
{
    if (huart == 0) {
        return;
    }

    if (huart->Instance == USART1) {
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_app_ctx.rx1_buffer, APP_RX_BUF_SIZE);
    } else if (huart->Instance == USART3) {
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_app_ctx.rx3_buffer, APP_RX_BUF_SIZE);
    }
}

AppControlContext *AppControl_GetContext(void)
{
    return &g_app_ctx;
}

uint8_t *AppControl_GetUart1Buffer(void)
{
    return g_app_ctx.rx1_buffer;
}

uint8_t *AppControl_GetUart3Buffer(void)
{
    return g_app_ctx.rx3_buffer;
}
