#include "motor_drv.h"
#include "app_config.h"

void Motor_SetSpeed(uint8_t motor_id, int pwm_val)
{
    if (pwm_val > APP_PWM_DEADBAND) {
        pwm_val += APP_PWM_FEEDFORWARD;
    } else if (pwm_val < -APP_PWM_DEADBAND) {
        pwm_val -= APP_PWM_FEEDFORWARD;
    } else {
        pwm_val = 0;
    }

    if (pwm_val > APP_MAX_PWM) {
        pwm_val = APP_MAX_PWM;
    } else if (pwm_val < -APP_MAX_PWM) {
        pwm_val = -APP_MAX_PWM;
    }

    if (motor_id == 1U) {
        if (pwm_val >= 0) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pwm_val);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(-pwm_val));
        }
    } else if (motor_id == 2U) {
        pwm_val = -pwm_val;
        if (pwm_val >= 0) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)pwm_val);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(-pwm_val));
        }
    }
}

int Motor_ReadSpeed(TIM_HandleTypeDef *htim)
{
    int16_t speed;

    speed = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0U);
    return (int)speed;
}
