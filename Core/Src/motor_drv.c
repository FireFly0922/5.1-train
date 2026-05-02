#include "motor_drv.h"

#include "app_config.h"
#include "status.h"
#include "tim.h"

enum {
    MOTOR_LEFT_ID = 1U,
    MOTOR_RIGHT_ID = 2U,
    MOTOR_LEFT_INDEX = 0U,
    MOTOR_RIGHT_INDEX = 1U,
};

static uint32_t Motor_IndexFromId(uint8_t motor_id) {
    if (motor_id == MOTOR_LEFT_ID) {
        return MOTOR_LEFT_INDEX;
    }

    if (motor_id == MOTOR_RIGHT_ID) {
        return MOTOR_RIGHT_INDEX;
    }

    if (motor_id < STATUS_WHEEL_COUNT) {
        return (uint32_t)motor_id;
    }

    return STATUS_WHEEL_COUNT;
}

static int32_t Motor_ClampPwm(int32_t pwm_val) {
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

    return pwm_val;
}

void Motor_Init(void) {
    uint32_t index;

    for (index = 0U; index < STATUS_WHEEL_COUNT; ++index) {
        STATUS.motor.wheel[index].current_speed = 0.0f;
        STATUS.motor.wheel[index].target_speed = 0.0f;
        STATUS.motor.wheel[index].pwm_duty = 0;
    }

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
}

void Motor_SetSpeed(uint8_t motor_id, int32_t pwm_val) {
    uint32_t index = Motor_IndexFromId(motor_id);

    if (index >= STATUS_WHEEL_COUNT) {
        return;
    }

    pwm_val = Motor_ClampPwm(pwm_val);
    STATUS.motor.wheel[index].pwm_duty = pwm_val;

    if (index == MOTOR_LEFT_INDEX) {
        if (pwm_val >= 0) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pwm_val);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(-pwm_val));
        }
    } else if (index == MOTOR_RIGHT_INDEX) {
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

int32_t Motor_ReadSpeed(uint8_t motor_id) {
    uint32_t index = Motor_IndexFromId(motor_id);
    int32_t speed;

    if (index >= STATUS_WHEEL_COUNT) {
        return 0;
    }

    if (index == MOTOR_LEFT_INDEX) {
        speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0U);
    } else {
        speed = -(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        __HAL_TIM_SET_COUNTER(&htim4, 0U);
    }

    STATUS.motor.wheel[index].current_speed = (float)speed;

    return speed;
}
