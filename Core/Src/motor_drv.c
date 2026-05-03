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

static int32_t Motor_ApplySign(int32_t value, int32_t sign) {
    return (sign < 0) ? -value : value;
}

static uint32_t Motor_OutputFromIndex(uint32_t index) {
    if (index == MOTOR_LEFT_INDEX) {
        return APP_MOTOR_LEFT_OUTPUT;
    }

    return APP_MOTOR_RIGHT_OUTPUT;
}

static int32_t Motor_SignFromIndex(uint32_t index) {
    if (index == MOTOR_LEFT_INDEX) {
        return APP_MOTOR_LEFT_SIGN;
    }

    return APP_MOTOR_RIGHT_SIGN;
}

static uint32_t Motor_PwmTrimFromIndex(uint32_t index) {
    if (index == MOTOR_LEFT_INDEX) {
        return APP_MOTOR_LEFT_PWM_TRIM_X1000;
    }

    return APP_MOTOR_RIGHT_PWM_TRIM_X1000;
}

static int32_t Motor_ApplyPwmTrim(int32_t value, uint32_t trim_x1000) {
    int32_t sign = 1;
    int32_t scaled;

    if (value < 0) {
        sign = -1;
        value = -value;
    }

    scaled = (int32_t)(((int64_t)value * (int64_t)trim_x1000 + 500LL) / 1000LL);
    return sign * scaled;
}

static uint32_t Motor_EncoderFromIndex(uint32_t index) {
    if (index == MOTOR_LEFT_INDEX) {
        return APP_ENCODER_LEFT_TIMER;
    }

    return APP_ENCODER_RIGHT_TIMER;
}

static int32_t Motor_EncoderSignFromIndex(uint32_t index) {
    if (index == MOTOR_LEFT_INDEX) {
        return APP_ENCODER_LEFT_SIGN;
    }

    return APP_ENCODER_RIGHT_SIGN;
}

static void Motor_WriteOutput(uint32_t output, int32_t pwm_val) {
    uint32_t forward_channel;
    uint32_t reverse_channel;

    if (output == APP_MOTOR_OUTPUT_TIM3_CH34) {
        forward_channel = TIM_CHANNEL_3;
        reverse_channel = TIM_CHANNEL_4;
    } else {
        forward_channel = TIM_CHANNEL_1;
        reverse_channel = TIM_CHANNEL_2;
    }

    if (pwm_val >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, forward_channel, (uint32_t)pwm_val);
        __HAL_TIM_SET_COMPARE(&htim3, reverse_channel, 0U);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, forward_channel, 0U);
        __HAL_TIM_SET_COMPARE(&htim3, reverse_channel, (uint32_t)(-pwm_val));
    }
}

static int32_t Motor_ReadEncoder(uint32_t encoder_timer) {
    int32_t speed;

    if (encoder_timer == APP_ENCODER_TIMER_TIM4) {
        speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        __HAL_TIM_SET_COUNTER(&htim4, 0U);
    } else {
        speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0U);
    }

    return speed;
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
    int32_t logical_pwm;

    logical_pwm = Motor_ClampPwm(pwm_val);
    Motor_SetRawPwm(motor_id, logical_pwm);
}

void Motor_SetRawPwm(uint8_t motor_id, int32_t pwm_val) {
    uint32_t index = Motor_IndexFromId(motor_id);
    uint32_t output;
    int32_t hardware_pwm;

    if (index >= STATUS_WHEEL_COUNT) {
        return;
    }

    output = Motor_OutputFromIndex(index);
    pwm_val = Motor_ApplyPwmTrim(pwm_val, Motor_PwmTrimFromIndex(index));
    if (pwm_val > APP_MAX_PWM) {
        pwm_val = APP_MAX_PWM;
    } else if (pwm_val < -APP_MAX_PWM) {
        pwm_val = -APP_MAX_PWM;
    }

    hardware_pwm = Motor_ApplySign(pwm_val, Motor_SignFromIndex(index));
    STATUS.motor.wheel[index].pwm_duty = pwm_val;
    Motor_WriteOutput(output, hardware_pwm);
}

int32_t Motor_ReadSpeed(uint8_t motor_id) {
    uint32_t index = Motor_IndexFromId(motor_id);
    uint32_t encoder_timer;
    int32_t speed;

    if (index >= STATUS_WHEEL_COUNT) {
        return 0;
    }

    encoder_timer = Motor_EncoderFromIndex(index);
    speed = Motor_ReadEncoder(encoder_timer);
    speed = Motor_ApplySign(speed, Motor_EncoderSignFromIndex(index));

    STATUS.motor.wheel[index].current_speed = (float)speed;

    return speed;
}
