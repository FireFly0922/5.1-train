#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_RX_BUF_SIZE                     64U      /* 串口接收缓冲区大小，单位：字节。 */

/* 调试/整定默认值：TIM3 周期为 9999，3500 约等于 35% 占空比。 */
#define APP_MAX_PWM                         7000     /* 电机 PWM 最大输出限幅。 */
#define APP_PWM_DEADBAND                    5        /* PWM 死区，小于该值时按 0 处理。 */
#define APP_PWM_FEEDFORWARD                 2500     /* 电机启动前馈 PWM，用于克服静摩擦。 */

#define APP_MOTOR_OUTPUT_TIM3_CH12          0U       /* TIM3 CH1/CH2 motor driver input pair. */
#define APP_MOTOR_OUTPUT_TIM3_CH34          1U       /* TIM3 CH3/CH4 motor driver input pair. */
#define APP_ENCODER_TIMER_TIM2              0U       /* Encoder input on TIM2. */
#define APP_ENCODER_TIMER_TIM4              1U       /* Encoder input on TIM4. */

#define APP_MOTOR_LEFT_OUTPUT               APP_MOTOR_OUTPUT_TIM3_CH12
#define APP_MOTOR_RIGHT_OUTPUT              APP_MOTOR_OUTPUT_TIM3_CH34
#define APP_ENCODER_LEFT_TIMER              APP_ENCODER_TIMER_TIM2
#define APP_ENCODER_RIGHT_TIMER             APP_ENCODER_TIMER_TIM4

#define APP_MOTOR_LEFT_SIGN                 -1       /* 左电机方向符号，调整正反转。 */
#define APP_MOTOR_RIGHT_SIGN                1       /* 右电机方向符号，调整正反转。 */
#define APP_ENCODER_LEFT_SIGN               1        /* 左编码器计数方向符号。 */
#define APP_ENCODER_RIGHT_SIGN              -1       /* 右编码器计数方向符号。 */

#define APP_PID_INTEGRAL_LIMIT              10000.0f /* PID 积分项限幅，防止积分饱和。 */

#define APP_SERVO_CENTER_PULSE              1580     /* 舵机居中脉宽，单位：微秒。 */
#define APP_SERVO_LOOK_RIGHT_PULSE          1600     /* 舵机向右观察脉宽，单位：微秒。 */

#define APP_STATE1_PULSE_THRESHOLD          6000LL   /* 状态 1 行驶距离阈值，单位：编码器脉冲。 */
#define APP_STATE4_PULSE_THRESHOLD          2000LL   /* 状态 4 行驶距离阈值，单位：编码器脉冲。 */
#define APP_STATE7_PULSE_THRESHOLD          6000LL   /* 状态 7 行驶距离阈值，单位：编码器脉冲。 */

#define APP_STARTUP_DELAY_MS                2000U    /* 上电启动等待时间，单位：毫秒。 */
#define APP_MOTOR_ARM_DELAY_MS              2000U    /* 电机使能前等待时间，单位：毫秒。 */
#define APP_STOP_DELAY_MS                   1000U    /* 停车保持时间，单位：毫秒。 */
#define APP_TURN_SETTLE_DELAY_MS            800U     /* 转向完成后的稳定等待时间，单位：毫秒。 */
#define APP_MAIN_LOOP_DELAY_MS              10U      /* 主循环延时，单位：毫秒。 */
#define APP_LOG_PERIOD_MS                   1000U    /* 日志输出周期，单位：毫秒。 */

/* Temporary wheel-only bring-up test. Set to 0 to restore normal GW/PID logic. */
#define APP_WHEEL_TEST_ENABLE               0U
#define APP_WHEEL_TEST_PWM                  3500
#define APP_WHEEL_TEST_RUN_MS               2000U
#define APP_WHEEL_TEST_STOP_MS              1000U

#define APP_DRIVE_MODE_GW                   0U       /* Normal gray-sensor line following. */
#define APP_DRIVE_MODE_STRAIGHT_PID         1U       /* Ignore gray sensor; drive both wheels forward with PID. */
#define APP_DRIVE_MODE_STRAIGHT_OPEN_PWM    2U       /* Ignore sensors and encoders; drive both wheels with fixed PWM. */
#define APP_DRIVE_MODE                      APP_DRIVE_MODE_GW
#define APP_STRAIGHT_TEST_SPEED             10.0f
#define APP_STRAIGHT_TEST_PWM               3500

#define APP_BASE_SPEED_FAST                 25.0f     /* 快速直行基础速度。 */
#define APP_BASE_SPEED_SLOW                 8.0f     /* 慢速直行基础速度。 */
#define APP_TURN_ANGLE_DEG                  90.0f    /* 默认转向角度，单位：度。 */
#define APP_TURN_LOCK_THRESHOLD_DEG         3.0f     /* 转向锁定判定误差阈值，单位：度。 */

#define APP_GW_CALIBRATION_BLACK            12700U   /* 灰度传感器黑线校准值。 */
#define APP_GW_CALIBRATION_WHITE            45900U   /* 灰度传感器白底校准值。 */
#define APP_GW_BASE_SPEED                   APP_BASE_SPEED_SLOW /* 灰度循迹基础速度。 */
#define APP_GW_LOW_THRESHOLD                15U      /* 灰度归一化低阈值，用于判断是否压线。 */
#define APP_GW_LINE_KP                      0.8f     /* 灰度循迹 PID 比例系数。 */
#define APP_GW_LINE_KI                      0.0f     /* 灰度循迹 PID 积分系数。 */
#define APP_GW_LINE_KD                      0.0f     /* 灰度循迹 PID 微分系数。 */
#define APP_GW_MAX_CORRECTION               4.0f     /* 灰度循迹最大速度修正量。 */
#define APP_GW_STEER_SIGN                   1.0f     /* 灰度循迹转向方向符号。 */

#define APP_TURN_MAX_ROTATE                 8.0f     /* 转向时最大旋转速度。 */
#define APP_TURN_MAX_DRIVE                  15.0f    /* 转向时最大驱动速度。 */

#define APP_PID_LEFT_KP                     45.0f    /* 左轮速度 PID 比例系数。 */
#define APP_PID_LEFT_KI                     0.0f     /* 左轮速度 PID 积分系数。 */
#define APP_PID_LEFT_KD                     0.0f     /* 左轮速度 PID 微分系数。 */

#define APP_PID_RIGHT_KP                    45.0f    /* 右轮速度 PID 比例系数。 */
#define APP_PID_RIGHT_KI                    0.0f     /* 右轮速度 PID 积分系数。 */
#define APP_PID_RIGHT_KD                    0.0f     /* 右轮速度 PID 微分系数。 */

#define APP_PID_ANGLE_KP                    0.8f     /* 角度 PID 比例系数。 */
#define APP_PID_ANGLE_KI                    0.0f     /* 角度 PID 积分系数。 */
#define APP_PID_ANGLE_KD                    0.0f     /* 角度 PID 微分系数。 */

#define APP_DEMO_SETPOINT                   50.0f    /* Demo 模式目标值。 */
#define APP_DEMO_PID_KP                     1.0f     /* Demo PID 比例系数。 */
#define APP_DEMO_PID_KI                     0.2f     /* Demo PID 积分系数。 */
#define APP_DEMO_PID_KD                     0.01f    /* Demo PID 微分系数。 */

#endif /* APP_CONFIG_H */
