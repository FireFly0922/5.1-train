#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_PWM_SLEW_STEP                   9000
#define APP_PWM_STATIC_DRIVE                0

#define APP_RX_BUF_SIZE                     64U      /* 串口接收缓冲区大小，单位：字节。 */

/* 调试/整定默认值：TIM3 周期为 9999，7000 约等于 70% 占空比。 */
#define APP_MAX_PWM                         3000     /* 电机 PWM 最大输出限幅。 */
#define APP_PWM_DEADBAND                    5        /* PWM 死区，小于该值时按 0 处理。 */
#define APP_PWM_FEEDFORWARD                 0        /* 常驻前馈关闭，避免推一下后持续高速。 */

#define APP_MOTOR_DRIVER_TB6612_2PWM        1U       /* TB6612 bring-up wiring: PWMA/PWMB/STBY tied high, TIM3 pairs drive IN1/IN2. */
#define APP_MOTOR_OUTPUT_TIM3_CH12          0U       /* TIM3 CH1/CH2 motor driver input pair. */
#define APP_MOTOR_OUTPUT_TIM3_CH34          1U       /* TIM3 CH3/CH4 motor driver input pair. */
#define APP_ENCODER_TIMER_TIM2              0U       /* Encoder input on TIM2. */
#define APP_ENCODER_TIMER_TIM4              1U       /* Encoder input on TIM4. */

#define APP_MOTOR_LEFT_OUTPUT               APP_MOTOR_OUTPUT_TIM3_CH12
#define APP_MOTOR_RIGHT_OUTPUT              APP_MOTOR_OUTPUT_TIM3_CH34
#define APP_ENCODER_LEFT_TIMER              APP_ENCODER_TIMER_TIM4
#define APP_ENCODER_RIGHT_TIMER             APP_ENCODER_TIMER_TIM2

#define APP_MOTOR_LEFT_SIGN                 1        /* 左电机方向符号，调整正反转。 */
#define APP_MOTOR_RIGHT_SIGN                1       /* 右电机方向符号，调整正反转。 */
#define APP_MOTOR_LEFT_PWM_TRIM_X1000       1000U    /* Left PWM trim, 1000 means no scaling. */
#define APP_MOTOR_RIGHT_PWM_TRIM_X1000      1000U    /* Right PWM trim, 1000 means no scaling. */
#define APP_ENCODER_LEFT_SIGN               1        /* 左编码器计数方向符号。 */
#define APP_ENCODER_RIGHT_SIGN              -1       /* 右编码器计数方向符号。 */

#define APP_PID_INTEGRAL_LIMIT              2000.0f  /* PID 积分项限幅，防止积分饱和。 */

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
#define APP_WHEEL_TEST_PWM                  4200
#define APP_MOTOR_START_BOOST_PWM           6000
#define APP_MOTOR_START_BOOST_MS            150U
#define APP_WHEEL_TEST_RUN_MS               2000U
#define APP_WHEEL_TEST_STOP_MS              1000U

#define APP_DRIVE_MODE_GW                   0U       /* Normal gray-sensor line following. */
#define APP_DRIVE_MODE_STRAIGHT_PID         1U       /* Ignore gray sensor; drive both wheels forward with PID. */
#define APP_DRIVE_MODE_STRAIGHT_OPEN_PWM    2U       /* Ignore sensors and encoders; drive both wheels with fixed PWM. */
#define APP_DRIVE_MODE                      APP_DRIVE_MODE_GW
#define APP_STRAIGHT_TEST_SPEED             8.0f
#define APP_STRAIGHT_TEST_PWM               APP_WHEEL_TEST_PWM

#define APP_BASE_SPEED_FAST                 25.0f     /* 快速直行基础速度。 */
#define APP_BASE_SPEED_SLOW                 9.5f      /* 慢速直行基础速度。 */
#define APP_TURN_ANGLE_DEG                  90.0f    /* 默认转向角度，单位：度。 */
#define APP_TURN_LOCK_THRESHOLD_DEG         3.0f     /* 转向锁定判定误差阈值，单位：度。 */

#define APP_GW_CALIBRATION_BLACK            26000U   /* 灰度传感器黑线校准值。 */
#define APP_GW_CALIBRATION_WHITE            30000U   /* 灰度传感器白底校准值。 */
#define APP_GW_BASE_SPEED                   10.0f    /* 灰度循迹基础速度。 */
#define APP_GW_SPEED_RAMP_MS                0U       /* 灰度闭环寻线软启动时间，单位：毫秒。 */
#define APP_GW_LOW_THRESHOLD                70U      /* 灰度归一化低阈值，用于判断是否压线。 */
#define APP_GW_LINE_KP                      0.2f     /* 灰度循迹 PID 比例系数。 */
#define APP_GW_LINE_KI                      0.0f     /* 灰度循迹 PID 积分系数。 */
#define APP_GW_LINE_KD                      10.0f    /* 灰度循迹 PID 微分系数。 */
#define APP_GW_LINE_T_MS                    20.0f    /* 灰度循迹 PID 固定控制周期。 */
#define APP_GW_LINE_INTEGRAL_LIMIT          5.0f     /* 灰度循迹 PID 积分限幅。 */
#define APP_GW_MAX_CORRECTION               30.0f    /* 灰度循迹最大速度修正量。 */
#define APP_GW_STEER_SIGN                   1.0f     /* 灰度循迹转向方向符号。 */
#define APP_GW_STOP_ON_CROSS                0U       /* Set to 1 to stop on cross/full-line detection. */
#define APP_GW_OPEN_PWM                     5500     /* Open-loop PWM used by 8-bit gray-sensor line following. */
#define APP_GW_OPEN_TURN_PWM                1800      /*PW PWM reduction applied to the inner wheel on normal side trigger. */
#define APP_GW_OPEN_SHARP_TURN_PWM          700      /* M reduction applied when the outermost sensor is triggered. */
#define APP_GW_MIN_FORWARD_PWM              3700     /* Minimum forward PWM kept during gray-sensor steering. */

#define APP_ROUTE_ENABLE                    1U
#define APP_MAIXCAM_ENABLE                  0U
#define APP_ROUTE_FIXED_ENABLE              1U
#define APP_ROUTE_FIXED_CLASS_ID            3U
#define APP_ROUTE_CROSS_DEBOUNCE_MS         80U
#define APP_ROUTE_CROSS_EXIT_MS             120U
#define APP_ROUTE_STRAIGHT_MS               350U
#define APP_ROUTE_TURN_MIN_MS               450U
#define APP_ROUTE_TURN_TIMEOUT_MS           1800U
#define APP_ROUTE_TURN_LEFT_PWM             2200
#define APP_ROUTE_TURN_RIGHT_PWM            2200
#define APP_ROUTE_REACQUIRE_MIN_BITS        1U

#define APP_TURN_MAX_ROTATE                 10.0f     /* 转向时最大旋转速度。 */
#define APP_TURN_MAX_DRIVE                  18.0f    /* 转向时最大驱动速度。 */

#define APP_WHEEL_PID_T_MS                  6.0f     /* 轮速 PID 固定控制周期。 */

#define APP_PID_LEFT_KP                     1.0f     /* 左轮速度 PID 比例系数。 */
#define APP_PID_LEFT_KI                     1.0f     /* 左轮速度 PID 积分系数。 */
#define APP_PID_LEFT_KD                     1.0f     /* 左轮速度 PID 微分系数。 */

#define APP_PID_RIGHT_KP                    1.0f     /* 右轮速度 PID 比例系数。 */
#define APP_PID_RIGHT_KI                    1.0f     /* 右轮速度 PID 积分系数。 */
#define APP_PID_RIGHT_KD                    1.0f     /* 右轮速度 PID 微分系数。 */

#define APP_PID_ANGLE_KP                    0.8f     /* 角度 PID 比例系数。 */
#define APP_PID_ANGLE_KI                    0.0f     /* 角度 PID 积分系数。 */
#define APP_PID_ANGLE_KD                    0.0f     /* 角度 PID 微分系数。 */

#define APP_DEMO_SETPOINT                   50.0f    /* Demo 模式目标值。 */
#define APP_DEMO_PID_KP                     1.0f     /* Demo PID 比例系数。 */
#define APP_DEMO_PID_KI                     0.2f     /* Demo PID 积分系数。 */
#define APP_DEMO_PID_KD                     0.01f    /* Demo PID 微分系数。 */

/* Motor bring-up overrides: keep the reference line-following logic, but scale
 * wheel PID output to the TIM3 PWM range so the motors can overcome stiction.
 */
#undef APP_PWM_STATIC_DRIVE
#define APP_PWM_STATIC_DRIVE                2000

#undef APP_MAX_PWM
#define APP_MAX_PWM                         5000

#undef APP_PID_INTEGRAL_LIMIT
#define APP_PID_INTEGRAL_LIMIT              3000.0f

#undef APP_WHEEL_TEST_PWM
#define APP_WHEEL_TEST_PWM                  3500

#undef APP_MOTOR_START_BOOST_PWM
#define APP_MOTOR_START_BOOST_PWM           6500

#undef APP_GW_OPEN_PWM
#define APP_GW_OPEN_PWM                     3800

#undef APP_GW_BASE_SPEED
#define APP_GW_BASE_SPEED                   9.0f

#undef APP_GW_LINE_KP
#define APP_GW_LINE_KP                      0.55f

#undef APP_GW_LINE_KD
#define APP_GW_LINE_KD                      16.0f

#undef APP_GW_MAX_CORRECTION
#define APP_GW_MAX_CORRECTION               24.0f

#undef APP_ROUTE_TURN_LEFT_PWM
#define APP_ROUTE_TURN_LEFT_PWM             4500

#undef APP_ROUTE_TURN_RIGHT_PWM
#define APP_ROUTE_TURN_RIGHT_PWM            4500

#undef APP_ROUTE_STRAIGHT_MS
#define APP_ROUTE_STRAIGHT_MS               600U

#undef APP_ROUTE_CROSS_EXIT_MS
#define APP_ROUTE_CROSS_EXIT_MS             250U

#undef APP_PID_LEFT_KP
#define APP_PID_LEFT_KP                     55.0f

#undef APP_PID_LEFT_KI
#define APP_PID_LEFT_KI                     1.0f

#undef APP_PID_LEFT_KD
#define APP_PID_LEFT_KD                     0.0f

#undef APP_PID_RIGHT_KP
#define APP_PID_RIGHT_KP                    55.0f

#undef APP_PID_RIGHT_KI
#define APP_PID_RIGHT_KI                    1.0f

#undef APP_PID_RIGHT_KD
#define APP_PID_RIGHT_KD                    0.0f

#endif /* APP_CONFIG_H */
