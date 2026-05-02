#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_RX_BUF_SIZE                     64U

#define APP_MAX_PWM                         7500
#define APP_PWM_DEADBAND                    5
#define APP_PWM_FEEDFORWARD                 4000

#define APP_PID_INTEGRAL_LIMIT              10000.0f

#define APP_SERVO_CENTER_PULSE              1580
#define APP_SERVO_LOOK_RIGHT_PULSE          1600

#define APP_STATE1_PULSE_THRESHOLD          6000LL
#define APP_STATE4_PULSE_THRESHOLD          2000LL
#define APP_STATE7_PULSE_THRESHOLD          6000LL

#define APP_STARTUP_DELAY_MS                2000U
#define APP_STOP_DELAY_MS                   1000U
#define APP_TURN_SETTLE_DELAY_MS            800U
#define APP_MAIN_LOOP_DELAY_MS              10U
#define APP_LOG_PERIOD_MS                   200U
#define APP_HELLO_PERIOD_MS                 1000U

#define APP_BASE_SPEED_FAST                 8.0f
#define APP_BASE_SPEED_SLOW                 5.0f
#define APP_TURN_ANGLE_DEG                  90.0f
#define APP_TURN_LOCK_THRESHOLD_DEG         3.0f

#define APP_TURN_MAX_ROTATE                 8.0f
#define APP_TURN_MAX_DRIVE                  15.0f

#define APP_PID_LEFT_KP                     120.0f
#define APP_PID_LEFT_KI                     0.0f
#define APP_PID_LEFT_KD                     0.0f

#define APP_PID_RIGHT_KP                    120.0f
#define APP_PID_RIGHT_KI                    0.0f
#define APP_PID_RIGHT_KD                    0.0f

#define APP_PID_ANGLE_KP                    0.8f
#define APP_PID_ANGLE_KI                    0.0f
#define APP_PID_ANGLE_KD                    0.0f

#define APP_DEMO_SETPOINT                   50.0f
#define APP_DEMO_PID_KP                     1.0f
#define APP_DEMO_PID_KI                     0.2f
#define APP_DEMO_PID_KD                     0.01f

#endif /* APP_CONFIG_H */
