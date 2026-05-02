#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 涓插彛 DMA 缂撳啿鍖哄ぇ灏?*/
#define APP_RX_BUF_SIZE                     64U

/* 鐢垫満杈撳嚭鍙傛暟 */
#define APP_MAX_PWM                         10000
#define APP_PWM_DEADBAND                    5
#define APP_PWM_FEEDFORWARD                 2500

/* PID 鍙傛暟闄愬埗 */
#define APP_PID_INTEGRAL_LIMIT              10000.0f

/* 鑸垫満瑙掑害 PWM */
#define APP_SERVO_CENTER                    1580
#define APP_SERVO_LOOK_RIGHT                1600

/* 鐘舵€侀槇鍊间笌鏃跺簭 */
#define APP_STATE1_PULSE_THRESHOLD          6000LL
#define APP_STATE4_PULSE_THRESHOLD          2000LL
#define APP_STATE7_PULSE_THRESHOLD          6000LL

#define APP_STARTUP_DELAY_MS                2000U
#define APP_STOP_DELAY_MS                   1000U
#define APP_TURN_SETTLE_DELAY_MS            800U
#define APP_MAIN_LOOP_DELAY_MS              10U

/* 杩愬姩鍙傛暟 */
#define APP_BASE_SPEED_FAST                 10.0f
#define APP_BASE_SPEED_SLOW                 8.0f
#define APP_TURN_ANGLE_DEG                  90.0f
#define APP_TURN_LOCK_THRESHOLD_DEG         3.0f

/* 杞悜 PID 闄愬箙 */
#define APP_TURN_MAX_ROTATE                 8.0f
#define APP_TURN_MAX_DRIVE                  15.0f

/* PID 鍒濆鍙傛暟 */
#define APP_PID_LEFT_KP                     150.0f
#define APP_PID_LEFT_KI                     10.0f
#define APP_PID_LEFT_KD                     0.0f

#define APP_PID_RIGHT_KP                    150.0f
#define APP_PID_RIGHT_KI                    10.0f
#define APP_PID_RIGHT_KD                    0.0f

#define APP_PID_ANGLE_KP                    0.8f
#define APP_PID_ANGLE_KI                    0.0f
#define APP_PID_ANGLE_KD                    0.0f

#endif /* APP_CONFIG_H */
