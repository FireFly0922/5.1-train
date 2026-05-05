#ifndef STATUS_H
#define STATUS_H

/* Route state used by MaixCam ABCD parsing. */
typedef enum {
    ROUTE_ID_NONE = -1,
    ROUTE_ID_A = 0,
    ROUTE_ID_B = 1,
    ROUTE_ID_C = 2,
    ROUTE_ID_D = 3,
} RouteId_t;

typedef enum {
    ROUTE_ACTION_NONE = 0,
    ROUTE_ACTION_LEFT,
    ROUTE_ACTION_RIGHT,
    ROUTE_ACTION_STRAIGHT,
} RouteAction_t;

typedef enum {
    ROUTE_STAGE_WAIT_VISION = 0,
    ROUTE_STAGE_ROUTE_LOCKED,
    ROUTE_STAGE_FIRST_CROSS_LEFT,
    ROUTE_STAGE_FIRST_CROSS_RIGHT,
    ROUTE_STAGE_FIRST_CROSS_STRAIGHT,
    ROUTE_STAGE_SECOND_T_LEFT,
    ROUTE_STAGE_SECOND_T_RIGHT,
    ROUTE_STAGE_DONE,
    ROUTE_STAGE_INVALID,
} RouteStage_t;

#include <stdint.h>

#include "main.h"
#include "pid.h"

#define STATUS_WHEEL_COUNT 2U       /* 轮子数量。 */
#define STATUS_SERVO_COUNT 2U       /* 舵机数量。 */
#define STATUS_GW_CHANNEL_COUNT 8U  /* 灰度传感器通道数量。 */

typedef struct {
    PID_t pid;              /* 轮速闭环 PID 控制器。 */
    float current_speed;    /* 当前轮速反馈值。 */
    float target_speed;     /* 目标轮速。 */
    int32_t pwm_duty;       /* 当前输出 PWM 占空比/比较值。 */
} Wheel_t;

typedef struct {
    float current_angle;        /* 舵机当前角度，单位：度。 */
    float target_angle;         /* 舵机目标角度，单位：度。 */
    uint16_t current_pulse;     /* 舵机当前控制脉宽，单位：微秒。 */
    uint16_t target_pulse;      /* 舵机目标控制脉宽，单位：微秒。 */
} Servo_t;

typedef struct {
    float yaw;      /* 航向角，绕 Z 轴，单位：度。 */
    float pitch;    /* 俯仰角，绕 Y 轴，单位：度。 */
    float roll;     /* 横滚角，绕 X 轴，单位：度。 */
} Gyro_t;

typedef struct {
    uint16_t analog[STATUS_GW_CHANNEL_COUNT];        /* 灰度原始 ADC 采样值。 */
    uint16_t normalized[STATUS_GW_CHANNEL_COUNT];    /* 灰度归一化值。 */
    uint8_t line_bits;                               /* 每个 bit 表示对应灰度通道是否检测到线。 */
    float line_position;                             /* 计算得到的线位置偏差。 */
    uint8_t normal;                                  /* 正常循迹标志。 */
    uint8_t lost;                                    /* 丢线标志。 */
    uint8_t cross;                                   /* 十字/全线标志。 */
    int32_t last_status;                             /* 最近一次灰度模块返回状态。 */
} Gw_t;

typedef struct {
    uint8_t is_on;                  /* LED 当前是否点亮。 */
    uint8_t is_bound;               /* LED 是否已绑定到实际 GPIO。 */
    GPIO_TypeDef *port;             /* LED 所在 GPIO 端口。 */
    uint16_t pin;                   /* LED 所在 GPIO 引脚。 */
    GPIO_PinState active_state;     /* LED 点亮时对应的 GPIO 电平。 */
} Led_t;

typedef struct {
    Wheel_t wheel[STATUS_WHEEL_COUNT];   /* 轮组运行状态。 */
    Servo_t servo[STATUS_SERVO_COUNT];   /* 舵机运行状态。 */
} Motor_t;

typedef struct {
    Gyro_t gyro;             /* 陀螺仪/姿态状态。 */
    Gw_t gw;                 /* 灰度传感器状态。 */
    float vision_target;     /* 视觉目标位置或偏差。 */
    uint8_t vision_class_id;
    uint8_t vision_prob;
    uint8_t vision_valid;
} Sensor_t;

typedef struct {
    Led_t led;   /* 板载或外接 LED 状态。 */
} Device_t;

typedef struct {
    int32_t main_mode;       /* 主模式编号。 */
    int32_t sub_mode;        /* 子模式编号。 */
    int32_t car_run_state;   /* 小车运行状态编号。 */
    int32_t route_id;
    int32_t route_stage;
    int32_t route_first_action;
    int32_t route_second_action;
} State_t;

typedef struct {
    float demo_setpoint;         /* Demo 模式目标值。 */
    float measurement;           /* 当前测量/反馈值。 */
    float control_out;           /* 当前控制输出值。 */
    int64_t total_pulses;        /* 累计编码器脉冲数。 */
    uint32_t last_tick;          /* 上一次主循环时间戳，单位：毫秒。 */
    uint32_t last_log_tick;      /* 上一次日志输出时间戳，单位：毫秒。 */
} Runtime_t;

typedef struct {
    Motor_t motor;         /* 电机与舵机状态集合。 */
    Sensor_t sensor;       /* 传感器状态集合。 */
    Device_t device;       /* 外设状态集合。 */
    State_t state;         /* 模式与状态机变量。 */
    Runtime_t runtime;     /* 运行时临时变量和时间戳。 */
} Status_t;

extern Status_t STATUS;    /* 全局系统状态实例。 */

#endif
