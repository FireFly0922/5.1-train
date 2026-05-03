#ifndef GW_H
#define GW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

#ifndef HAL_ADC_MODULE_ENABLED
typedef struct __ADC_HandleTypeDef ADC_HandleTypeDef;
#endif

#define GW_CHANNELS              8u      /* 灰度传感器通道数量。 */
#define GW_DEFAULT_SAMPLE_TIMES  8u      /* 默认每个通道重复采样次数。 */
#define GW_DEFAULT_LOW_THRESHOLD 40u     /* 默认低阈值，低于该值认为检测到黑线。 */
#define GW_DEFAULT_HIGH_VALUE    100u    /* 默认归一化满量程值。 */
#define GW_DEFAULT_ADC_TIMEOUT   10u     /* 默认 ADC 转换超时时间，单位：毫秒。 */

typedef enum
{
    GW_OK = 0,                  /* 操作成功。 */
    GW_ERROR_NULL,              /* 传入了空指针。 */
    GW_ERROR_NOT_CONFIGURED,    /* 模块尚未完成配置或配置无效。 */
    GW_ERROR_ADC                /* ADC 配置、启动、采样或停止失败。 */
} GW_Status;

typedef enum
{
    GW_DIR_NORMAL = 0,  /* 正常通道方向。 */
    GW_DIR_REVERSE = 1  /* 反向通道方向，用于左右安装方向相反的情况。 */
} GW_Direction;

typedef struct
{
    GPIO_TypeDef *port; /* 地址选择 GPIO 端口。 */
    uint16_t pin;       /* 地址选择 GPIO 引脚。 */
} GW_GpioPin;

typedef struct
{
    ADC_HandleTypeDef *hadc;        /* 灰度传感器使用的 ADC 句柄。 */
    uint32_t adc_channel;           /* ADC 通道号。 */
    uint32_t adc_rank;              /* ADC 规则组转换序号。 */
    uint32_t adc_sampling_time;     /* ADC 采样时间。 */
    uint32_t adc_single_diff;       /* ADC 单端/差分输入模式。 */
    uint32_t adc_timeout_ms;        /* ADC 转换超时时间，单位：毫秒。 */
    GW_GpioPin address[3];          /* 三位地址选择引脚，用于切换 8 路灰度通道。 */
} GW_Config;

typedef struct
{
    uint16_t analog[GW_CHANNELS];               /* 各通道原始 ADC 采样值。 */
    uint16_t normalized[GW_CHANNELS];           /* 各通道归一化值。 */
    uint16_t calibrated_white[GW_CHANNELS];     /* 校准得到的白底 ADC 值。 */
    uint16_t calibrated_black[GW_CHANNELS];     /* 校准得到的黑线 ADC 值。 */
    uint16_t gray_white[GW_CHANNELS];           /* 当前使用的白底参考值。 */
    uint16_t gray_black[GW_CHANNELS];           /* 当前使用的黑线参考值。 */

    int8_t weights[GW_CHANNELS];    /* 各通道位置权重，用于计算线位置。 */
    uint8_t line_bits;              /* 每个 bit 表示对应通道是否检测到线。 */
    uint8_t sample_times;           /* 每个通道重复采样次数。 */
    uint8_t low_threshold;          /* 低阈值，低于该值认为检测到黑线。 */
    uint8_t high_value;             /* 归一化满量程值。 */
    uint8_t calibrated;             /* 是否已完成黑白校准。 */
    uint8_t configured;             /* 是否已完成硬件配置。 */
    GW_Direction direction;         /* 通道方向设置。 */

    float line_position;    /* 计算得到的线位置偏差。 */
    uint8_t normal;         /* 正常循迹标志。 */
    uint8_t lost;           /* 丢线标志。 */
    uint8_t cross;          /* 十字/全线标志。 */

    GW_Config config;       /* 当前生效的硬件配置。 */
} GW_Sensor;

void GW_Init(GW_Sensor *sensor);
GW_Status GW_InitConfig(GW_Sensor *sensor, const GW_Config *config);
GW_Status GW_InitCalibrated(GW_Sensor *sensor,
                            const GW_Config *config,
                            const uint16_t white[GW_CHANNELS],
                            const uint16_t black[GW_CHANNELS]);

GW_Status GW_SetConfig(GW_Sensor *sensor, const GW_Config *config);
void GW_SetDirection(GW_Sensor *sensor, GW_Direction direction);
void GW_SetThreshold(GW_Sensor *sensor, uint8_t low_threshold);
void GW_SetSampleTimes(GW_Sensor *sensor, uint8_t sample_times);

GW_Status GW_ReadAnalog(GW_Sensor *sensor);
GW_Status GW_Update(GW_Sensor *sensor);
GW_Status GW_ProcessNormalized(GW_Sensor *sensor,
                               const uint16_t normalized[GW_CHANNELS]);

uint8_t GW_GetLineBits(const GW_Sensor *sensor);
float GW_GetPosition(const GW_Sensor *sensor);
uint8_t GW_IsNormal(const GW_Sensor *sensor);
uint8_t GW_IsLost(const GW_Sensor *sensor);
uint8_t GW_IsCross(const GW_Sensor *sensor);

void GW_CopyAnalog(const GW_Sensor *sensor, uint16_t out[GW_CHANNELS]);
void GW_CopyNormalized(const GW_Sensor *sensor, uint16_t out[GW_CHANNELS]);

#ifdef __cplusplus
}
#endif

#endif /* GW_H */
