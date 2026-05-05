#include "GW.h"

#include <string.h>

#define GW_POS_EPSILON 0.000001f
#define GW_LINE_CORE_FIRST 2u
#define GW_LINE_CORE_LAST  5u
#define GW_DIGITAL_LOW_THRESHOLD 33u
#define GW_DIGITAL_HIGH_THRESHOLD 66u

/**
 * @brief 加载灰度传感器默认位置权重。
 * @param sensor 灰度传感器对象指针。
 */
static void GW_LoadDefaultWeights(GW_Sensor *sensor)
{
    static const int8_t default_weights[GW_CHANNELS] =
    {
        -30, -20, -15, -10, 10, 15, 20, 30
    };

    memcpy(sensor->weights, default_weights, sizeof(default_weights));
}

/**
 * @brief 检查灰度传感器配置是否有效。
 * @param config 待检查的配置结构体指针。
 * @return 1 表示配置有效，0 表示配置无效。
 */
static uint8_t GW_ConfigIsValid(const GW_Config *config)
{
    uint8_t i;

    if ((config == NULL) || (config->hadc == NULL))
    {
        return 0u;
    }

    for (i = 0u; i < 3u; i++)
    {
        if ((config->address[i].port == NULL) || (config->address[i].pin == 0u))
        {
            return 0u;
        }
    }

    return 1u;
}

/**
 * @brief 设置模拟多路复用器的地址选择引脚。
 * @param sensor 灰度传感器对象指针。
 * @param index 需要选择的通道索引，范围为 0 到 GW_CHANNELS - 1。
 */
static void GW_SetAddress(const GW_Sensor *sensor, uint8_t index)
{
    uint8_t i;

    for (i = 0u; i < 3u; i++)
    {
        GPIO_PinState state;

        state = ((index & (uint8_t)(1u << i)) != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(sensor->config.address[i].port,
                          sensor->config.address[i].pin,
                          state);
    }
}

/**
 * @brief 统计 8 位数值中置 1 的位数。
 * @param value 待统计的 8 位数值。
 * @return value 中为 1 的 bit 数量。
 */
static uint8_t GW_CountBits(uint8_t value)
{
    uint8_t count = 0u;

    while (value != 0u)
    {
        if ((value & 0x01u) != 0u)
        {
            count++;
        }
        value >>= 1;
    }

    return count;
}

/**
 * @brief 将单个 ADC 原始值按黑白标定值归一化。
 * @param raw ADC 原始采样值。
 * @param black 黑线或暗色标定值。
 * @param white 白底或亮色标定值。
 * @return 归一化后的数值，范围为 0 到 GW_DEFAULT_HIGH_VALUE。
 */
static uint16_t GW_NormalizeOne(uint16_t raw, uint16_t black, uint16_t white)
{
    uint32_t value;
    uint16_t low;
    uint16_t high;

    if (white == black)
    {
        return GW_DEFAULT_HIGH_VALUE;
    }

    if (black < white)
    {
        if (raw <= black)
        {
            return 0u;
        }

        value = ((uint32_t)(raw - black) * GW_DEFAULT_HIGH_VALUE) /
                (uint32_t)(white - black);
    }
    else
    {
        low = white;
        high = black;

        if (raw >= high)
        {
            return 0u;
        }

        if (raw <= low)
        {
            return GW_DEFAULT_HIGH_VALUE;
        }

        value = ((uint32_t)(high - raw) * GW_DEFAULT_HIGH_VALUE) /
                (uint32_t)(high - low);
    }

    if (value > GW_DEFAULT_HIGH_VALUE)
    {
        value = GW_DEFAULT_HIGH_VALUE;
    }

    return (uint16_t)value;
}

/**
 * @brief 根据归一化值更新巡线位图。
 * @param sensor 灰度传感器对象指针。
 */
static void GW_UpdateLineBits(GW_Sensor *sensor)
{
    uint8_t i;
    uint8_t bits = sensor->line_bits;
    uint8_t low_threshold = sensor->low_threshold;
    uint8_t high_threshold = GW_DIGITAL_HIGH_THRESHOLD;

    if (low_threshold > GW_DIGITAL_LOW_THRESHOLD) {
        high_threshold = (uint8_t)(low_threshold +
                                   ((sensor->high_value - low_threshold) / 2u));
    }

    for (i = 0u; i < GW_CHANNELS; i++)
    {
        if (sensor->normalized[i] <= low_threshold)
        {
            bits |= (uint8_t)(1u << i);
        }
        else if (sensor->normalized[i] >= high_threshold)
        {
            bits &= (uint8_t)~(uint8_t)(1u << i);
        }
    }

    sensor->line_bits = bits;
}

/**
 * @brief 根据各通道归一化值和权重计算当前线位置。
 * @param sensor 灰度传感器对象指针。
 */
static void GW_UpdatePosition(GW_Sensor *sensor)
{
    uint8_t i;
    float strength[GW_CHANNELS] = {0.0f};
    float total_strength = 0.0f;
    float sum_weight = 0.0f;
    uint8_t high_threshold = GW_DIGITAL_HIGH_THRESHOLD;

    if (sensor->low_threshold > GW_DIGITAL_LOW_THRESHOLD) {
        high_threshold = (uint8_t)(sensor->low_threshold +
                                   ((sensor->high_value - sensor->low_threshold) / 2u));
    }

    for (i = GW_LINE_CORE_FIRST; i <= GW_LINE_CORE_LAST; i++)
    {
        if (sensor->normalized[i] < high_threshold)
        {
            strength[i] = (float)(sensor->high_value - sensor->normalized[i]);
            total_strength += strength[i];
        }
    }

    if (total_strength > GW_POS_EPSILON)
    {
        for (i = GW_LINE_CORE_FIRST; i <= GW_LINE_CORE_LAST; i++)
        {
            sum_weight += (strength[i] / total_strength) * (float)sensor->weights[i];
        }

        sensor->line_position = sum_weight;
    }
    else
    {
        sensor->line_position = 0.0f;
    }
}

/**
 * @brief 根据巡线位图更新正常、丢线和十字状态。
 * @param sensor 灰度传感器对象指针。
 */
static void GW_UpdateState(GW_Sensor *sensor)
{
    uint8_t bit_count;

    sensor->lost = (sensor->line_bits == 0x00u) ? 1u : 0u;
    sensor->cross = 0u;

    bit_count = GW_CountBits(sensor->line_bits);
    if (bit_count >= 5u)
    {
        sensor->cross = 1u;
    }
    else if ((sensor->line_bits & 0x81u) == 0x81u)
    {
        sensor->cross = 1u;
    }
    else if (((sensor->line_bits & 0x03u) != 0u) &&
             ((sensor->line_bits & 0x3Cu) != 0u))
    {
        sensor->cross = 1u;
    }
    else if (((sensor->line_bits & 0xC0u) != 0u) &&
             ((sensor->line_bits & 0x3Cu) != 0u))
    {
        sensor->cross = 1u;
    }

    sensor->normal = ((sensor->lost == 0u) && (sensor->cross == 0u)) ? 1u : 0u;
}

/**
 * @brief 读取一次 ADC 采样值。
 * @param config 灰度传感器配置结构体指针，包含 ADC 通道、采样时间和超时等配置。
 * @param value 用于保存 ADC 采样结果的输出指针。
 * @return GW_OK 表示读取成功，其他值表示读取失败原因。
 */
static GW_Status GW_ReadAdcOnce(const GW_Config *config, uint16_t *value)
{
#ifdef HAL_ADC_MODULE_ENABLED
    ADC_ChannelConfTypeDef adc_config = {0};
    uint32_t timeout;

    adc_config.Channel = config->adc_channel;
    adc_config.Rank = config->adc_rank;
    adc_config.SamplingTime = config->adc_sampling_time;
    adc_config.SingleDiff = config->adc_single_diff;
    adc_config.OffsetNumber = ADC_OFFSET_NONE;
    adc_config.Offset = 0;

#ifdef ADC_OFFSET_SIGNED_SATURATION_DISABLE
    adc_config.OffsetSignedSaturation = ADC_OFFSET_SIGNED_SATURATION_DISABLE;
#endif

    timeout = (config->adc_timeout_ms == 0u) ? GW_DEFAULT_ADC_TIMEOUT :
              config->adc_timeout_ms;

    if (HAL_ADC_ConfigChannel(config->hadc, &adc_config) != HAL_OK)
    {
        return GW_ERROR_ADC;
    }

    if (HAL_ADC_Start(config->hadc) != HAL_OK)
    {
        return GW_ERROR_ADC;
    }

    if (HAL_ADC_PollForConversion(config->hadc, timeout) != HAL_OK)
    {
        (void)HAL_ADC_Stop(config->hadc);
        return GW_ERROR_ADC;
    }

    *value = (uint16_t)HAL_ADC_GetValue(config->hadc);

    if (HAL_ADC_Stop(config->hadc) != HAL_OK)
    {
        return GW_ERROR_ADC;
    }

    return GW_OK;
#else
    (void)config;
    (void)value;
    return GW_ERROR_ADC;
#endif
}

/**
 * @brief 初始化灰度传感器对象，并填入默认参数。
 * @param sensor 灰度传感器对象指针。
 */
void GW_Init(GW_Sensor *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    memset(sensor, 0, sizeof(*sensor));
    GW_LoadDefaultWeights(sensor);

    sensor->sample_times = GW_DEFAULT_SAMPLE_TIMES;
    sensor->low_threshold = GW_DEFAULT_LOW_THRESHOLD;
    sensor->high_value = GW_DEFAULT_HIGH_VALUE;
    sensor->direction = GW_DIR_REVERSE;
}

/**
 * @brief 设置灰度传感器的硬件配置。
 * @param sensor 灰度传感器对象指针。
 * @param config 灰度传感器配置结构体指针。
 * @return GW_OK 表示设置成功，其他值表示参数为空或配置无效。
 */
GW_Status GW_SetConfig(GW_Sensor *sensor, const GW_Config *config)
{
    if ((sensor == NULL) || (config == NULL))
    {
        return GW_ERROR_NULL;
    }

    if (GW_ConfigIsValid(config) == 0u)
    {
        sensor->configured = 0u;
        return GW_ERROR_NOT_CONFIGURED;
    }

    sensor->config = *config;
    if (sensor->config.adc_timeout_ms == 0u)
    {
        sensor->config.adc_timeout_ms = GW_DEFAULT_ADC_TIMEOUT;
    }
    sensor->configured = 1u;

    return GW_OK;
}

/**
 * @brief 初始化灰度传感器对象并设置硬件配置。
 * @param sensor 灰度传感器对象指针。
 * @param config 灰度传感器配置结构体指针。
 * @return GW_OK 表示初始化和配置成功，其他值表示失败原因。
 */
GW_Status GW_InitConfig(GW_Sensor *sensor, const GW_Config *config)
{
    GW_Status status;

    if (sensor == NULL)
    {
        return GW_ERROR_NULL;
    }

    GW_Init(sensor);
    status = GW_SetConfig(sensor, config);

    return status;
}

/**
 * @brief 初始化灰度传感器对象、设置硬件配置并写入黑白标定值。
 * @param sensor 灰度传感器对象指针。
 * @param config 灰度传感器配置结构体指针。
 * @param white 白底或亮色标定数组，长度为 GW_CHANNELS。
 * @param black 黑线或暗色标定数组，长度为 GW_CHANNELS。
 * @return GW_OK 表示初始化和标定成功，其他值表示失败原因。
 */
GW_Status GW_InitCalibrated(GW_Sensor *sensor,
                            const GW_Config *config,
                            const uint16_t white[GW_CHANNELS],
                            const uint16_t black[GW_CHANNELS])
{
    uint8_t i;
    uint16_t white_value;
    uint16_t black_value;
    GW_Status status;

    if ((sensor == NULL) || (white == NULL) || (black == NULL))
    {
        return GW_ERROR_NULL;
    }

    status = GW_InitConfig(sensor, config);
    if (status != GW_OK)
    {
        return status;
    }

    for (i = 0u; i < GW_CHANNELS; i++)
    {
        white_value = white[i];
        black_value = black[i];

        sensor->calibrated_white[i] = white_value;
        sensor->calibrated_black[i] = black_value;
        sensor->gray_white[i] = (uint16_t)(((uint32_t)white_value * 2u +
                                            black_value) / 3u);
        sensor->gray_black[i] = (uint16_t)(((uint32_t)white_value +
                                            (uint32_t)black_value * 2u) / 3u);
    }

    sensor->calibrated = 1u;

    return GW_OK;
}

/**
 * @brief 设置灰度传感器通道输出方向。
 * @param sensor 灰度传感器对象指针。
 * @param direction 通道方向，GW_DIR_NORMAL 表示正向，GW_DIR_REVERSE 表示反向。
 */
void GW_SetDirection(GW_Sensor *sensor, GW_Direction direction)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->direction = direction;
}

/**
 * @brief 设置判定黑线的低阈值。
 * @param sensor 灰度传感器对象指针。
 * @param low_threshold 低阈值，小于等于该值的通道会被判定为压线。
 */
void GW_SetThreshold(GW_Sensor *sensor, uint8_t low_threshold)
{
    if (sensor == NULL)
    {
        return;
    }

    if (low_threshold > sensor->high_value)
    {
        low_threshold = sensor->high_value;
    }

    sensor->low_threshold = low_threshold;
}

/**
 * @brief 设置每个通道的 ADC 平均采样次数。
 * @param sensor 灰度传感器对象指针。
 * @param sample_times 采样次数，传入 0 时会自动修正为 1。
 */
void GW_SetSampleTimes(GW_Sensor *sensor, uint8_t sample_times)
{
    if (sensor == NULL)
    {
        return;
    }

    if (sample_times == 0u)
    {
        sample_times = 1u;
    }

    sensor->sample_times = sample_times;
}

/**
 * @brief 读取全部灰度通道的 ADC 原始值。
 * @param sensor 灰度传感器对象指针。
 * @return GW_OK 表示读取成功，其他值表示未配置、空指针或 ADC 读取失败。
 */
GW_Status GW_ReadAnalog(GW_Sensor *sensor)
{
    uint8_t i;
    uint8_t j;
    uint8_t out_index;
    uint32_t analog_sum;
    uint16_t adc_value;
    GW_Status status;

    if (sensor == NULL)
    {
        return GW_ERROR_NULL;
    }

    if (sensor->configured == 0u)
    {
        return GW_ERROR_NOT_CONFIGURED;
    }

    for (i = 0u; i < GW_CHANNELS; i++)
    {
        analog_sum = 0u;
        GW_SetAddress(sensor, i);

        for (j = 0u; j < sensor->sample_times; j++)
        {
            status = GW_ReadAdcOnce(&sensor->config, &adc_value);
            if (status != GW_OK)
            {
                return status;
            }
            analog_sum += adc_value;
        }

        if (sensor->direction == GW_DIR_REVERSE)
        {
            out_index = (uint8_t)((GW_CHANNELS - 1u) - i);
        }
        else
        {
            out_index = i;
        }

        sensor->analog[out_index] = (uint16_t)(analog_sum / sensor->sample_times);
    }

    return GW_OK;
}

/**
 * @brief 读取灰度传感器并更新归一化值、线位置和状态。
 * @param sensor 灰度传感器对象指针。
 * @return GW_OK 表示更新成功，其他值表示读取或配置失败原因。
 */
GW_Status GW_Update(GW_Sensor *sensor)
{
    uint8_t i;
    GW_Status status;

    if (sensor == NULL)
    {
        return GW_ERROR_NULL;
    }

    status = GW_ReadAnalog(sensor);
    if (status != GW_OK)
    {
        return status;
    }

    if (sensor->calibrated == 0u)
    {
        return GW_OK;
    }

    for (i = 0u; i < GW_CHANNELS; i++)
    {
        sensor->normalized[i] = GW_NormalizeOne(sensor->analog[i],
                                                sensor->calibrated_black[i],
                                                sensor->calibrated_white[i]);
    }

    GW_UpdateLineBits(sensor);
    GW_UpdatePosition(sensor);
    GW_UpdateState(sensor);

    return GW_OK;
}

/**
 * @brief 使用外部传入的归一化数据更新线位置和状态。
 * @param sensor 灰度传感器对象指针。
 * @param normalized 外部归一化数组，长度为 GW_CHANNELS。
 * @return GW_OK 表示处理成功，GW_ERROR_NULL 表示存在空指针参数。
 */
GW_Status GW_ProcessNormalized(GW_Sensor *sensor,
                               const uint16_t normalized[GW_CHANNELS])
{
    uint8_t i;

    if ((sensor == NULL) || (normalized == NULL))
    {
        return GW_ERROR_NULL;
    }

    for (i = 0u; i < GW_CHANNELS; i++)
    {
        if (normalized[i] > sensor->high_value)
        {
            sensor->normalized[i] = sensor->high_value;
        }
        else
        {
            sensor->normalized[i] = normalized[i];
        }
    }

    GW_UpdateLineBits(sensor);
    GW_UpdatePosition(sensor);
    GW_UpdateState(sensor);

    return GW_OK;
}

/**
 * @brief 获取当前巡线位图。
 * @param sensor 灰度传感器对象指针。
 * @return 当前巡线位图，每一位对应一个灰度通道。
 */
uint8_t GW_GetLineBits(const GW_Sensor *sensor)
{
    if (sensor == NULL)
    {
        return 0u;
    }

    return sensor->line_bits;
}

/**
 * @brief 获取当前线位置。
 * @param sensor 灰度传感器对象指针。
 * @return 当前线位置，负值偏左，正值偏右。
 */
float GW_GetPosition(const GW_Sensor *sensor)
{
    if (sensor == NULL)
    {
        return 0.0f;
    }

    return sensor->line_position;
}

/**
 * @brief 获取当前是否为正常巡线状态。
 * @param sensor 灰度传感器对象指针。
 * @return 1 表示正常巡线，0 表示非正常巡线或参数为空。
 */
uint8_t GW_IsNormal(const GW_Sensor *sensor)
{
    if (sensor == NULL)
    {
        return 0u;
    }

    return sensor->normal;
}

/**
 * @brief 获取当前是否丢线。
 * @param sensor 灰度传感器对象指针。
 * @return 1 表示丢线，0 表示未丢线或参数为空。
 */
uint8_t GW_IsLost(const GW_Sensor *sensor)
{
    if (sensor == NULL)
    {
        return 0u;
    }

    return sensor->lost;
}

/**
 * @brief 获取当前是否检测到十字线。
 * @param sensor 灰度传感器对象指针。
 * @return 1 表示检测到十字线，0 表示未检测到或参数为空。
 */
uint8_t GW_IsCross(const GW_Sensor *sensor)
{
    if (sensor == NULL)
    {
        return 0u;
    }

    return sensor->cross;
}

/**
 * @brief 拷贝当前 ADC 原始值数组。
 * @param sensor 灰度传感器对象指针。
 * @param out 用于接收 ADC 原始值的输出数组，长度为 GW_CHANNELS。
 */
void GW_CopyAnalog(const GW_Sensor *sensor, uint16_t out[GW_CHANNELS])
{
    if ((sensor == NULL) || (out == NULL))
    {
        return;
    }

    memcpy(out, sensor->analog, sizeof(sensor->analog));
}

/**
 * @brief 拷贝当前归一化值数组。
 * @param sensor 灰度传感器对象指针。
 * @param out 用于接收归一化值的输出数组，长度为 GW_CHANNELS。
 */
void GW_CopyNormalized(const GW_Sensor *sensor, uint16_t out[GW_CHANNELS])
{
    if ((sensor == NULL) || (out == NULL))
    {
        return;
    }

    memcpy(out, sensor->normalized, sizeof(sensor->normalized));
}
