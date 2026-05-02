#include "protocol_parser.h"

#include "status.h"

static float Protocol_DecodeYaw(const uint8_t *frame) {
    int16_t raw_yaw = (int16_t)(((uint16_t)frame[7] << 8U) | frame[6]);
    return ((float)raw_yaw / 32768.0f) * 180.0f;
}

void Protocol_ParseUartData(UART_HandleTypeDef *huart,
                            const uint8_t *buf,
                            uint16_t size) {
    uint16_t i;

    if ((huart == 0) || (buf == 0) || (size == 0U)) {
        return;
    }

    if (huart->Instance == USART3) {
        if (size < 11U) {
            return;
        }

        /* 陀螺仪一帧固定 11 字节，这里在缓冲区中滑动查找有效帧。 */
        for (i = 0U; i <= (uint16_t)(size - 11U); ++i) {
            uint8_t sum = 0U;
            uint8_t j;

            if ((buf[i] != 0x55U) || (buf[i + 1U] != 0x53U)) {
                continue;
            }

            for (j = 0U; j < 10U; ++j) {
                sum = (uint8_t)(sum + buf[i + j]);
            }

            if (sum == buf[i + 10U]) {
                /* 校验通过后直接更新姿态角，减少无意义的中转函数。 */
                STATUS.sensor.gyro.yaw = Protocol_DecodeYaw(&buf[i]);
                break;
            }
        }
    } else if (huart->Instance == USART1) {
        if ((size >= 2U) && (buf[0] == 0xA5U) && (buf[size - 1U] == 0x5AU)) {
            /* 当前协议里，第 2 个字节直接表示视觉目标值。 */
            STATUS.sensor.vision_target = (float)buf[1];
        }
    }
}
