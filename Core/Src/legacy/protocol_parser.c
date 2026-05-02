#include "protocol_parser.h"

/* JY901 鍋忚埅瑙掕В鐮侊紝鍗曚綅杞崲涓鸿搴︺€?*/
static float protocol_decode_yaw(const uint8_t *frame)
{
    int16_t raw_yaw;
    raw_yaw = (int16_t)(((uint16_t)frame[7] << 8U) | frame[6]);
    return ((float)raw_yaw / 32768.0f) * 180.0f;
}

void Protocol_ParseUartData(AppControlContext *ctx, UART_HandleTypeDef *huart, const uint8_t *buf, uint16_t size)
{
    uint16_t i;

    if ((ctx == 0) || (huart == 0) || (buf == 0) || (size == 0U)) {
        return;
    }

    if (huart->Instance == USART3) {
        if (size >= 11U) {
            for (i = 0U; i <= (uint16_t)(size - 11U); i++) {
                uint8_t sum;
                uint8_t j;

                if ((buf[i] != 0x55U) || (buf[i + 1U] != 0x53U)) {
                    continue;
                }

                sum = 0U;
                for (j = 0U; j < 10U; j++) {
                    sum = (uint8_t)(sum + buf[i + j]);
                }

                if (sum == buf[i + 10U]) {
                    ctx->current_yaw = protocol_decode_yaw(&buf[i]);
                    break;
                }
            }
        }
    } else if (huart->Instance == USART1) {
        if ((size >= 2U) && (buf[0] == 0xA5U) && (buf[size - 1U] == 0x5AU)) {
            ctx->vision_target = (float)buf[1];
        }
    }
}
