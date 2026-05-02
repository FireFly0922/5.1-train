#ifndef PROTOCOL_PARSER_H
#define PROTOCOL_PARSER_H

#include "main.h"
#include "app_types.h"

void Protocol_ParseUartData(AppControlContext *ctx, UART_HandleTypeDef *huart, const uint8_t *buf, uint16_t size);

#endif /* PROTOCOL_PARSER_H */
