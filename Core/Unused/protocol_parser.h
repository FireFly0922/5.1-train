#ifndef PROTOCOL_PARSER_H
#define PROTOCOL_PARSER_H

#include <stdint.h>

#include "main.h"

void Protocol_ParseUartData(UART_HandleTypeDef *huart,
                            const uint8_t *buf,
                            uint16_t size);

#endif /* PROTOCOL_PARSER_H */
