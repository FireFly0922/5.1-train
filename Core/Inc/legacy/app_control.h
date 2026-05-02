#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "main.h"
#include "app_types.h"

void AppControl_Init(void);
void AppControl_Step(void);
void AppControl_On10msTick(void);
void AppControl_OnUartRx(UART_HandleTypeDef *huart, uint16_t size);
void AppControl_OnUartError(UART_HandleTypeDef *huart);

AppControlContext *AppControl_GetContext(void);
uint8_t *AppControl_GetUart1Buffer(void);
uint8_t *AppControl_GetUart3Buffer(void);

#endif /* APP_CONTROL_H */
