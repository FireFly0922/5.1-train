#ifndef MAIXCAM_UART_H
#define MAIXCAM_UART_H

#include "usart.h"

void MaixCamUart_Init(void);
void MaixCamUart_Process(void);
void MaixCamUart_OnRxCplt(UART_HandleTypeDef *huart);
void MaixCamUart_OnError(UART_HandleTypeDef *huart);

#endif /* MAIXCAM_UART_H */
