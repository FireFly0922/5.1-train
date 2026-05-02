// @63

#include "log.h"

#include <stdarg.h>
#include <stdio.h>

/*
 * Returns the UART currently used for logging.
 * This indirection keeps the hardware binding local to the log module.
 */
static UART_HandleTypeDef *log_get_default_uart(void) {
  return LOG_DEFAULT_UART;
}

void log_printf(const char *format, ...) {
  UART_HandleTypeDef *huart = log_get_default_uart();
  va_list args;
  int len = 0;
  static char buf[LOG_FORMAT_BUF_LENGTH];

  if ((huart == NULL) || (format == NULL)) {
    return;
  }

  va_start(args, format);
  len = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (len <= 0) {
    return;
  }

  if (len >= (int)sizeof(buf)) {
    len = (int)sizeof(buf) - 1;
  }

  /*
   * Blocking transmit keeps the behavior simple during bring-up and debugging.
   * If you later need non-blocking logs, the change can stay inside this file.
   */
  HAL_UART_Transmit(huart, (uint8_t *)buf, (uint16_t)len, 100);
}
