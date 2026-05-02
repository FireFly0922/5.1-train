// @63

#ifndef __LOG_H__
#define __LOG_H__

/*
 * Centralized logging interface.
 *
 * Upper-layer modules should only use the macros and function declared here.
 * This keeps formatting rules and the physical output interface isolated from
 * business logic.
 */

#include "usart.h"

/*
 * Default UART used by the logging module.
 * If you later want to redirect logs to another UART, change this macro and
 * keep the rest of the application code untouched.
 */
#define LOG_DEFAULT_UART (&huart1)

/*
 * Maximum formatted log message length, including the string terminator.
 * Messages longer than this are truncated before transmission.
 */
#define LOG_FORMAT_BUF_LENGTH 256

/*
 * Low-level log output entry.
 * Most code should use PRINTF/PRINTLN/INFO/WARN/ERROR instead of calling this
 * function directly.
 */
void log_printf(const char *format, ...);

/* Raw formatted output. */
#define PRINTF(fmt, ...) log_printf(fmt, ##__VA_ARGS__)

/* Raw formatted output with CRLF line ending. */
#define PRINTLN(fmt, ...) PRINTF(fmt "\r\n", ##__VA_ARGS__)

/* Log with a short severity prefix. */
#define LOG_EVENT(level, fmt, ...) PRINTLN(level " " fmt, ##__VA_ARGS__)

/* Log with function name and source line for quick tracing. */
#define LOG_SPAN(level, fmt, ...) \
  LOG_EVENT(level, "%s:%u " fmt, __func__, __LINE__, ##__VA_ARGS__)

#define ERROR(fmt, ...) LOG_EVENT("E", fmt, ##__VA_ARGS__)
#define WARN(fmt, ...) LOG_EVENT("W", fmt, ##__VA_ARGS__)
#define INFO(fmt, ...) LOG_EVENT("I", fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) LOG_SPAN("D", fmt, ##__VA_ARGS__)
#define TRACE(var, fmt) LOG_SPAN("T", #var "=" fmt, var)

#define THROW_ERROR(fmt, ...) LOG_SPAN("E", fmt, ##__VA_ARGS__)
#define THROW_WARN(fmt, ...) LOG_SPAN("W", fmt, ##__VA_ARGS__)

#ifdef DEV
#define LOG_ENABLE
#endif  // DEV

#ifndef LOG_ENABLE
#undef LOG_EVENT
#define LOG_EVENT(level, fmt, ...)
#endif  // !LOG_ENABLE

#endif  // !__LOG_H__
