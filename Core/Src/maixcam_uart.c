#include "maixcam_uart.h"

#include <stdint.h>

#include "log.h"
#include "status.h"

#define MAIXCAM_UART_RX_BUFFER_SIZE 128U
#define MAIXCAM_UART_PRINT_BATCH_BYTES 16U
#define MAIXCAM_UART_STATUS_PERIOD_MS 1000U
#define MAIXCAM_UART_FRAME_SIZE 8U
#define MAIXCAM_UART_FRAME_HEAD0 0xAAU
#define MAIXCAM_UART_FRAME_HEAD1 0x55U
#define MAIXCAM_UART_FRAME_LENGTH 0x03U
#define MAIXCAM_UART_FRAME_CMD_CLASS 0x01U
#define MAIXCAM_UART_INVALID_CLASS_ID 0xFFU

static volatile uint16_t g_maixcam_rx_head;
static volatile uint16_t g_maixcam_rx_tail;
static volatile uint8_t g_maixcam_rx_overflow;
static volatile uint8_t g_maixcam_rx_error;
static volatile uint32_t g_maixcam_rx_error_code;
static volatile uint32_t g_maixcam_rx_total;
static uint32_t g_maixcam_last_status_tick;
static uint8_t g_maixcam_rx_buffer[MAIXCAM_UART_RX_BUFFER_SIZE];
static uint8_t g_maixcam_rx_byte;
static uint8_t g_maixcam_parse_window[MAIXCAM_UART_FRAME_SIZE];
static uint8_t g_maixcam_parse_len;

static uint16_t MaixCamUart_NextIndex(uint16_t index)
{
  index++;
  if (index >= MAIXCAM_UART_RX_BUFFER_SIZE) {
    index = 0U;
  }
  return index;
}

static void MaixCamUart_StartReceive(void)
{
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &g_maixcam_rx_byte, 1U);

  if ((status != HAL_OK) && (status != HAL_BUSY)) {
    g_maixcam_rx_error = 1U;
    g_maixcam_rx_error_code = (uint32_t)status;
  }
}

static void MaixCamUart_PushByte(uint8_t byte)
{
  uint16_t next_head = MaixCamUart_NextIndex(g_maixcam_rx_head);

  if (next_head == g_maixcam_rx_tail) {
    g_maixcam_rx_overflow = 1U;
    return;
  }

  g_maixcam_rx_buffer[g_maixcam_rx_head] = byte;
  g_maixcam_rx_head = next_head;
  g_maixcam_rx_total++;
}

static uint16_t MaixCamUart_PopBatch(uint8_t *out, uint16_t max_len,
                                     uint8_t *overflow)
{
  uint16_t len = 0U;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();

  if (overflow != NULL) {
    *overflow = g_maixcam_rx_overflow;
    g_maixcam_rx_overflow = 0U;
  }

  while ((len < max_len) && (g_maixcam_rx_tail != g_maixcam_rx_head)) {
    out[len] = g_maixcam_rx_buffer[g_maixcam_rx_tail];
    g_maixcam_rx_tail = MaixCamUart_NextIndex(g_maixcam_rx_tail);
    len++;
  }

  if (primask == 0U) {
    __enable_irq();
  }

  return len;
}

static const char *MaixCamUart_RouteName(uint8_t class_id)
{
  switch (class_id) {
    case 0U:
      return "A";
    case 1U:
      return "B";
    case 2U:
      return "C";
    case 3U:
      return "D";
    default:
      return "?";
  }
}

static const char *MaixCamUart_StageName(int32_t stage)
{
  switch (stage) {
    case ROUTE_STAGE_WAIT_VISION:
      return "WAIT_VISION";
    case ROUTE_STAGE_ROUTE_LOCKED:
      return "ROUTE_LOCKED";
    case ROUTE_STAGE_WAIT_ROUTE:
      return "WAIT_ROUTE";
    case ROUTE_STAGE_LINE_FOLLOW:
      return "LINE_FOLLOW";
    case ROUTE_STAGE_CROSS_DETECTED:
      return "CROSS_DETECTED";
    case ROUTE_STAGE_TURN_LEFT:
      return "TURN_LEFT";
    case ROUTE_STAGE_TURN_RIGHT:
      return "TURN_RIGHT";
    case ROUTE_STAGE_STRAIGHT_THROUGH:
      return "STRAIGHT_THROUGH";
    case ROUTE_STAGE_REACQUIRE_LINE:
      return "REACQUIRE_LINE";
    case ROUTE_STAGE_DONE:
      return "DONE";
    case ROUTE_STAGE_INVALID:
      return "INVALID";
    default:
      return "UNKNOWN";
  }
}

static void MaixCamUart_SetInvalidRoute(uint8_t class_id, uint8_t prob,
                                        uint8_t valid)
{
  if ((STATUS.sensor.vision_valid == 1U) &&
      (STATUS.state.route_id >= ROUTE_ID_A) &&
      (STATUS.state.route_id <= ROUTE_ID_D)) {
    PRINTLN("MaixCam route invalid ignored id=%u prob=%u valid=%u locked=%s",
            (unsigned int)class_id,
            (unsigned int)prob,
            (unsigned int)valid,
            MaixCamUart_RouteName((uint8_t)STATUS.state.route_id));
    return;
  }

  STATUS.sensor.vision_class_id = class_id;
  STATUS.sensor.vision_prob = prob;
  STATUS.sensor.vision_valid = valid;
  STATUS.sensor.vision_target = (float)class_id;
  STATUS.state.route_id = ROUTE_ID_NONE;
  STATUS.state.route_stage = ROUTE_STAGE_INVALID;
  STATUS.state.route_first_action = ROUTE_ACTION_NONE;
  STATUS.state.route_second_action = ROUTE_ACTION_NONE;
}

static void MaixCamUart_SetRoute(uint8_t class_id, uint8_t prob,
                                 uint8_t valid)
{
  int32_t first_action = ROUTE_ACTION_NONE;
  int32_t second_action = ROUTE_ACTION_NONE;

  if ((STATUS.sensor.vision_valid == 1U) &&
      (STATUS.state.route_id >= ROUTE_ID_A) &&
      (STATUS.state.route_id <= ROUTE_ID_D)) {
    PRINTLN("MaixCam route locked keep=%s new=%s id=%u prob=%u",
            MaixCamUart_RouteName((uint8_t)STATUS.state.route_id),
            MaixCamUart_RouteName(class_id),
            (unsigned int)class_id,
            (unsigned int)prob);
    return;
  }

  STATUS.sensor.vision_class_id = class_id;
  STATUS.sensor.vision_prob = prob;
  STATUS.sensor.vision_valid = valid;
  STATUS.sensor.vision_target = (float)class_id;
  STATUS.state.route_id = (int32_t)class_id;

  switch (class_id) {
    case 0U:
      first_action = ROUTE_ACTION_LEFT;
      break;
    case 1U:
      first_action = ROUTE_ACTION_RIGHT;
      break;
    case 2U:
      first_action = ROUTE_ACTION_STRAIGHT;
      second_action = ROUTE_ACTION_LEFT;
      break;
    case 3U:
      first_action = ROUTE_ACTION_STRAIGHT;
      second_action = ROUTE_ACTION_RIGHT;
      break;
    default:
      MaixCamUart_SetInvalidRoute(class_id, prob, valid);
      return;
  }

  STATUS.state.route_stage = ROUTE_STAGE_ROUTE_LOCKED;
  STATUS.state.car_run_state = ROUTE_STAGE_ROUTE_LOCKED;
  STATUS.state.route_first_action = first_action;
  STATUS.state.route_second_action = second_action;

  PRINTLN("MaixCam route=%s id=%u prob=%u valid=%u state=%s",
          MaixCamUart_RouteName(class_id),
          (unsigned int)class_id,
          (unsigned int)prob,
          (unsigned int)valid,
          MaixCamUart_StageName(ROUTE_STAGE_ROUTE_LOCKED));

  if (second_action == ROUTE_ACTION_LEFT) {
    PRINTLN("MaixCam route=%s next=SECOND_T_LEFT",
            MaixCamUart_RouteName(class_id));
  } else if (second_action == ROUTE_ACTION_RIGHT) {
    PRINTLN("MaixCam route=%s next=SECOND_T_RIGHT",
            MaixCamUart_RouteName(class_id));
  }
}

static uint8_t MaixCamUart_Checksum(const uint8_t *frame)
{
  return (uint8_t)(frame[2] + frame[3] + frame[4] + frame[5] + frame[6]);
}

static void MaixCamUart_HandleFrame(const uint8_t *frame)
{
  uint8_t class_id = frame[4];
  uint8_t prob = frame[5];
  uint8_t valid = frame[6];
  uint8_t checksum = MaixCamUart_Checksum(frame);

  if (checksum != frame[7]) {
    PRINTLN("MaixCam frame checksum error");
    return;
  }

  if ((frame[2] != MAIXCAM_UART_FRAME_LENGTH) ||
      (frame[3] != MAIXCAM_UART_FRAME_CMD_CLASS)) {
    PRINTLN("MaixCam frame unsupported len=%u cmd=%u",
            (unsigned int)frame[2],
            (unsigned int)frame[3]);
    return;
  }

  if ((valid != 1U) || (class_id > 3U) || (prob > 100U)) {
    MaixCamUart_SetInvalidRoute(class_id, prob, valid);
    PRINTLN("MaixCam route invalid id=%u prob=%u valid=%u",
            (unsigned int)class_id,
            (unsigned int)prob,
            (unsigned int)valid);
    return;
  }

  MaixCamUart_SetRoute(class_id, prob, valid);
}

static void MaixCamUart_ResetParserToHeader(uint8_t byte)
{
  if (byte == MAIXCAM_UART_FRAME_HEAD0) {
    g_maixcam_parse_window[0] = byte;
    g_maixcam_parse_len = 1U;
  } else {
    g_maixcam_parse_len = 0U;
  }
}

static void MaixCamUart_ParseByte(uint8_t byte)
{
  if (g_maixcam_parse_len == 0U) {
    MaixCamUart_ResetParserToHeader(byte);
    return;
  }

  if ((g_maixcam_parse_len == 1U) && (byte != MAIXCAM_UART_FRAME_HEAD1)) {
    MaixCamUart_ResetParserToHeader(byte);
    return;
  }

  g_maixcam_parse_window[g_maixcam_parse_len] = byte;
  g_maixcam_parse_len++;

  if (g_maixcam_parse_len >= MAIXCAM_UART_FRAME_SIZE) {
    MaixCamUart_HandleFrame(g_maixcam_parse_window);
    g_maixcam_parse_len = 0U;
  }
}

static void MaixCamUart_ParseBatch(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0U; i < len; i++) {
    MaixCamUart_ParseByte(data[i]);
  }
}

static void MaixCamUart_PrintHex(const uint8_t *data, uint16_t len)
{
  static const char hex_chars[] = "0123456789ABCDEF";
  char hex[(MAIXCAM_UART_PRINT_BATCH_BYTES * 3U)];
  uint16_t out_index = 0U;

  for (uint16_t i = 0U; i < len; i++) {
    if (i > 0U) {
      hex[out_index] = ' ';
      out_index++;
    }
    hex[out_index] = hex_chars[(data[i] >> 4) & 0x0FU];
    out_index++;
    hex[out_index] = hex_chars[data[i] & 0x0FU];
    out_index++;
  }

  hex[out_index] = '\0';
  PRINTLN("MaixCam RX len=%u hex=%s", (unsigned int)len, hex);
}

void MaixCamUart_Init(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  g_maixcam_rx_head = 0U;
  g_maixcam_rx_tail = 0U;
  g_maixcam_rx_overflow = 0U;
  g_maixcam_rx_error = 0U;
  g_maixcam_rx_error_code = 0U;
  g_maixcam_rx_total = 0U;
  g_maixcam_parse_len = 0U;
  if (primask == 0U) {
    __enable_irq();
  }

  STATUS.sensor.vision_class_id = MAIXCAM_UART_INVALID_CLASS_ID;
  STATUS.sensor.vision_prob = 0U;
  STATUS.sensor.vision_valid = 0U;
  STATUS.state.route_id = ROUTE_ID_NONE;
  STATUS.state.route_stage = ROUTE_STAGE_WAIT_VISION;
  STATUS.state.route_first_action = ROUTE_ACTION_NONE;
  STATUS.state.route_second_action = ROUTE_ACTION_NONE;

  g_maixcam_last_status_tick = HAL_GetTick();
  MaixCamUart_StartReceive();
  PRINTLN("MaixCam UART ready: USART2 RX PA3 115200 8N1, log USART1");
}

void MaixCamUart_Process(void)
{
  uint8_t batch[MAIXCAM_UART_PRINT_BATCH_BYTES];
  uint8_t overflow = 0U;
  uint8_t error = 0U;
  uint32_t error_code = 0U;
  uint32_t total = 0U;
  uint32_t now_tick = HAL_GetTick();
  uint16_t len = MaixCamUart_PopBatch(batch, MAIXCAM_UART_PRINT_BATCH_BYTES,
                                      &overflow);

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  error = g_maixcam_rx_error;
  error_code = g_maixcam_rx_error_code;
  total = g_maixcam_rx_total;
  g_maixcam_rx_error = 0U;
  g_maixcam_rx_error_code = 0U;
  if (primask == 0U) {
    __enable_irq();
  }

  if (overflow != 0U) {
    PRINTLN("MaixCam RX overflow");
  }

  if (error != 0U) {
    PRINTLN("MaixCam UART error=0x%08lX", (unsigned long)error_code);
  }

  if (len > 0U) {
    MaixCamUart_PrintHex(batch, len);
    MaixCamUart_ParseBatch(batch, len);
  }

  if ((now_tick - g_maixcam_last_status_tick) >= MAIXCAM_UART_STATUS_PERIOD_MS) {
    PRINTLN("MaixCam RX total=%lu", (unsigned long)total);
    g_maixcam_last_status_tick = now_tick;
  }
}

void MaixCamUart_OnRxCplt(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2)) {
    return;
  }

  MaixCamUart_PushByte(g_maixcam_rx_byte);
  MaixCamUart_StartReceive();
}

void MaixCamUart_OnError(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2)) {
    return;
  }

  g_maixcam_rx_error = 1U;
  g_maixcam_rx_error_code = huart->ErrorCode;
  (void)HAL_UART_AbortReceive(&huart2);
  MaixCamUart_StartReceive();
}
