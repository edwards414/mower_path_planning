#include "uart_interface.hpp"
#include "settings_storage.hpp"
#include "wheel_controller.hpp"
#include "ws2812.h"
#include "boot_animation.hpp"
#include "cmsis_os2.h"
#include <stddef.h>
#include <string.h>

volatile uint16_t uart_last_pos = 0;
uint8_t uart_rx_dma[UART_RX_DMA_BUF_SIZE];

namespace {
constexpr uint32_t UART_TX_DMA_WAIT_MS = 100U;
constexpr uint8_t UART_FRAME_BASE_SIZE = 8U;
constexpr uint8_t UART_CRC_INPUT_HEADER_SIZE = 4U;
constexpr uint16_t UART_DEFAULT_COMMAND_TIMEOUT_MS =
    MOTOR_DEFAULT_COMMAND_TIMEOUT_MS;
constexpr uint16_t LAWER_PWM_MAX_COUNTS = MOTOR_PWM_MAX_COUNTS;
constexpr uint16_t WS2812_DEFAULT_EFFECT_PERIOD_MS = 100U;
volatile bool uart_tx_ready = false;

typedef struct {
  int16_t command_permille;
  uint16_t command_timeout_ms;
  bool valid;
  uint8_t last_rx_seq;
  uint32_t last_update_ms;
} lawer_motor_runtime_command_t;

typedef struct {
  int16_t commanded_permille;
  int16_t applied_pwm;
  uint16_t command_age_ms;
  uint8_t flags;
  uint8_t last_rx_seq;
} lawer_motor_runtime_status_t;

typedef struct {
  uint8_t mode;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t effect_period_ms;
  bool valid;
  uint8_t last_rx_seq;
  uint32_t last_update_ms;
} ws2812_runtime_command_t;

typedef struct {
  uint8_t mode;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t effect_period_ms;
  uint8_t flags;
  uint8_t last_rx_seq;
} ws2812_runtime_status_t;

volatile lawer_motor_runtime_command_t g_lawer_command = {};
volatile lawer_motor_runtime_status_t g_lawer_status = {};
volatile ws2812_runtime_command_t g_ws2812_command = {};
volatile ws2812_runtime_status_t g_ws2812_status = {};
volatile uint8_t g_pid_last_rx_seq = 0U;
volatile bool g_pid_last_apply_ok = true;

uint8_t g_ws2812_flow_pos = 0U;
uint8_t g_ws2812_turn_left_pos = 0U;
uint8_t g_ws2812_turn_right_pos = (uint8_t)(LED_NUM - 1);
uint32_t g_ws2812_last_step_ms = 0U;
bool g_ws2812_static_applied = false;
uint8_t g_ws2812_last_static_seq = 0U;

uint32_t uart_enter_critical(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void uart_exit_critical(uint32_t primask) {
  if (primask == 0U) {
    __enable_irq();
  }
}

int16_t clamp_permille(int16_t command) {
  if (command > MOTOR_COMMAND_PERMILLE_LIMIT) {
    return MOTOR_COMMAND_PERMILLE_LIMIT;
  }
  if (command < -MOTOR_COMMAND_PERMILLE_LIMIT) {
    return -MOTOR_COMMAND_PERMILLE_LIMIT;
  }
  return command;
}

int16_t permille_to_lawer_pwm(int16_t command_permille) {
  int32_t scaled =
      (int32_t)clamp_permille(command_permille) * (int32_t)LAWER_PWM_MAX_COUNTS;
  return (int16_t)(scaled / MOTOR_COMMAND_PERMILLE_LIMIT);
}

uint16_t saturating_u16(uint32_t value) {
  if (value > 0xFFFFU) {
    return 0xFFFFU;
  }
  return (uint16_t)value;
}

uint16_t ws2812_sanitize_period(uint16_t effect_period_ms) {
  return (effect_period_ms == 0U) ? WS2812_DEFAULT_EFFECT_PERIOD_MS
                                  : effect_period_ms;
}

int16_t float_to_i16_x100(float value) {
  float scaled = value * 100.0f;
  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return -32768;
  }
  return (int16_t)scaled;
}

int16_t float_to_i16_pwm(float value) {
  if (value > 32767.0f) {
    return 32767;
  }
  if (value < -32768.0f) {
    return -32768;
  }
  return (int16_t)value;
}

void ws2812_set_led_pair(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  /* Animations index 0..LED_NUM-1 (16). Front strip has 32 LEDs, so map each
   * index to two adjacent front LEDs to keep both strips in step. */
  uint32_t f = (uint32_t)index * LED_NUM_FRONT / LED_NUM;
  ws2812_set_led_buf(ws2812_buf_front, (int)f, r, g, b);
  if (f + 1U < LED_NUM_FRONT) {
    ws2812_set_led_buf(ws2812_buf_front, (int)f + 1, r, g, b);
  }
  ws2812_set_led_buf(ws2812_buf_back, index, r, g, b);
}

void ws2812_pick_turn_color(const ws2812_runtime_command_t *command,
                            uint8_t *r, uint8_t *g, uint8_t *b) {
  if ((command == NULL) || (r == NULL) || (g == NULL) || (b == NULL)) {
    return;
  }

  if ((command->r == 0U) && (command->g == 0U) && (command->b == 0U)) {
    *r = 255U;
    *g = 120U;
    *b = 0U;
    return;
  }

  *r = command->r;
  *g = command->g;
  *b = command->b;
}

void ws2812_apply_static_mode(const ws2812_runtime_command_t *command) {
  if (command == NULL) {
    return;
  }

  switch (command->mode) {
  case UART_WS2812_MODE_CLEAR:
    ws2812_clear_all();
    ws2812_show_dual();
    break;

  case UART_WS2812_MODE_ALL_ON:
    ws2812_all_on(command->r, command->g, command->b);
    break;

  case UART_WS2812_MODE_SHOW:
    ws2812_show_dual();
    break;

  default:
    break;
  }
}

void ws2812_step_animation(const ws2812_runtime_command_t *command) {
  if (command == NULL) {
    return;
  }

  uint8_t main_r = 0U;
  uint8_t main_g = 0U;
  uint8_t main_b = 0U;

  switch (command->mode) {
  case UART_WS2812_MODE_FLOW:
    ws2812_clear_all();
    ws2812_set_led_pair(g_ws2812_flow_pos, command->r, command->g, command->b);
    ws2812_show_dual();
    g_ws2812_flow_pos = (uint8_t)((g_ws2812_flow_pos + 1U) % LED_NUM);
    break;

  case UART_WS2812_MODE_TURN_LEFT: {
    ws2812_pick_turn_color(command, &main_r, &main_g, &main_b);
    ws2812_clear_all();
    ws2812_set_led_pair(g_ws2812_turn_left_pos, main_r, main_g, main_b);
    ws2812_set_led_pair(
        (uint8_t)((g_ws2812_turn_left_pos + LED_NUM - 1U) % LED_NUM),
        (uint8_t)(main_r >> 1), (uint8_t)(main_g >> 1),
        (uint8_t)(main_b >> 1));
    ws2812_show_dual();
    g_ws2812_turn_left_pos =
        (uint8_t)((g_ws2812_turn_left_pos + 1U) % LED_NUM);
    break;
  }

  case UART_WS2812_MODE_TURN_RIGHT: {
    ws2812_pick_turn_color(command, &main_r, &main_g, &main_b);
    ws2812_clear_all();
    ws2812_set_led_pair(g_ws2812_turn_right_pos, main_r, main_g, main_b);
    ws2812_set_led_pair((uint8_t)((g_ws2812_turn_right_pos + 1U) % LED_NUM),
                        (uint8_t)(main_r >> 1), (uint8_t)(main_g >> 1),
                        (uint8_t)(main_b >> 1));
    ws2812_show_dual();
    g_ws2812_turn_right_pos =
        (g_ws2812_turn_right_pos == 0U)
            ? (uint8_t)(LED_NUM - 1U)
            : (uint8_t)(g_ws2812_turn_right_pos - 1U);
    break;
  }

  default:
    break;
  }
}

void lawer_set_open_loop_command(int16_t command_permille,
                                 uint16_t command_timeout_ms, uint8_t rx_seq) {
  uint32_t primask = uart_enter_critical();

  g_lawer_command.command_permille = clamp_permille(command_permille);
  g_lawer_command.command_timeout_ms =
      (command_timeout_ms == 0U) ? UART_DEFAULT_COMMAND_TIMEOUT_MS
                                 : command_timeout_ms;
  g_lawer_command.valid = true;
  g_lawer_command.last_rx_seq = rx_seq;
  g_lawer_command.last_update_ms = HAL_GetTick();

  uart_exit_critical(primask);
}

void ws2812_set_command(const ws2812_command_payload_t *payload, uint8_t rx_seq) {
  if (payload == NULL) {
    return;
  }

  uint32_t primask = uart_enter_critical();

  g_ws2812_command.mode = payload->mode;
  g_ws2812_command.r = payload->r;
  g_ws2812_command.g = payload->g;
  g_ws2812_command.b = payload->b;
  g_ws2812_command.effect_period_ms =
      ws2812_sanitize_period(payload->effect_period_ms);
  g_ws2812_command.valid = true;
  g_ws2812_command.last_rx_seq = rx_seq;
  g_ws2812_command.last_update_ms = HAL_GetTick();

  uart_exit_critical(primask);
}

void pid_config_set_command(const pid_config_payload_t *payload,
                            uint8_t rx_seq) {
  if (payload == NULL) {
    return;
  }

  controller_settings_t settings = {};
  WheelController_GetSettings(&settings);

  settings.left_wheel_pid.kp = payload->left_kp;
  settings.left_wheel_pid.ki = payload->left_ki;
  settings.left_wheel_pid.kd = payload->left_kd;
  settings.right_wheel_pid.kp = payload->right_kp;
  settings.right_wheel_pid.ki = payload->right_ki;
  settings.right_wheel_pid.kd = payload->right_kd;
  settings.closed_loop_enabled =
      (payload->closed_loop_enabled == 0U) ? 0U : 1U;

  bool ok = WheelController_ApplySettings(&settings,
                                          payload->persist_to_flash != 0U);

  uint32_t primask = uart_enter_critical();
  g_pid_last_rx_seq = rx_seq;
  g_pid_last_apply_ok = ok;
  uart_exit_critical(primask);
}

void update_lawer_control(void) {
  lawer_motor_runtime_command_t command_snapshot = {};

  uint32_t primask = uart_enter_critical();
  command_snapshot.command_permille = g_lawer_command.command_permille;
  command_snapshot.command_timeout_ms = g_lawer_command.command_timeout_ms;
  command_snapshot.valid = g_lawer_command.valid;
  command_snapshot.last_rx_seq = g_lawer_command.last_rx_seq;
  command_snapshot.last_update_ms = g_lawer_command.last_update_ms;
  uart_exit_critical(primask);

  uint32_t now = HAL_GetTick();
  uint32_t command_age_ms =
      command_snapshot.valid ? (now - command_snapshot.last_update_ms)
                             : 0xFFFFFFFFU;
  bool timeout = (!command_snapshot.valid) ||
                 (command_age_ms > command_snapshot.command_timeout_ms);

  int16_t applied_pwm = 0;
  if (!timeout) {
    applied_pwm = permille_to_lawer_pwm(command_snapshot.command_permille);
  }
  /* Blade is single-direction: negative commands are refused (reported as 0). */
  if (applied_pwm < 0) {
    applied_pwm = 0;
  }

  uint8_t dir = (applied_pwm >= 0) ? 1U : 0U;
  uint16_t pwm = (applied_pwm >= 0) ? (uint16_t)applied_pwm
                                    : (uint16_t)(-applied_pwm);
  Grass_cutting_motor(pwm, dir);

  lawer_motor_runtime_status_t status = {};
  status.commanded_permille =
      command_snapshot.valid ? command_snapshot.command_permille : (int16_t)0;
  status.applied_pwm = applied_pwm;
  status.command_age_ms = saturating_u16(command_age_ms);
  status.flags = 0U;
  status.last_rx_seq = command_snapshot.last_rx_seq;

  if (command_snapshot.valid) {
    status.flags |= MOTOR_STATUS_FLAG_COMMAND_VALID;
  }
  if (timeout) {
    status.flags |= MOTOR_STATUS_FLAG_COMMAND_TIMEOUT;
  }

  primask = uart_enter_critical();
  g_lawer_status.commanded_permille = status.commanded_permille;
  g_lawer_status.applied_pwm = status.applied_pwm;
  g_lawer_status.command_age_ms = status.command_age_ms;
  g_lawer_status.flags = status.flags;
  g_lawer_status.last_rx_seq = status.last_rx_seq;
  uart_exit_critical(primask);
}

void update_ws2812_control(void) {
  ws2812_runtime_command_t command_snapshot = {};

  uint32_t primask = uart_enter_critical();
  command_snapshot.mode = g_ws2812_command.mode;
  command_snapshot.r = g_ws2812_command.r;
  command_snapshot.g = g_ws2812_command.g;
  command_snapshot.b = g_ws2812_command.b;
  command_snapshot.effect_period_ms = g_ws2812_command.effect_period_ms;
  command_snapshot.valid = g_ws2812_command.valid;
  command_snapshot.last_rx_seq = g_ws2812_command.last_rx_seq;
  command_snapshot.last_update_ms = g_ws2812_command.last_update_ms;
  uart_exit_critical(primask);

  /* Power-on light show owns the strips until it finishes; a UART command
   * received meanwhile is applied as soon as it ends. */
  bool boot_animation_active = BootAnimation_Update();
  if (boot_animation_active) {
    g_ws2812_static_applied = false;
  }

  if (command_snapshot.valid && !boot_animation_active) {
    if ((command_snapshot.mode == UART_WS2812_MODE_FLOW) ||
        (command_snapshot.mode == UART_WS2812_MODE_TURN_LEFT) ||
        (command_snapshot.mode == UART_WS2812_MODE_TURN_RIGHT)) {
      uint32_t now = HAL_GetTick();
      if ((now - g_ws2812_last_step_ms) >= command_snapshot.effect_period_ms) {
        ws2812_step_animation(&command_snapshot);
        g_ws2812_last_step_ms = now;
      }
      g_ws2812_static_applied = false;
    } else if ((!g_ws2812_static_applied) ||
               (g_ws2812_last_static_seq != command_snapshot.last_rx_seq)) {
      ws2812_apply_static_mode(&command_snapshot);
      g_ws2812_last_static_seq = command_snapshot.last_rx_seq;
      g_ws2812_static_applied = true;
    }
  }

  ws2812_runtime_status_t status = {};
  status.mode = command_snapshot.mode;
  status.r = command_snapshot.r;
  status.g = command_snapshot.g;
  status.b = command_snapshot.b;
  status.effect_period_ms = command_snapshot.effect_period_ms;
  status.flags = command_snapshot.valid ? MOTOR_STATUS_FLAG_COMMAND_VALID : 0U;
  status.last_rx_seq = command_snapshot.last_rx_seq;

  primask = uart_enter_critical();
  g_ws2812_status.mode = status.mode;
  g_ws2812_status.r = status.r;
  g_ws2812_status.g = status.g;
  g_ws2812_status.b = status.b;
  g_ws2812_status.effect_period_ms = status.effect_period_ms;
  g_ws2812_status.flags = status.flags;
  g_ws2812_status.last_rx_seq = status.last_rx_seq;
  uart_exit_critical(primask);
}

enum class ParserState : uint8_t {
  WaitSof0 = 0,
  WaitSof1,
  ReadVersion,
  ReadType,
  ReadSeq,
  ReadLength,
  ReadPayload,
  ReadCrcLow,
  ReadCrcHigh,
};

typedef struct {
  ParserState state;
  uint8_t version;
  uint8_t type;
  uint8_t seq;
  uint8_t payload_len;
  uint8_t payload_index;
  uint8_t payload[UART_MAX_PAYLOAD_SIZE];
  uint8_t crc_low;
} UartFrameParser;

UartFrameParser g_parser = {};

void parser_reset(UartFrameParser *parser) {
  if (parser == NULL) {
    return;
  }

  parser->state = ParserState::WaitSof0;
  parser->version = 0U;
  parser->type = 0U;
  parser->seq = 0U;
  parser->payload_len = 0U;
  parser->payload_index = 0U;
  parser->crc_low = 0U;
  memset(parser->payload, 0, sizeof(parser->payload));
}

void uart_write_blocking(const uint8_t *data, uint16_t len) {
  if ((data == NULL) || (len == 0U)) {
    return;
  }

  for (uint32_t retry = 0; retry < 5U; ++retry) {
    HAL_StatusTypeDef status =
        HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000U);
    if (status == HAL_OK) {
      return;
    }

    if (status != HAL_BUSY) {
      return;
    }

    for (volatile uint32_t spin = 0; spin < 20000U; ++spin) {
    }
  }
}

uint16_t uart_crc16_ccitt(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFFU;

  if (data == NULL) {
    return crc;
  }

  for (uint16_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

void uart_handle_frame(uint8_t version, uint8_t type, uint8_t seq,
                       const uint8_t *payload, uint8_t payload_len) {
  if ((version != UART_PROTOCOL_VERSION) || (payload == NULL)) {
    return;
  }

  switch (type) {
  case UART_FRAME_TYPE_MOTOR_OPEN_LOOP_COMMAND:
    if (payload_len != sizeof(motor_open_loop_command_payload_t)) {
      return;
    }

    motor_open_loop_command_payload_t command_payload;
    memcpy(&command_payload, payload, sizeof(command_payload));
    Motor_SetOpenLoopCommand(command_payload.left_command_permille,
                             command_payload.right_command_permille,
                             command_payload.command_timeout_ms, seq);
    break;

  case UART_FRAME_TYPE_LAWER_MOTOR_COMMAND:
    if (payload_len != sizeof(lawer_motor_command_payload_t)) {
      return;
    }
    lawer_motor_command_payload_t lawer_payload;
    memcpy(&lawer_payload, payload, sizeof(lawer_payload));
    lawer_set_open_loop_command(lawer_payload.command_permille,
                                lawer_payload.command_timeout_ms, seq);
    break;

  case UART_FRAME_TYPE_WS2812_COMMAND:
    if (payload_len != sizeof(ws2812_command_payload_t)) {
      return;
    }
    ws2812_command_payload_t ws_payload;
    memcpy(&ws_payload, payload, sizeof(ws_payload));
    ws2812_set_command(&ws_payload, seq);
    break;

  case UART_FRAME_TYPE_PID_CONFIG_COMMAND:
    if (payload_len != sizeof(pid_config_payload_t)) {
      return;
    }
    pid_config_payload_t pid_payload;
    memcpy(&pid_payload, payload, sizeof(pid_payload));
    pid_config_set_command(&pid_payload, seq);
    break;

  default:
    break;
  }
}

void uart_finalize_frame(UartFrameParser *parser, uint8_t crc_high) {
  if (parser == NULL) {
    return;
  }

  uint8_t crc_input[UART_CRC_INPUT_HEADER_SIZE + UART_MAX_PAYLOAD_SIZE];
  uint16_t crc_input_len = UART_CRC_INPUT_HEADER_SIZE + parser->payload_len;

  crc_input[0] = parser->version;
  crc_input[1] = parser->type;
  crc_input[2] = parser->seq;
  crc_input[3] = parser->payload_len;
  if (parser->payload_len > 0U) {
    memcpy(&crc_input[UART_CRC_INPUT_HEADER_SIZE], parser->payload,
           parser->payload_len);
  }

  uint16_t received_crc =
      (uint16_t)parser->crc_low | ((uint16_t)crc_high << 8);
  uint16_t expected_crc = uart_crc16_ccitt(crc_input, crc_input_len);
  if (received_crc == expected_crc) {
    uart_handle_frame(parser->version, parser->type, parser->seq,
                      parser->payload, parser->payload_len);
  }
}

void uart_process_byte(UartFrameParser *parser, uint8_t byte) {
  if (parser == NULL) {
    return;
  }

  switch (parser->state) {
  case ParserState::WaitSof0:
    if (byte == UART_FRAME_SOF_0) {
      parser->state = ParserState::WaitSof1;
    }
    break;

  case ParserState::WaitSof1:
    if (byte == UART_FRAME_SOF_1) {
      parser->state = ParserState::ReadVersion;
    } else if (byte == UART_FRAME_SOF_0) {
      parser->state = ParserState::WaitSof1;
    } else {
      parser_reset(parser);
    }
    break;

  case ParserState::ReadVersion:
    parser->version = byte;
    parser->state = ParserState::ReadType;
    break;

  case ParserState::ReadType:
    parser->type = byte;
    parser->state = ParserState::ReadSeq;
    break;

  case ParserState::ReadSeq:
    parser->seq = byte;
    parser->state = ParserState::ReadLength;
    break;

  case ParserState::ReadLength:
    if (byte > UART_MAX_PAYLOAD_SIZE) {
      parser_reset(parser);
      break;
    }

    parser->payload_len = byte;
    parser->payload_index = 0U;
    if (parser->payload_len == 0U) {
      parser->state = ParserState::ReadCrcLow;
    } else {
      parser->state = ParserState::ReadPayload;
    }
    break;

  case ParserState::ReadPayload:
    parser->payload[parser->payload_index++] = byte;
    if (parser->payload_index >= parser->payload_len) {
      parser->state = ParserState::ReadCrcLow;
    }
    break;

  case ParserState::ReadCrcLow:
    parser->crc_low = byte;
    parser->state = ParserState::ReadCrcHigh;
    break;

  case ParserState::ReadCrcHigh:
    uart_finalize_frame(parser, byte);
    parser_reset(parser);
    break;

  default:
    parser_reset(parser);
    break;
  }
}

void uart_consume_chunk(const UartChunk_t *chunk) {
  if ((chunk == NULL) || (chunk->len == 0U) ||
      (chunk->start >= UART_RX_DMA_BUF_SIZE)) {
    return;
  }

  if ((uint32_t)chunk->start + (uint32_t)chunk->len > UART_RX_DMA_BUF_SIZE) {
    return;
  }

  for (uint16_t i = 0; i < chunk->len; ++i) {
    uart_process_byte(&g_parser, uart_rx_dma[chunk->start + i]);
  }
}

bool uart_send_frame(uint8_t type, uint8_t seq, const void *payload,
                     uint8_t payload_len) {
  if (payload_len > UART_MAX_PAYLOAD_SIZE) {
    return false;
  }

  uint16_t frame_len = (uint16_t)(UART_FRAME_BASE_SIZE + payload_len);
  if (frame_len > UART_TX_MAX_SIZE) {
    return false;
  }

  UartMsg_t msg = {};
  msg.data[0] = UART_FRAME_SOF_0;
  msg.data[1] = UART_FRAME_SOF_1;
  msg.data[2] = UART_PROTOCOL_VERSION;
  msg.data[3] = type;
  msg.data[4] = seq;
  msg.data[5] = payload_len;

  if ((payload_len > 0U) && (payload != NULL)) {
    memcpy(&msg.data[6], payload, payload_len);
  }

  uint16_t crc = uart_crc16_ccitt(&msg.data[2],
                                  (uint16_t)(UART_CRC_INPUT_HEADER_SIZE +
                                             payload_len));
  msg.data[6 + payload_len] = (uint8_t)(crc & 0xFFU);
  msg.data[7 + payload_len] = (uint8_t)((crc >> 8) & 0xFFU);
  msg.len = frame_len;

  if ((osKernelGetState() != osKernelRunning) || (uartTxQueue == NULL) ||
      !uart_tx_ready) {
    uart_write_blocking(msg.data, msg.len);
    return true;
  }

  return (osMessageQueuePut(uartTxQueue, &msg, 0U, 0U) == osOK);
}

void uart_send_motor_status(void) {
  motor_open_loop_status_t status_snapshot;
  Motor_GetStatusSnapshot(&status_snapshot);

  motor_status_payload_t status_payload;
  status_payload.commanded_left_permille =
      status_snapshot.commanded_left_permille;
  status_payload.commanded_right_permille =
      status_snapshot.commanded_right_permille;
  status_payload.applied_left_pwm = status_snapshot.applied_left_pwm;
  status_payload.applied_right_pwm = status_snapshot.applied_right_pwm;
  status_payload.command_age_ms = status_snapshot.command_age_ms;
  status_payload.flags = status_snapshot.flags;
  status_payload.last_rx_seq = status_snapshot.last_rx_seq;

  (void)uart_send_frame(UART_FRAME_TYPE_MOTOR_STATUS,
                        status_snapshot.last_rx_seq, &status_payload,
                        (uint8_t)sizeof(status_payload));
}

void uart_send_lawer_status(void) {
  lawer_motor_runtime_status_t status_snapshot = {};

  uint32_t primask = uart_enter_critical();
  status_snapshot.commanded_permille = g_lawer_status.commanded_permille;
  status_snapshot.applied_pwm = g_lawer_status.applied_pwm;
  status_snapshot.command_age_ms = g_lawer_status.command_age_ms;
  status_snapshot.flags = g_lawer_status.flags;
  status_snapshot.last_rx_seq = g_lawer_status.last_rx_seq;
  uart_exit_critical(primask);

  lawer_motor_status_payload_t status_payload = {};
  status_payload.commanded_permille = status_snapshot.commanded_permille;
  status_payload.applied_pwm = status_snapshot.applied_pwm;
  status_payload.command_age_ms = status_snapshot.command_age_ms;
  status_payload.flags = status_snapshot.flags;
  status_payload.last_rx_seq = status_snapshot.last_rx_seq;

  (void)uart_send_frame(UART_FRAME_TYPE_LAWER_MOTOR_STATUS,
                        status_snapshot.last_rx_seq, &status_payload,
                        (uint8_t)sizeof(status_payload));
}

void uart_send_ws2812_status(void) {
  ws2812_runtime_status_t status_snapshot = {};

  uint32_t primask = uart_enter_critical();
  status_snapshot.mode = g_ws2812_status.mode;
  status_snapshot.r = g_ws2812_status.r;
  status_snapshot.g = g_ws2812_status.g;
  status_snapshot.b = g_ws2812_status.b;
  status_snapshot.effect_period_ms = g_ws2812_status.effect_period_ms;
  status_snapshot.flags = g_ws2812_status.flags;
  status_snapshot.last_rx_seq = g_ws2812_status.last_rx_seq;
  uart_exit_critical(primask);

  ws2812_status_payload_t status_payload = {};
  status_payload.mode = status_snapshot.mode;
  status_payload.r = status_snapshot.r;
  status_payload.g = status_snapshot.g;
  status_payload.b = status_snapshot.b;
  status_payload.effect_period_ms = status_snapshot.effect_period_ms;
  status_payload.flags = status_snapshot.flags;
  status_payload.last_rx_seq = status_snapshot.last_rx_seq;

  (void)uart_send_frame(UART_FRAME_TYPE_WS2812_STATUS,
                        status_snapshot.last_rx_seq, &status_payload,
                        (uint8_t)sizeof(status_payload));
}

void uart_send_pid_config_status(void) {
  controller_settings_t settings = {};
  settings_storage_status_t storage_status = {};
  uint8_t last_seq = 0U;
  bool last_apply_ok = false;

  WheelController_GetSettings(&settings);
  SettingsStorage_GetStatus(&storage_status);

  uint32_t primask = uart_enter_critical();
  last_seq = g_pid_last_rx_seq;
  last_apply_ok = g_pid_last_apply_ok;
  uart_exit_critical(primask);

  pid_config_status_payload_t payload = {};
  payload.left_kp = settings.left_wheel_pid.kp;
  payload.left_ki = settings.left_wheel_pid.ki;
  payload.left_kd = settings.left_wheel_pid.kd;
  payload.right_kp = settings.right_wheel_pid.kp;
  payload.right_ki = settings.right_wheel_pid.ki;
  payload.right_kd = settings.right_wheel_pid.kd;
  payload.flags = 0U;
  payload.last_rx_seq = last_seq;
  if (settings.closed_loop_enabled != 0U) {
    payload.flags |= UART_PID_STATUS_FLAG_CLOSED_LOOP_ENABLED;
  }
  if (storage_status.flash_valid) {
    payload.flags |= UART_PID_STATUS_FLAG_FLASH_VALID;
  }
  if (storage_status.last_save_ok) {
    payload.flags |= UART_PID_STATUS_FLAG_LAST_SAVE_OK;
  }
  if (last_apply_ok) {
    payload.flags |= UART_PID_STATUS_FLAG_LAST_APPLY_OK;
  }

  (void)uart_send_frame(UART_FRAME_TYPE_PID_CONFIG_STATUS, last_seq, &payload,
                        (uint8_t)sizeof(payload));
}

void uart_send_wheel_feedback_status(void) {
  wheel_controller_status_t wheel_status = {};
  motor_open_loop_status_t motor_status = {};

  WheelController_GetStatus(&wheel_status);
  Motor_GetStatusSnapshot(&motor_status);

  wheel_feedback_status_payload_t payload = {};
  payload.left_target_rpm_x100 =
      float_to_i16_x100(wheel_status.left.target_rpm);
  payload.left_measured_rpm_x100 =
      float_to_i16_x100(wheel_status.left.measured_rpm);
  payload.right_target_rpm_x100 =
      float_to_i16_x100(wheel_status.right.target_rpm);
  payload.right_measured_rpm_x100 =
      float_to_i16_x100(wheel_status.right.measured_rpm);
  payload.left_pid_output = float_to_i16_pwm(wheel_status.left.pid_output);
  payload.right_pid_output = float_to_i16_pwm(wheel_status.right.pid_output);
  payload.left_delta_counts = wheel_status.left.delta_counts;
  payload.right_delta_counts = wheel_status.right.delta_counts;
  payload.flags = wheel_status.flags;

  (void)uart_send_frame(UART_FRAME_TYPE_WHEEL_FEEDBACK_STATUS,
                        motor_status.last_rx_seq, &payload,
                        (uint8_t)sizeof(payload));
}
} // namespace

void uart_server(void) {
  uart_last_pos = 0U;
  parser_reset(&g_parser);

  if (HAL_UART_Receive_DMA(&huart1, uart_rx_dma, UART_RX_DMA_BUF_SIZE) ==
      HAL_OK) {
    if (huart1.hdmarx != NULL) {
      __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  }
}

void UartParserTask(void *arg) {
  (void)arg;

  uart_server();

  for (;;) {
    UartChunk_t chunk;
    if (osMessageQueueGet(uartRxQueue, &chunk, NULL, osWaitForever) == osOK) {
      uart_consume_chunk(&chunk);
    }
  }
}

void MotorTask(void *arg) {
  (void)arg;

  uint32_t last_status_tick = HAL_GetTick();

  for (;;) {
    control_update_50hz();
    update_lawer_control();
    update_ws2812_control();

    uint32_t now = HAL_GetTick();
    if ((now - last_status_tick) >= UART_STATUS_PERIOD_MS) {
      uart_send_motor_status();
      uart_send_wheel_feedback_status();
      uart_send_lawer_status();
      uart_send_ws2812_status();
      uart_send_pid_config_status();
      last_status_tick = now;
    }

    osDelay(20U);
  }
}

void UartTxTask(void *arg) {
  (void)arg;

  uart_tx_ready = true;

  for (;;) {
    UartMsg_t msg;
    if (osMessageQueueGet(uartTxQueue, &msg, NULL, osWaitForever) != osOK) {
      continue;
    }

    if ((msg.len == 0U) || (msg.len > UART_TX_MAX_SIZE)) {
      continue;
    }

    (void)osThreadFlagsClear(0x01U);

    if (HAL_UART_Transmit_DMA(&huart1, msg.data, msg.len) != HAL_OK) {
      (void)HAL_UART_AbortTransmit(&huart1);
      uart_write_blocking(msg.data, msg.len);
      continue;
    }

    if ((osThreadFlagsWait(0x01U, osFlagsWaitAny, UART_TX_DMA_WAIT_MS) &
         0x01U) == 0U) {
      (void)HAL_UART_AbortTransmit(&huart1);
      uart_write_blocking(msg.data, msg.len);
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if ((huart == &huart1) && (uartTxTaskHandle != NULL)) {
    (void)osThreadFlagsSet(uartTxTaskHandle, 0x01U);
  }
}

int _write(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;

  return (len > 0) ? len : 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if ((huart == NULL) || (huart->Instance != USART1)) {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);

  if (huart->hdmarx != NULL) {
    uart_last_pos = 0U;
    (void)HAL_UART_Receive_DMA(&huart1, uart_rx_dma, UART_RX_DMA_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  }
}
