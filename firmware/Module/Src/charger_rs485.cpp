#include "charger_rs485.hpp"
#include "hardware_pins.hpp"
#include "modbus_rtu.hpp"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include <string.h>

namespace {

/* Request (8) + echo (8) + reply (15) fits with margin; a garbage burst
 * larger than this just ends the poll as a bad frame. */
constexpr uint16_t RX_BUF_SIZE = 64U;

enum class PollState : uint8_t {
  Idle = 0,
  Waiting,
};

charger_rs485_snapshot_t g_snapshot = {};
/* Written by the 10 ms tick, read by the USART6 ISR. */
volatile PollState g_state = PollState::Idle;
bool g_enabled = true;
uint32_t g_last_poll_start_ms = 0U;
uint32_t g_tx_start_ms = 0U;

uint8_t g_tx_buf[8];
uint8_t g_rx_buf[RX_BUF_SIZE];
/* Written from the USART6 ISR, read from the 10 ms tick. */
volatile uint16_t g_rx_len = 0U;
volatile bool g_rx_done = false;
uint16_t g_regs[CHARGER_RS485_REG_COUNT];
modbus_read_reply_t g_reply_info = {};
volatile modbus_parse_result_t g_rx_result = MODBUS_PARSE_NONE;

/* MAX485 DE/RE: high while our request is on the wire, low otherwise. */
void set_driver_enabled(bool tx) {
#if CHARGER_RS485_USE_DE_PIN
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin,
                    tx ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  (void)tx;
#endif
}

void configure_de_pin(void) {
#if CHARGER_RS485_USE_DE_PIN
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
  GPIO_InitTypeDef gpio = {};
  gpio.Pin = RS485_DE_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_DE_GPIO_Port, &gpio);
#endif
}

/* Arm reception for the rest of the buffer starting at `offset`. */
bool arm_rx(uint16_t offset) {
  if (offset >= RX_BUF_SIZE) {
    return false;
  }
  return HAL_UARTEx_ReceiveToIdle_IT(&huart6, &g_rx_buf[offset],
                                     (uint16_t)(RX_BUF_SIZE - offset)) ==
         HAL_OK;
}

void note_failure(void) {
  g_snapshot.comm_error_count++;
  if (g_snapshot.consecutive_fails < 0xFFU) {
    g_snapshot.consecutive_fails++;
  }
  if (g_snapshot.consecutive_fails >= CHARGER_RS485_OFFLINE_AFTER_FAILS) {
    g_snapshot.online = false;
  }
}

void note_success(uint32_t now) {
  g_snapshot.voltage_cv = g_regs[0];
  g_snapshot.current_ca = g_regs[1];
  g_snapshot.temp_c = g_regs[2];
  g_snapshot.reg3 = g_regs[3];
  g_snapshot.reg4 = g_regs[4];
  g_snapshot.online = true;
  g_snapshot.ever_seen = true;
  g_snapshot.last_ok_ms = now;
  g_snapshot.consecutive_fails = 0U;
  g_snapshot.last_exception_code = 0U;
}

void start_poll(uint32_t now) {
  g_last_poll_start_ms = now;
  g_rx_len = 0U;
  g_rx_done = false;
  g_rx_result = MODBUS_PARSE_NONE;

  size_t n = ModbusRtu_BuildReadHolding(g_tx_buf, sizeof(g_tx_buf),
                                        CHARGER_RS485_SLAVE_ADDR, 0U,
                                        CHARGER_RS485_REG_COUNT);
  if (n == 0U) {
    note_failure();
    return;
  }

  /* Drop anything that arrived while idle (noise, late reply) so it does
   * not raise ORE and abort the reception we are about to start. */
  (void)HAL_UART_AbortReceive(&huart6);
  __HAL_UART_CLEAR_OREFLAG(&huart6);
  __HAL_UART_FLUSH_DRREGISTER(&huart6);

  /* RX is armed before TX so a transceiver that echoes our own request
   * lands in the buffer instead of overrunning the DR. */
  if (!arm_rx(0U)) {
    note_failure();
    return;
  }
  /* Waiting must be visible to the ISR before the first echoed byte. */
  g_tx_start_ms = now;
  g_state = PollState::Waiting;
  set_driver_enabled(true);
  if (HAL_UART_Transmit_IT(&huart6, g_tx_buf, (uint16_t)n) != HAL_OK) {
    set_driver_enabled(false);
    (void)HAL_UART_AbortReceive(&huart6);
    g_state = PollState::Idle;
    note_failure();
    return;
  }
}

void finish_poll(uint32_t now) {
  modbus_parse_result_t result = g_rx_result;
  if (result == MODBUS_PARSE_OK) {
    if (g_reply_info.reg_count >= CHARGER_RS485_REG_COUNT) {
      note_success(now);
    } else {
      note_failure();
    }
  } else if (result == MODBUS_PARSE_EXCEPTION) {
    g_snapshot.last_exception_code = g_reply_info.exception_code;
    note_failure();
  } else {
    note_failure();
  }
  g_state = PollState::Idle;
}

} // namespace

void ChargerRs485_Init(void) {
  configure_de_pin();
  memset(&g_snapshot, 0, sizeof(g_snapshot));
  g_state = PollState::Idle;
  g_enabled = true;
  /* First poll one period after boot so the module has time to power up. */
  g_last_poll_start_ms = HAL_GetTick();
}

void ChargerRs485_SetEnabled(bool enabled) {
  if (g_enabled == enabled) {
    return;
  }
  g_enabled = enabled;
  if (!enabled && (g_state == PollState::Waiting)) {
    (void)HAL_UART_AbortTransmit(&huart6);
    set_driver_enabled(false);
    (void)HAL_UART_AbortReceive(&huart6);
    g_state = PollState::Idle;
  }
}

void ChargerRs485_Update10ms(void) {
  uint32_t now = HAL_GetTick();

  switch (g_state) {
  case PollState::Idle:
    if (!g_enabled) {
      /* Nothing on the bus: age out the online flag like a lost reply. */
      if (g_snapshot.online &&
          ((now - g_snapshot.last_ok_ms) >
           (CHARGER_RS485_POLL_PERIOD_MS *
            CHARGER_RS485_OFFLINE_AFTER_FAILS))) {
        g_snapshot.online = false;
      }
      break;
    }
    if ((now - g_last_poll_start_ms) >= CHARGER_RS485_POLL_PERIOD_MS) {
      start_poll(now);
    }
    break;

  case PollState::Waiting:
    if (g_rx_done) {
      finish_poll(now);
    } else if ((now - g_tx_start_ms) >= CHARGER_RS485_REPLY_TIMEOUT_MS) {
      (void)HAL_UART_AbortTransmit(&huart6);
      set_driver_enabled(false);
      (void)HAL_UART_AbortReceive(&huart6);
      g_rx_result = MODBUS_PARSE_NONE;
      finish_poll(now);
    }
    break;
  }
}

void ChargerRs485_GetSnapshot(charger_rs485_snapshot_t *snapshot) {
  if (snapshot == nullptr) {
    return;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *snapshot = g_snapshot;
  __set_PRIMASK(primask);
}

void ChargerRs485_OnTxComplete(void) {
  /* TC interrupt: the last stop bit has left the shifter, so the bus can
   * be released for the slave's turnaround. */
  set_driver_enabled(false);
}

bool ChargerRs485_IsCharging(const charger_rs485_snapshot_t *s) {
  return (s != nullptr) && s->online &&
         (s->current_ca >= CHARGER_RS485_CHARGING_MIN_CA);
}

bool ChargerRs485_IsInputPresent(const charger_rs485_snapshot_t *s) {
  return (s != nullptr) && s->online &&
         (s->voltage_cv >= CHARGER_RS485_INPUT_PRESENT_MIN_CV);
}

/* USART6 RX-to-idle event. Runs in the USART6 ISR. The reply may arrive in
 * pieces (echoed request, then the slave's reply after its turnaround), so
 * keep accumulating until a valid frame is found or the buffer is full. */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart6) {
    return;
  }
  if (g_state != PollState::Waiting) {
    return;
  }

  uint16_t total = (uint16_t)(g_rx_len + Size);
  if (total > RX_BUF_SIZE) {
    total = RX_BUF_SIZE;
  }
  g_rx_len = total;

  modbus_parse_result_t result = ModbusRtu_FindReadReply(
      g_rx_buf, total, CHARGER_RS485_SLAVE_ADDR, g_regs,
      CHARGER_RS485_REG_COUNT, &g_reply_info);
  if ((result != MODBUS_PARSE_NONE) || !arm_rx(total)) {
    g_rx_result = result;
    g_rx_done = true;
  }
}
