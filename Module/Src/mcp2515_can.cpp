#include "mcp2515_can.hpp"
#include "hardware_pins.hpp"
#include "stm32f4xx_hal.h"

#ifdef HAL_SPI_MODULE_ENABLED
extern "C" SPI_HandleTypeDef hspi1 __attribute__((weak));
#endif

namespace {

constexpr uint8_t MCP2515_CMD_RESET = 0xC0U;
constexpr uint8_t MCP2515_CMD_READ = 0x03U;
constexpr uint8_t MCP2515_CMD_WRITE = 0x02U;
constexpr uint8_t MCP2515_CMD_BIT_MODIFY = 0x05U;
constexpr uint8_t MCP2515_REG_CNF3 = 0x28U;
constexpr uint8_t MCP2515_REG_CNF2 = 0x29U;
constexpr uint8_t MCP2515_REG_CNF1 = 0x2AU;

mcp2515_status_t g_status = {};

void configure_cs_pin(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = CAN_CS_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &gpio);
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);
}

void cs_low(void) {
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);
}

void cs_high(void) {
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);
}

bool spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len) {
  if ((tx == nullptr) || (len == 0U)) {
    return false;
  }

#ifdef HAL_SPI_MODULE_ENABLED
  if (&hspi1 == nullptr) {
    g_status.spi_available = false;
    g_status.last_transfer_ok = false;
    return false;
  }

  uint8_t dummy_rx[4] = {};
  if (rx == nullptr) {
    rx = dummy_rx;
  }
  HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, 100U);
  g_status.spi_available = true;
  g_status.last_transfer_ok = (status == HAL_OK);
  return status == HAL_OK;
#else
  (void)rx;
  g_status.spi_available = false;
  g_status.last_transfer_ok = false;
  return false;
#endif
}

} // namespace

bool Mcp2515_Reset(void) {
  uint8_t cmd = MCP2515_CMD_RESET;
  cs_low();
  bool ok = spi_transfer(&cmd, nullptr, 1U);
  cs_high();
  HAL_Delay(10U);
  return ok;
}

bool Mcp2515_WriteRegister(uint8_t address, uint8_t value) {
  uint8_t tx[3] = {MCP2515_CMD_WRITE, address, value};
  cs_low();
  bool ok = spi_transfer(tx, nullptr, sizeof(tx));
  cs_high();
  return ok;
}

bool Mcp2515_ReadRegister(uint8_t address, uint8_t *value) {
  if (value == nullptr) {
    return false;
  }

  uint8_t tx[3] = {MCP2515_CMD_READ, address, 0x00U};
  uint8_t rx[3] = {};
  cs_low();
  bool ok = spi_transfer(tx, rx, sizeof(tx));
  cs_high();
  if (ok) {
    *value = rx[2];
  }
  return ok;
}

bool Mcp2515_BitModify(uint8_t address, uint8_t mask, uint8_t value) {
  uint8_t tx[4] = {MCP2515_CMD_BIT_MODIFY, address, mask, value};
  cs_low();
  bool ok = spi_transfer(tx, nullptr, sizeof(tx));
  cs_high();
  return ok;
}

bool Mcp2515_Init(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3) {
  configure_cs_pin();
  g_status = {};

  bool ok = Mcp2515_Reset();
  ok = ok && Mcp2515_WriteRegister(MCP2515_REG_CNF1, cnf1);
  ok = ok && Mcp2515_WriteRegister(MCP2515_REG_CNF2, cnf2);
  ok = ok && Mcp2515_WriteRegister(MCP2515_REG_CNF3, cnf3);
  g_status.initialized = ok;
  return ok;
}

void Mcp2515_GetStatus(mcp2515_status_t *status) {
  if (status == nullptr) {
    return;
  }
  *status = g_status;
}
