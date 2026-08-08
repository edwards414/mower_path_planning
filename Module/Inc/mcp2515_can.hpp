#ifndef MODULE_INC_MCP2515_CAN_HPP_
#define MODULE_INC_MCP2515_CAN_HPP_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool spi_available;
  bool initialized;
  bool last_transfer_ok;
} mcp2515_status_t;

bool Mcp2515_Init(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);
bool Mcp2515_Reset(void);
bool Mcp2515_ReadRegister(uint8_t address, uint8_t *value);
bool Mcp2515_WriteRegister(uint8_t address, uint8_t value);
bool Mcp2515_BitModify(uint8_t address, uint8_t mask, uint8_t value);
void Mcp2515_GetStatus(mcp2515_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_MCP2515_CAN_HPP_ */
