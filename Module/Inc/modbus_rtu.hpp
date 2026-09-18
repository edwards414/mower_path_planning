#ifndef MODULE_INC_MODBUS_RTU_HPP_
#define MODULE_INC_MODBUS_RTU_HPP_

/* Minimal Modbus RTU master codec: CRC-16, FC03 read request, FC03 reply
 * parsing. Pure C, no HAL dependency, so it can be unit tested on the host.
 * The RS485 charger (数控 30V5A) only needs FC03; FC16 write is provided for
 * a later CC/CV set command. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_FC_READ_HOLDING 0x03U
#define MODBUS_FC_WRITE_MULTIPLE 0x10U
#define MODBUS_EXCEPTION_BIT 0x80U

/* Standard Modbus CRC-16 (poly 0xA001 reflected, init 0xFFFF). The low
 * byte is transmitted first. */
uint16_t ModbusRtu_Crc16(const uint8_t *data, size_t len);

/* Build "addr 03 start(2) count(2) crc(2)". Returns bytes written (8) or 0
 * when the buffer is too small. */
size_t ModbusRtu_BuildReadHolding(uint8_t *out, size_t out_size, uint8_t addr,
                                  uint16_t start_reg, uint16_t reg_count);

/* Build "addr 10 start(2) count(2) bytes(1) data... crc(2)". Returns bytes
 * written or 0 when the buffer is too small. */
size_t ModbusRtu_BuildWriteMultiple(uint8_t *out, size_t out_size,
                                    uint8_t addr, uint16_t start_reg,
                                    const uint16_t *regs, uint16_t reg_count);

typedef enum {
  MODBUS_PARSE_NONE = 0,      /* no complete frame for this addr found */
  MODBUS_PARSE_OK = 1,        /* regs filled, reg_count valid */
  MODBUS_PARSE_EXCEPTION = 2, /* device answered FC|0x80, exception_code set */
} modbus_parse_result_t;

typedef struct {
  uint16_t reg_count;      /* registers decoded on OK */
  uint8_t exception_code;  /* valid on EXCEPTION */
  size_t frame_offset;     /* where the frame started in the buffer */
} modbus_read_reply_t;

/* Scan `buf` for a FC03 reply from `addr` with a valid CRC. The scan starts
 * at every offset so an echoed request (half-duplex transceivers that loop
 * TX back to RX) or leading noise is skipped. Up to `max_regs` registers are
 * stored in `regs` (big-endian on the wire, host order in `regs`). */
modbus_parse_result_t ModbusRtu_FindReadReply(const uint8_t *buf, size_t len,
                                              uint8_t addr, uint16_t *regs,
                                              uint16_t max_regs,
                                              modbus_read_reply_t *info);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_MODBUS_RTU_HPP_ */
