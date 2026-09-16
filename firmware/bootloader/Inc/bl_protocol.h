/* Frame parser and command handlers for the bootloader side of the UART
 * protocol (framing identical to UART_OPEN_LOOP_PROTOCOL.md). */
#ifndef BL_PROTOCOL_H_
#define BL_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool entered_by_request; /* app wrote BOOT_REQUEST_MAGIC before reset */
  bool app_valid;          /* reset vector at app start looks sane */
  bool stay;               /* do not auto-jump after the grace period */
  bool run_app;            /* host asked us to jump to the app */
  bool erased;             /* an erase succeeded in this session */
  uint32_t erased_size;    /* bytes covered by that erase */
} bl_session_t;

void BlProtocol_Init(bl_session_t *session);

/* Feed one received byte; complete CRC-valid frames are dispatched. */
void BlProtocol_FeedByte(uint8_t byte);

/* Provided by main.c: blocking UART transmit. */
void BlUart_Send(const uint8_t *data, uint16_t len);

#endif /* BL_PROTOCOL_H_ */
