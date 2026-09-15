/*
 * boot_shared.h
 *
 * Constants shared between the UART bootloader (bootloader/) and the
 * application. Both sides must agree on the flash layout, the RAM mailbox
 * used to request a bootloader entry, and the UART frame types.
 *
 * Flash layout (STM32F411CE, 512 KB):
 *
 *   sector 0   0x08000000  16 KB  \ bootloader (32 KB)
 *   sector 1   0x08004000  16 KB  /
 *   sector 2   0x08008000  16 KB  \
 *   sector 3   0x0800C000  16 KB   |
 *   sector 4   0x08010000  64 KB   | application (352 KB)
 *   sector 5   0x08020000 128 KB   |
 *   sector 6   0x08040000 128 KB  /
 *   sector 7   0x08060000 128 KB    settings (settings_storage.hpp)
 *
 * RAM: the first 32 bytes at 0x20000000 are excluded from both linker scripts
 * and used as a reset-surviving mailbox. The app writes BOOT_REQUEST_MAGIC
 * there and issues a system reset; the bootloader sees the magic, clears it,
 * and stays in download mode instead of jumping to the app.
 */
#ifndef MODULE_INC_BOOT_SHARED_H_
#define MODULE_INC_BOOT_SHARED_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Flash layout --------------------------------------------------------*/
#define BOOT_FLASH_BASE_ADDRESS 0x08000000UL
#define BOOT_BOOTLOADER_SIZE 0x00008000UL /* sectors 0-1 */
#define BOOT_APP_START_ADDRESS 0x08008000UL /* sector 2 */
#define BOOT_APP_END_ADDRESS 0x08060000UL   /* sector 7 = settings, never touched */
#define BOOT_APP_MAX_SIZE (BOOT_APP_END_ADDRESS - BOOT_APP_START_ADDRESS)
#define BOOT_APP_FIRST_SECTOR 2U
#define BOOT_APP_LAST_SECTOR 6U
#define BOOT_APP_VECTOR_OFFSET (BOOT_APP_START_ADDRESS - BOOT_FLASH_BASE_ADDRESS)

#define BOOT_RAM_BASE_ADDRESS 0x20000000UL
#define BOOT_RAM_SIZE 0x00020000UL /* 128 KB */

/* ---- Reset-surviving mailbox ---------------------------------------------*/
#define BOOT_SHARED_RAM_ADDRESS 0x20000000UL
#define BOOT_SHARED_RAM_SIZE 32U
#define BOOT_REQUEST_MAGIC 0xB007B007UL

#define BOOT_SHARED_MAGIC_PTR ((volatile uint32_t *)BOOT_SHARED_RAM_ADDRESS)

/* ---- Bootloader behaviour ------------------------------------------------*/
#define BOOT_VERSION 1U
/* After power-up with a valid app the bootloader listens this long for a
 * BL_PING before jumping. Lets a host recover a board whose app is broken. */
#define BOOT_GRACE_PERIOD_MS 500U
#define BOOT_WRITE_CHUNK_MAX 128U /* bytes of data per BL_WRITE, multiple of 4 */

/* ---- UART frame types (same framing as UART_OPEN_LOOP_PROTOCOL.md) -------*/
/* Handled by the application */
#define BOOT_FRAME_TYPE_ENTER_BOOTLOADER 0x0FU     /* Host -> app */
#define BOOT_FRAME_TYPE_ENTER_BOOTLOADER_ACK 0x8FU /* app -> Host */

/* Handled by the bootloader */
#define BOOT_FRAME_TYPE_BL_PING 0x10U    /* Host -> BL, no payload */
#define BOOT_FRAME_TYPE_BL_ERASE 0x11U   /* Host -> BL, boot_erase_payload_t */
#define BOOT_FRAME_TYPE_BL_WRITE 0x12U   /* Host -> BL, u32 offset + data */
#define BOOT_FRAME_TYPE_BL_VERIFY 0x13U  /* Host -> BL, boot_verify_payload_t */
#define BOOT_FRAME_TYPE_BL_RUN_APP 0x14U /* Host -> BL, no payload */
#define BOOT_FRAME_TYPE_BL_INFO 0x90U    /* BL -> Host, reply to PING */
#define BOOT_FRAME_TYPE_BL_ACK 0x91U     /* BL -> Host, reply to ERASE/WRITE/VERIFY/RUN */

/* ---- Payloads -------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
  uint32_t magic; /* must equal BOOT_REQUEST_MAGIC */
  uint32_t reserved;
} boot_enter_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t status; /* BOOT_STATUS_* */
  uint8_t reserved[3];
} boot_enter_ack_payload_t;

typedef struct __attribute__((packed)) {
  uint32_t app_start_address;
  uint32_t app_max_size;
  uint16_t write_chunk_max;
  uint8_t bootloader_version;
  uint8_t flags; /* BOOT_INFO_FLAG_* */
  uint32_t reserved;
} boot_info_payload_t;

#define BOOT_INFO_FLAG_APP_VALID 0x01U
#define BOOT_INFO_FLAG_ENTERED_BY_REQUEST 0x02U
#define BOOT_INFO_FLAG_ERASED 0x04U /* an erase has been done this session */

typedef struct __attribute__((packed)) {
  uint32_t image_size; /* bytes that will be written, 1..BOOT_APP_MAX_SIZE */
} boot_erase_payload_t;

/* BL_WRITE payload: boot_write_header_t followed by `data_len` bytes. */
typedef struct __attribute__((packed)) {
  uint32_t offset; /* byte offset from BOOT_APP_START_ADDRESS, multiple of 4 */
} boot_write_header_t;

typedef struct __attribute__((packed)) {
  uint32_t image_size;
  uint32_t crc32; /* CRC-32 (IEEE, same as zlib.crc32) of the image */
} boot_verify_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t command; /* frame type that is being acknowledged */
  uint8_t status;  /* BOOT_STATUS_* */
  uint32_t value;  /* WRITE: offset, VERIFY: computed crc32, others: 0 */
} boot_ack_payload_t;

#define BOOT_STATUS_OK 0U
#define BOOT_STATUS_BAD_ARGUMENT 1U
#define BOOT_STATUS_FLASH_ERROR 2U
#define BOOT_STATUS_CRC_MISMATCH 3U
#define BOOT_STATUS_OUT_OF_RANGE 4U
#define BOOT_STATUS_NOT_ERASED 5U
#define BOOT_STATUS_NO_VALID_APP 6U

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_BOOT_SHARED_H_ */
