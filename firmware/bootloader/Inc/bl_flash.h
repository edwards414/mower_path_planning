/* Internal-flash helpers for the application region (sectors 2-6). */
#ifndef BL_FLASH_H_
#define BL_FLASH_H_

#include <stdbool.h>
#include <stdint.h>

/* Erase every application sector needed to hold `image_size` bytes.
 * Never touches the bootloader (0-1) or settings (7) sectors. */
bool BlFlash_EraseForImage(uint32_t image_size);

/* Program `len` bytes (multiple of 4) at BOOT_APP_START_ADDRESS + offset and
 * read them back. */
bool BlFlash_Write(uint32_t offset, const uint8_t *data, uint32_t len);

/* CRC-32 (IEEE 802.3, same as zlib.crc32) over `len` bytes at `address`. */
uint32_t BlFlash_Crc32(uint32_t address, uint32_t len);

/* True when the reset vector at BOOT_APP_START_ADDRESS looks like a real
 * Cortex-M image: stack pointer inside RAM, reset handler inside the app
 * region with the Thumb bit set. */
bool BlFlash_AppLooksValid(void);

#endif /* BL_FLASH_H_ */
