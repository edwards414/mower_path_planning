/*
 * The STM32F411CE's internal flash for the fake bootloader.
 *
 * Linked with the UNMODIFIED firmware/bootloader/Src/bl_protocol.c into a
 * shared library that fake_base.py --bootloader-lib drives byte by byte. This
 * file supplies what bl_protocol.c calls:
 *
 *   - the four functions of firmware/bootloader/Inc/bl_flash.h, over a
 *     512 KB array standing for 0x08000000..0x0807FFFF: the same sector map,
 *     erase rule and argument checks as bl_flash.c, programming that can only
 *     clear bits (NOR flash: the new word is ANDed in) and is read back, an
 *     erase that fills with 0xFF, the same CRC-32 and vector table check;
 *   - BlUart_Send, which hands each reply frame to a callback (Python).
 *
 * Anything that would program or erase outside sectors 2-6 (the bootloader,
 * or sector 7 with the PID settings) aborts the process after reporting
 * SIM_EV_VIOLATION: neither host tool can make the real bootloader do that,
 * so the fake must never see it either.
 *
 * Faults for the tests: an erase that fails (before anything is erased, like
 * a HAL_FLASH_Unlock failure), a write at one offset that fails, and a bit
 * that flips after a write at one offset was programmed and read back (so
 * the write is acknowledged and only BL_VERIFY's CRC can catch it).
 *
 * sim_boot() starts a bootloader session exactly as main() in
 * firmware/bootloader/Src/bl_main.c does; the mailbox, the 500 ms grace
 * period and the jump are fake_base.py's.
 */
#include "bl_flash.h"
#include "bl_protocol.h"
#include "boot_shared.h"

#include <stdlib.h>
#include <string.h>

#define SIM_FLASH_SIZE (512U * 1024U)

enum {
  SIM_EV_ERASE = 1,        /* a: first sector, b: number of sectors */
  SIM_EV_ERASE_FAIL = 2,   /* a: first sector, b: number of sectors */
  SIM_EV_WRITE = 3,        /* a: offset, b: bytes */
  SIM_EV_WRITE_FAIL = 4,   /* a: offset, b: bytes (refused or not read back) */
  SIM_EV_CRC = 5,          /* a: address, b: bytes */
  SIM_EV_FLIP = 6,         /* a: offset of the flipped byte, b: bit */
  SIM_EV_VIOLATION = 7,    /* a: address, b: bytes; the process aborts */
};

typedef void (*sim_tx_fn)(const uint8_t *data, uint16_t len);
typedef void (*sim_event_fn)(int kind, uint32_t a, uint32_t b);

static uint8_t g_flash[SIM_FLASH_SIZE];
static bl_session_t g_session;
static sim_tx_fn g_tx;
static sim_event_fn g_event;

static int g_fail_erase;
static uint32_t g_fail_write_offset = UINT32_MAX;
static uint32_t g_flip_offset = UINT32_MAX;
static uint32_t g_flip_bit;

/* bl_flash.c kSectors */
static const struct {
  uint32_t start;
  uint32_t size;
} kSectors[] = {
    {0x08000000UL, 16U * 1024U},  /* 0 */
    {0x08004000UL, 16U * 1024U},  /* 1 */
    {0x08008000UL, 16U * 1024U},  /* 2 */
    {0x0800C000UL, 16U * 1024U},  /* 3 */
    {0x08010000UL, 64U * 1024U},  /* 4 */
    {0x08020000UL, 128U * 1024U}, /* 5 */
    {0x08040000UL, 128U * 1024U}, /* 6 */
    {0x08060000UL, 128U * 1024U}, /* 7 */
};

static void event(int kind, uint32_t a, uint32_t b) {
  if (g_event != NULL) {
    g_event(kind, a, b);
  }
}

static uint8_t *at(uint32_t address) { return &g_flash[address - BOOT_FLASH_BASE_ADDRESS]; }

/* Only sectors 2-6 may ever change. */
static void guard(uint32_t address, uint32_t len) {
  if ((address < BOOT_APP_START_ADDRESS) || (address >= BOOT_APP_END_ADDRESS) ||
      (len > (BOOT_APP_END_ADDRESS - address))) {
    event(SIM_EV_VIOLATION, address, len);
    abort();
  }
}

bool BlFlash_EraseForImage(uint32_t image_size) {
  if ((image_size == 0U) || (image_size > BOOT_APP_MAX_SIZE)) {
    return false;
  }

  uint32_t image_end = BOOT_APP_START_ADDRESS + image_size;
  uint32_t last_sector = BOOT_APP_FIRST_SECTOR;
  for (uint32_t s = BOOT_APP_FIRST_SECTOR; s <= BOOT_APP_LAST_SECTOR; ++s) {
    if (kSectors[s].start < image_end) {
      last_sector = s;
    }
  }
  uint32_t count = last_sector - BOOT_APP_FIRST_SECTOR + 1U;

  if (g_fail_erase) {
    event(SIM_EV_ERASE_FAIL, BOOT_APP_FIRST_SECTOR, count);
    return false;
  }
  for (uint32_t s = BOOT_APP_FIRST_SECTOR; s <= last_sector; ++s) {
    guard(kSectors[s].start, kSectors[s].size);
    memset(at(kSectors[s].start), 0xFF, kSectors[s].size);
  }
  event(SIM_EV_ERASE, BOOT_APP_FIRST_SECTOR, count);
  return true;
}

bool BlFlash_Write(uint32_t offset, const uint8_t *data, uint32_t len) {
  if ((data == 0) || (len == 0U) || ((len % 4U) != 0U) || ((offset % 4U) != 0U) ||
      (offset >= BOOT_APP_MAX_SIZE) || (len > (BOOT_APP_MAX_SIZE - offset))) {
    event(SIM_EV_WRITE_FAIL, offset, len);
    return false;
  }
  if (offset == g_fail_write_offset) {
    event(SIM_EV_WRITE_FAIL, offset, len);
    return false;
  }

  uint32_t address = BOOT_APP_START_ADDRESS + offset;
  guard(address, len);
  bool ok = true;
  for (uint32_t i = 0U; i < len; i += 4U) {
    uint32_t word = (uint32_t)data[i] | ((uint32_t)data[i + 1U] << 8) |
                    ((uint32_t)data[i + 2U] << 16) | ((uint32_t)data[i + 3U] << 24);
    uint8_t *cell = at(address + i);
    uint32_t old = (uint32_t)cell[0] | ((uint32_t)cell[1] << 8) | ((uint32_t)cell[2] << 16) |
                   ((uint32_t)cell[3] << 24);
    uint32_t now = old & word; /* programming only clears bits */
    cell[0] = (uint8_t)now;
    cell[1] = (uint8_t)(now >> 8);
    cell[2] = (uint8_t)(now >> 16);
    cell[3] = (uint8_t)(now >> 24);
    if (now != word) { /* bl_flash.c's read-back check */
      ok = false;
      break;
    }
  }
  if (!ok) {
    event(SIM_EV_WRITE_FAIL, offset, len);
    return false;
  }
  if (offset == g_flip_offset) {
    uint32_t byte = offset + (g_flip_bit / 8U);
    *at(BOOT_APP_START_ADDRESS + byte) ^= (uint8_t)(1U << (g_flip_bit % 8U));
    event(SIM_EV_FLIP, byte, g_flip_bit % 8U);
  }
  event(SIM_EV_WRITE, offset, len);
  return true;
}

uint32_t BlFlash_Crc32(uint32_t address, uint32_t len) {
  const uint8_t *bytes = at(address);
  uint32_t crc = 0xFFFFFFFFUL;

  for (uint32_t i = 0U; i < len; ++i) {
    crc ^= (uint32_t)bytes[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 1UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
  }
  event(SIM_EV_CRC, address, len);
  return ~crc;
}

bool BlFlash_AppLooksValid(void) {
  const uint8_t *p = at(BOOT_APP_START_ADDRESS);
  uint32_t sp = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
                ((uint32_t)p[3] << 24);
  uint32_t pc = (uint32_t)p[4] | ((uint32_t)p[5] << 8) | ((uint32_t)p[6] << 16) |
                ((uint32_t)p[7] << 24);

  bool sp_ok = (sp > BOOT_RAM_BASE_ADDRESS) && (sp <= (BOOT_RAM_BASE_ADDRESS + BOOT_RAM_SIZE)) &&
               ((sp & 0x3U) == 0U);
  bool pc_ok = ((pc & 0x1U) == 0x1U) && (pc >= BOOT_APP_START_ADDRESS) && (pc < BOOT_APP_END_ADDRESS);
  return sp_ok && pc_ok;
}

void BlUart_Send(const uint8_t *data, uint16_t len) {
  if (g_tx != NULL) {
    g_tx(data, len);
  }
}

/* ---- what fake_base.py calls --------------------------------------------- */

void sim_set_callbacks(sim_tx_fn tx, sim_event_fn ev) {
  g_tx = tx;
  g_event = ev;
}

uint8_t *sim_flash(void) { return g_flash; }

uint32_t sim_flash_size(void) { return SIM_FLASH_SIZE; }

/* bl_main.c main(), after take_boot_request(): `requested` is whether the
 * mailbox held BOOT_REQUEST_MAGIC. */
void sim_boot(int requested) {
  g_session.entered_by_request = requested != 0;
  g_session.app_valid = BlFlash_AppLooksValid();
  g_session.stay = g_session.entered_by_request || !g_session.app_valid;
  g_session.run_app = false;
  g_session.erased = false;
  g_session.erased_size = 0U;
  BlProtocol_Init(&g_session);
}

/* bit 0 entered_by_request, 1 app_valid, 2 stay, 3 run_app, 4 erased */
int sim_session(void) {
  return (g_session.entered_by_request ? 0x01 : 0) | (g_session.app_valid ? 0x02 : 0) |
         (g_session.stay ? 0x04 : 0) | (g_session.run_app ? 0x08 : 0) |
         (g_session.erased ? 0x10 : 0);
}

int sim_app_looks_valid(void) { return BlFlash_AppLooksValid() ? 1 : 0; }

void sim_fault_erase(int fail) { g_fail_erase = fail; }

void sim_fault_write(uint32_t offset) { g_fail_write_offset = offset; }

void sim_fault_flip(uint32_t offset, uint32_t bit) {
  g_flip_offset = offset;
  g_flip_bit = bit;
}
