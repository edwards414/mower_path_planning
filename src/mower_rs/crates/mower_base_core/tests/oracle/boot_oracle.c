/* Oracle harness for mower_base_core::boot.
 *
 * Compiled against the firmware's own Module/Inc/boot_shared.h, and against
 * the unmodified bootloader/Src/bl_flash.c (through stub_hal/, whose
 * functions are defined below) for the sector table and the erase decision.
 * Prints the constants, sizeof / offsetof of every packed payload, the packed
 * bytes of sample values, the sector map and what BlFlash_EraseForImage
 * erases for a range of image sizes, as JSON on stdout. boot_generate.sh
 * adds zlib's CRC-32 vectors and writes tests/vectors/boot_oracle.json.
 */
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "bl_flash.c"

/* ---- the HAL the bootloader calls -------------------------------------- */

static int g_erase_calls;
static uint32_t g_erase_first;
static uint32_t g_erase_count;

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *sector_error) {
  g_erase_calls++;
  g_erase_first = erase->Sector;
  g_erase_count = erase->NbSectors;
  *sector_error = 0xFFFFFFFFU;
  return HAL_OK;
}

/* Never reached: the oracle does not call BlFlash_Write. */
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t data) {
  (void)type;
  (void)address;
  (void)data;
  return HAL_ERROR;
}

/* ---- JSON helpers ------------------------------------------------------ */

static int g_first;

static void sep(void) {
  if (!g_first) {
    printf(",");
  }
  g_first = 0;
  printf("\n");
}

static void hex(const void *p, size_t n) {
  const unsigned char *b = (const unsigned char *)p;
  printf("\"");
  for (size_t i = 0; i < n; ++i) {
    printf("%02x", b[i]);
  }
  printf("\"");
}

#define CONST(name)                                                            \
  do {                                                                         \
    sep();                                                                     \
    printf("    \"%s\": %lu", #name, (unsigned long)(name));                   \
  } while (0)

#define SIZE(type)                                                             \
  do {                                                                         \
    sep();                                                                     \
    printf("    \"%s\": %u", #type, (unsigned)sizeof(type));                   \
  } while (0)

#define OFFSET(type, field)                                                    \
  do {                                                                         \
    sep();                                                                     \
    printf("    \"%s.%s\": %u", #type, #field, (unsigned)offsetof(type, field)); \
  } while (0)

static void packed_begin(const char *type) {
  sep();
  printf("    {\"struct\": \"%s\", \"fields\": {", type);
}

static void packed_end(const void *p, size_t n) {
  printf("}, \"bytes\": ");
  hex(p, n);
  printf("}");
}

static void enter(uint32_t magic, uint32_t reserved) {
  boot_enter_payload_t v;
  memset(&v, 0, sizeof(v));
  v.magic = magic;
  v.reserved = reserved;
  packed_begin("boot_enter_payload_t");
  printf("\"magic\": %lu, \"reserved\": %lu", (unsigned long)v.magic, (unsigned long)v.reserved);
  packed_end(&v, sizeof(v));
}

static void enter_ack(uint8_t status, uint8_t r0, uint8_t r1, uint8_t r2) {
  boot_enter_ack_payload_t v;
  memset(&v, 0, sizeof(v));
  v.status = status;
  v.reserved[0] = r0;
  v.reserved[1] = r1;
  v.reserved[2] = r2;
  packed_begin("boot_enter_ack_payload_t");
  printf("\"status\": %u, \"reserved\": [%u, %u, %u]", v.status, v.reserved[0], v.reserved[1],
         v.reserved[2]);
  packed_end(&v, sizeof(v));
}

static void info(uint32_t start, uint32_t max, uint16_t chunk, uint8_t version, uint8_t flags,
                 uint32_t reserved) {
  boot_info_payload_t v;
  memset(&v, 0, sizeof(v));
  v.app_start_address = start;
  v.app_max_size = max;
  v.write_chunk_max = chunk;
  v.bootloader_version = version;
  v.flags = flags;
  v.reserved = reserved;
  packed_begin("boot_info_payload_t");
  printf("\"app_start_address\": %lu, \"app_max_size\": %lu, \"write_chunk_max\": %u, "
         "\"bootloader_version\": %u, \"flags\": %u, \"reserved\": %lu",
         (unsigned long)v.app_start_address, (unsigned long)v.app_max_size,
         (unsigned)v.write_chunk_max, v.bootloader_version, v.flags, (unsigned long)v.reserved);
  packed_end(&v, sizeof(v));
}

static void erase_payload(uint32_t size) {
  boot_erase_payload_t v;
  memset(&v, 0, sizeof(v));
  v.image_size = size;
  packed_begin("boot_erase_payload_t");
  printf("\"image_size\": %lu", (unsigned long)v.image_size);
  packed_end(&v, sizeof(v));
}

static void write_header(uint32_t offset) {
  boot_write_header_t v;
  memset(&v, 0, sizeof(v));
  v.offset = offset;
  packed_begin("boot_write_header_t");
  printf("\"offset\": %lu", (unsigned long)v.offset);
  packed_end(&v, sizeof(v));
}

static void verify(uint32_t size, uint32_t crc) {
  boot_verify_payload_t v;
  memset(&v, 0, sizeof(v));
  v.image_size = size;
  v.crc32 = crc;
  packed_begin("boot_verify_payload_t");
  printf("\"image_size\": %lu, \"crc32\": %lu", (unsigned long)v.image_size, (unsigned long)v.crc32);
  packed_end(&v, sizeof(v));
}

static void ack(uint8_t command, uint8_t status, uint32_t value) {
  boot_ack_payload_t v;
  memset(&v, 0, sizeof(v));
  v.command = command;
  v.status = status;
  v.value = value;
  packed_begin("boot_ack_payload_t");
  printf("\"command\": %u, \"status\": %u, \"value\": %lu", v.command, v.status,
         (unsigned long)v.value);
  packed_end(&v, sizeof(v));
}

int main(void) {
  printf("{\n  \"constants\": {");
  g_first = 1;
  CONST(BOOT_FLASH_BASE_ADDRESS);
  CONST(BOOT_BOOTLOADER_SIZE);
  CONST(BOOT_APP_START_ADDRESS);
  CONST(BOOT_APP_END_ADDRESS);
  CONST(BOOT_APP_MAX_SIZE);
  CONST(BOOT_APP_FIRST_SECTOR);
  CONST(BOOT_APP_LAST_SECTOR);
  CONST(BOOT_APP_VECTOR_OFFSET);
  CONST(BOOT_RAM_BASE_ADDRESS);
  CONST(BOOT_RAM_SIZE);
  CONST(BOOT_SHARED_RAM_ADDRESS);
  CONST(BOOT_SHARED_RAM_SIZE);
  CONST(BOOT_REQUEST_MAGIC);
  CONST(BOOT_VERSION);
  CONST(BOOT_GRACE_PERIOD_MS);
  CONST(BOOT_WRITE_CHUNK_MAX);
  CONST(BOOT_FRAME_TYPE_ENTER_BOOTLOADER);
  CONST(BOOT_FRAME_TYPE_ENTER_BOOTLOADER_ACK);
  CONST(BOOT_FRAME_TYPE_BL_PING);
  CONST(BOOT_FRAME_TYPE_BL_ERASE);
  CONST(BOOT_FRAME_TYPE_BL_WRITE);
  CONST(BOOT_FRAME_TYPE_BL_VERIFY);
  CONST(BOOT_FRAME_TYPE_BL_RUN_APP);
  CONST(BOOT_FRAME_TYPE_BL_INFO);
  CONST(BOOT_FRAME_TYPE_BL_ACK);
  CONST(BOOT_INFO_FLAG_APP_VALID);
  CONST(BOOT_INFO_FLAG_ENTERED_BY_REQUEST);
  CONST(BOOT_INFO_FLAG_ERASED);
  CONST(BOOT_STATUS_OK);
  CONST(BOOT_STATUS_BAD_ARGUMENT);
  CONST(BOOT_STATUS_FLASH_ERROR);
  CONST(BOOT_STATUS_CRC_MISMATCH);
  CONST(BOOT_STATUS_OUT_OF_RANGE);
  CONST(BOOT_STATUS_NOT_ERASED);
  CONST(BOOT_STATUS_NO_VALID_APP);
  printf("\n  },\n  \"sizes\": {");
  g_first = 1;
  SIZE(boot_enter_payload_t);
  SIZE(boot_enter_ack_payload_t);
  SIZE(boot_info_payload_t);
  SIZE(boot_erase_payload_t);
  SIZE(boot_write_header_t);
  SIZE(boot_verify_payload_t);
  SIZE(boot_ack_payload_t);
  printf("\n  },\n  \"offsets\": {");
  g_first = 1;
  OFFSET(boot_enter_payload_t, magic);
  OFFSET(boot_enter_payload_t, reserved);
  OFFSET(boot_enter_ack_payload_t, status);
  OFFSET(boot_enter_ack_payload_t, reserved);
  OFFSET(boot_info_payload_t, app_start_address);
  OFFSET(boot_info_payload_t, app_max_size);
  OFFSET(boot_info_payload_t, write_chunk_max);
  OFFSET(boot_info_payload_t, bootloader_version);
  OFFSET(boot_info_payload_t, flags);
  OFFSET(boot_info_payload_t, reserved);
  OFFSET(boot_erase_payload_t, image_size);
  OFFSET(boot_write_header_t, offset);
  OFFSET(boot_verify_payload_t, image_size);
  OFFSET(boot_verify_payload_t, crc32);
  OFFSET(boot_ack_payload_t, command);
  OFFSET(boot_ack_payload_t, status);
  OFFSET(boot_ack_payload_t, value);
  printf("\n  },\n  \"packed\": [");
  g_first = 1;
  enter(BOOT_REQUEST_MAGIC, 0U);
  enter(0x12345678U, 0x9ABCDEF0U);
  enter_ack(BOOT_STATUS_OK, 0U, 0U, 0U);
  enter_ack(BOOT_STATUS_NO_VALID_APP, 1U, 2U, 3U);
  info(BOOT_APP_START_ADDRESS, BOOT_APP_MAX_SIZE, BOOT_WRITE_CHUNK_MAX, BOOT_VERSION,
       BOOT_INFO_FLAG_APP_VALID | BOOT_INFO_FLAG_ERASED, 0U);
  info(0xFFFFFFFFU, 1U, 0xBEEFU, 0xFFU, 0xFFU, 0xA5A55A5AU);
  info(0U, 0U, 0U, 0U, BOOT_INFO_FLAG_ENTERED_BY_REQUEST, 0U);
  erase_payload(1U);
  erase_payload(BOOT_APP_MAX_SIZE);
  erase_payload(0xFFFFFFFFU);
  write_header(0U);
  write_header(0x80U);
  write_header(0x57F80U);
  verify(46440U, 0x0178F726U);
  verify(1U, 0xFFFFFFFFU);
  ack(BOOT_FRAME_TYPE_BL_WRITE, BOOT_STATUS_OK, 0x80U);
  ack(BOOT_FRAME_TYPE_BL_VERIFY, BOOT_STATUS_CRC_MISMATCH, 0xDEADBEEFU);
  ack(BOOT_FRAME_TYPE_BL_RUN_APP, BOOT_STATUS_OK, BOOT_APP_START_ADDRESS);
  ack(BOOT_FRAME_TYPE_BL_ERASE, BOOT_STATUS_OUT_OF_RANGE, BOOT_APP_MAX_SIZE);
  printf("\n  ],\n  \"sectors\": [");
  g_first = 1;
  for (size_t s = 0; s < sizeof(kSectors) / sizeof(kSectors[0]); ++s) {
    sep();
    printf("    {\"start\": %lu, \"size\": %lu}", (unsigned long)kSectors[s].start,
           (unsigned long)kSectors[s].size);
  }
  printf("\n  ],\n  \"erase\": [");
  g_first = 1;
  static const uint32_t sizes[] = {
      0U,        1U,        4U,           16383U,      16384U,      16385U,
      32768U,    32769U,    49152U,       98304U,      98305U,      229376U,
      229377U,   360444U,   BOOT_APP_MAX_SIZE, BOOT_APP_MAX_SIZE + 1U, 0x7FFFFFFFU, 0xFFFFFFFFU,
  };
  for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
    g_erase_calls = 0;
    g_erase_first = 0U;
    g_erase_count = 0U;
    bool ok = BlFlash_EraseForImage(sizes[i]);
    sep();
    printf("    {\"image_size\": %lu, \"ok\": %s, \"calls\": %d, \"first\": %lu, \"count\": %lu}",
           (unsigned long)sizes[i], ok ? "true" : "false", g_erase_calls,
           (unsigned long)g_erase_first, (unsigned long)g_erase_count);
  }
  printf("\n  ]\n}\n");
  return 0;
}
