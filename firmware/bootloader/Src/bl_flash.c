#include "bl_flash.h"
#include "boot_shared.h"
#include "stm32f4xx_hal.h"

/* STM32F411CE sector map (start address, size). */
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

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_PGSERR);

  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0U;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = BOOT_APP_FIRST_SECTOR;
  erase.NbSectors = last_sector - BOOT_APP_FIRST_SECTOR + 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
  (void)HAL_FLASH_Lock();
  return status == HAL_OK;
}

bool BlFlash_Write(uint32_t offset, const uint8_t *data, uint32_t len) {
  if ((data == 0) || (len == 0U) || ((len % 4U) != 0U) ||
      ((offset % 4U) != 0U) || (offset >= BOOT_APP_MAX_SIZE) ||
      (len > (BOOT_APP_MAX_SIZE - offset))) {
    return false;
  }

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }

  HAL_StatusTypeDef status = HAL_OK;
  uint32_t address = BOOT_APP_START_ADDRESS + offset;
  for (uint32_t i = 0U; i < len; i += 4U) {
    uint32_t word = (uint32_t)data[i] | ((uint32_t)data[i + 1U] << 8) |
                    ((uint32_t)data[i + 2U] << 16) |
                    ((uint32_t)data[i + 3U] << 24);
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word);
    if (status != HAL_OK) {
      break;
    }
    if (*(volatile uint32_t *)(address + i) != word) {
      status = HAL_ERROR;
      break;
    }
  }

  (void)HAL_FLASH_Lock();
  return status == HAL_OK;
}

uint32_t BlFlash_Crc32(uint32_t address, uint32_t len) {
  const uint8_t *bytes = (const uint8_t *)address;
  uint32_t crc = 0xFFFFFFFFUL;

  for (uint32_t i = 0U; i < len; ++i) {
    crc ^= (uint32_t)bytes[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 1UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
  }
  return ~crc;
}

bool BlFlash_AppLooksValid(void) {
  uint32_t sp = *(volatile uint32_t *)BOOT_APP_START_ADDRESS;
  uint32_t pc = *(volatile uint32_t *)(BOOT_APP_START_ADDRESS + 4U);

  bool sp_ok = (sp > BOOT_RAM_BASE_ADDRESS) &&
               (sp <= (BOOT_RAM_BASE_ADDRESS + BOOT_RAM_SIZE)) &&
               ((sp & 0x3U) == 0U);
  bool pc_ok = ((pc & 0x1U) == 0x1U) && (pc >= BOOT_APP_START_ADDRESS) &&
               (pc < BOOT_APP_END_ADDRESS);
  return sp_ok && pc_ok;
}
