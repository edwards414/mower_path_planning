/* Just enough of the STM32F4 HAL for firmware/bootloader/Src/bl_flash.c to
 * compile on a host (boot_oracle.c includes it unmodified). The functions
 * are defined by the oracle, which records what the bootloader asks for. */
#ifndef BOOT_ORACLE_STUB_HAL_H_
#define BOOT_ORACLE_STUB_HAL_H_

#include <stdint.h>

typedef enum { HAL_OK = 0x00U, HAL_ERROR = 0x01U, HAL_BUSY = 0x02U, HAL_TIMEOUT = 0x03U } HAL_StatusTypeDef;

typedef struct {
  uint32_t TypeErase;
  uint32_t Banks;
  uint32_t Sector;
  uint32_t NbSectors;
  uint32_t VoltageRange;
} FLASH_EraseInitTypeDef;

#define FLASH_TYPEERASE_SECTORS 0x00000000U
#define FLASH_VOLTAGE_RANGE_3 0x00000002U
#define FLASH_TYPEPROGRAM_WORD 0x00000002U

#define FLASH_FLAG_EOP 0x00000001U
#define FLASH_FLAG_OPERR 0x00000002U
#define FLASH_FLAG_WRPERR 0x00000010U
#define FLASH_FLAG_PGAERR 0x00000020U
#define FLASH_FLAG_PGPERR 0x00000040U
#define FLASH_FLAG_PGSERR 0x00000080U
#define __HAL_FLASH_CLEAR_FLAG(flags) ((void)(flags))

HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *sector_error);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t data);

#endif /* BOOT_ORACLE_STUB_HAL_H_ */
