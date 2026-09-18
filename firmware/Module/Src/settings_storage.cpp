#include "settings_storage.hpp"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>

namespace {

constexpr uint32_t SETTINGS_MAGIC = 0x4D575243UL; // "MWRC"
constexpr float WHEEL_DEFAULT_MAX_RPM = 58.0f;

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t size;
  uint32_t sequence;
  controller_settings_t settings;
  uint32_t crc32;
} settings_record_t;

static_assert((sizeof(settings_record_t) % sizeof(uint32_t)) == 0U,
              "settings record must be word aligned");

controller_settings_t g_settings;
settings_storage_status_t g_status = {};

float sanitize_float(float value, float fallback, float min_value,
                     float max_value) {
  if ((value != value) || (value < min_value) || (value > max_value)) {
    return fallback;
  }
  return value;
}

void sanitize_pid(pid_gains_t *pid, const pid_gains_t *fallback) {
  if ((pid == NULL) || (fallback == NULL)) {
    return;
  }

  pid->kp = sanitize_float(pid->kp, fallback->kp, 0.0f, 1000.0f);
  pid->ki = sanitize_float(pid->ki, fallback->ki, 0.0f, 1000.0f);
  pid->kd = sanitize_float(pid->kd, fallback->kd, 0.0f, 1000.0f);
  pid->integral_min =
      sanitize_float(pid->integral_min, fallback->integral_min, -1000.0f,
                     1000.0f);
  pid->integral_max =
      sanitize_float(pid->integral_max, fallback->integral_max, -1000.0f,
                     1000.0f);
  pid->output_min =
      sanitize_float(pid->output_min, fallback->output_min, -1000.0f, 1000.0f);
  pid->output_max =
      sanitize_float(pid->output_max, fallback->output_max, -1000.0f, 1000.0f);

  if (pid->integral_min > pid->integral_max) {
    pid->integral_min = fallback->integral_min;
    pid->integral_max = fallback->integral_max;
  }
  if (pid->output_min > pid->output_max) {
    pid->output_min = fallback->output_min;
    pid->output_max = fallback->output_max;
  }
}

void sanitize_settings(controller_settings_t *settings) {
  if (settings == NULL) {
    return;
  }

  controller_settings_t defaults;
  SettingsStorage_LoadDefaults(&defaults);

  sanitize_pid(&settings->left_wheel_pid, &defaults.left_wheel_pid);
  sanitize_pid(&settings->right_wheel_pid, &defaults.right_wheel_pid);
  settings->wheel_max_rpm =
      sanitize_float(settings->wheel_max_rpm, defaults.wheel_max_rpm, 1.0f,
                     300.0f);
  settings->closed_loop_enabled =
      (settings->closed_loop_enabled == 0U) ? 0U : 1U;
}

uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= (uint32_t)data;
  for (uint8_t bit = 0U; bit < 8U; ++bit) {
    if ((crc & 1UL) != 0UL) {
      crc = (crc >> 1) ^ 0xEDB88320UL;
    } else {
      crc >>= 1;
    }
  }
  return crc;
}

uint32_t crc32_compute(const void *data, uint32_t len) {
  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  uint32_t crc = 0xFFFFFFFFUL;

  if (bytes == NULL) {
    return 0UL;
  }

  for (uint32_t i = 0U; i < len; ++i) {
    crc = crc32_update(crc, bytes[i]);
  }
  return ~crc;
}

const settings_record_t *flash_record(void) {
  return reinterpret_cast<const settings_record_t *>(
      SETTINGS_STORAGE_FLASH_ADDRESS);
}

bool record_is_valid(const settings_record_t *record) {
  if (record == NULL) {
    return false;
  }

  if ((record->magic != SETTINGS_MAGIC) ||
      (record->version != SETTINGS_STORAGE_VERSION) ||
      (record->size != sizeof(settings_record_t))) {
    return false;
  }

  uint32_t expected =
      crc32_compute(record, offsetof(settings_record_t, crc32));
  return expected == record->crc32;
}

constexpr uint32_t FLASH_ERROR_FLAGS = FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                                       FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                                       FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR |
                                       FLASH_FLAG_RDERR;

HAL_StatusTypeDef erase_and_program(const settings_record_t *record) {
  FLASH_EraseInitTypeDef erase = {};
  uint32_t sector_error = 0U;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_7;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
  if (status != HAL_OK) {
    return status;
  }
  const uint32_t *words = reinterpret_cast<const uint32_t *>(record);
  uint32_t address = SETTINGS_STORAGE_FLASH_ADDRESS;
  for (uint32_t i = 0U; i < (sizeof(settings_record_t) / sizeof(uint32_t));
       ++i) {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, words[i]);
    if (status != HAL_OK) {
      return status;
    }
    address += sizeof(uint32_t);
  }
  return HAL_OK;
}

bool write_record_to_flash(const settings_record_t *record) {
  if (record == NULL) {
    return false;
  }

  if (HAL_FLASH_Unlock() != HAL_OK) {
    g_status.last_flash_error = 0xFFU;
    return false;
  }

  /* FLASH_SR error bits survive the bootloader's jump into the app (no
   * reset in between) and any stray write to a flash address; the HAL
   * refuses to start an erase while one is pending and only clears them on
   * that failed attempt, so the first save after boot would always fail.
   * Remember what was pending for the 0x84 diagnostics, then clear. */
  g_status.stale_flash_flags = (uint8_t)(FLASH->SR & 0xF2U);
  __HAL_FLASH_CLEAR_FLAG(FLASH_ERROR_FLAGS);

  HAL_StatusTypeDef status = erase_and_program(record);
  if (status != HAL_OK) {
    g_status.last_flash_error = (uint8_t)HAL_FLASH_GetError();
    __HAL_FLASH_CLEAR_FLAG(FLASH_ERROR_FLAGS);
    status = erase_and_program(record); /* one retry with a clean SR */
    if (status != HAL_OK) {
      g_status.last_flash_error = (uint8_t)HAL_FLASH_GetError();
    }
  }
  if (status == HAL_OK) {
    g_status.last_flash_error = 0U;
  }

  (void)HAL_FLASH_Lock();
  return status == HAL_OK;
}

void build_record(settings_record_t *record, const controller_settings_t *settings,
                  uint32_t sequence) {
  if ((record == NULL) || (settings == NULL)) {
    return;
  }

  memset(record, 0, sizeof(*record));
  record->magic = SETTINGS_MAGIC;
  record->version = SETTINGS_STORAGE_VERSION;
  record->size = sizeof(settings_record_t);
  record->sequence = sequence;
  record->settings = *settings;
  record->crc32 = crc32_compute(record, offsetof(settings_record_t, crc32));
}

} // namespace

void SettingsStorage_LoadDefaults(controller_settings_t *settings) {
  if (settings == NULL) {
    return;
  }

  settings->left_wheel_pid = {2.0f, 0.6f, 0.0f, -80.0f,
                              80.0f, -200.0f, 200.0f};
  settings->right_wheel_pid = {2.0f, 0.6f, 0.0f, -80.0f,
                               80.0f, -200.0f, 200.0f};
  settings->wheel_max_rpm = WHEEL_DEFAULT_MAX_RPM;
  settings->closed_loop_enabled = 1U;
}

void SettingsStorage_Init(void) {
  const settings_record_t *record = flash_record();
  if (record_is_valid(record)) {
    g_settings = record->settings;
    sanitize_settings(&g_settings);
    g_status.flash_valid = true;
    g_status.last_save_ok = true;
    g_status.sequence = record->sequence;
    return;
  }

  SettingsStorage_LoadDefaults(&g_settings);
  g_status.flash_valid = false;
  g_status.last_save_ok = false;
  g_status.sequence = 0U;
}

void SettingsStorage_GetSnapshot(controller_settings_t *settings) {
  if (settings == NULL) {
    return;
  }
  *settings = g_settings;
}

bool SettingsStorage_Update(const controller_settings_t *settings,
                            bool persist_to_flash) {
  if (settings == NULL) {
    return false;
  }

  controller_settings_t sanitized = *settings;
  sanitize_settings(&sanitized);
  g_settings = sanitized;

  if (!persist_to_flash) {
    return true;
  }

  settings_record_t record;
  build_record(&record, &g_settings, g_status.sequence + 1U);
  bool ok = write_record_to_flash(&record);
  g_status.last_save_ok = ok;
  if (ok) {
    g_status.flash_valid = true;
    g_status.sequence = record.sequence;
  }
  return ok;
}

void SettingsStorage_GetStatus(settings_storage_status_t *status) {
  if (status == NULL) {
    return;
  }
  *status = g_status;
}
