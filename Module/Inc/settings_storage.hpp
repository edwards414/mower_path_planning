#ifndef MODULE_INC_SETTINGS_STORAGE_HPP_
#define MODULE_INC_SETTINGS_STORAGE_HPP_

#include "pid_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SETTINGS_STORAGE_FLASH_ADDRESS 0x08060000UL
#define SETTINGS_STORAGE_FLASH_SECTOR 7U
#define SETTINGS_STORAGE_VERSION 1U

typedef struct {
  pid_gains_t left_wheel_pid;
  pid_gains_t right_wheel_pid;
  float wheel_max_rpm;
  uint32_t closed_loop_enabled;
} controller_settings_t;

typedef struct {
  bool flash_valid;
  bool last_save_ok;
  uint32_t sequence;
  /* Diagnostics for the 0x84 status frame. last_flash_error is the HAL
   * flash error code of the last failed save (HAL_FLASH_ERROR_*, 0 = none);
   * stale_flash_flags is FLASH_SR error bits found pending before the last
   * save started (they make the first erase fail unless cleared). */
  uint8_t last_flash_error;
  uint8_t stale_flash_flags;
} settings_storage_status_t;

void SettingsStorage_Init(void);
void SettingsStorage_LoadDefaults(controller_settings_t *settings);
void SettingsStorage_GetSnapshot(controller_settings_t *settings);
bool SettingsStorage_Update(const controller_settings_t *settings,
                            bool persist_to_flash);
void SettingsStorage_GetStatus(settings_storage_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_SETTINGS_STORAGE_HPP_ */
