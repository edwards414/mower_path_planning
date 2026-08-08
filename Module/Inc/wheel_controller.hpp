#ifndef MODULE_INC_WHEEL_CONTROLLER_HPP_
#define MODULE_INC_WHEEL_CONTROLLER_HPP_

#include "settings_storage.hpp"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_CONTROLLER_CONTROL_PERIOD_MS 20U
#define WHEEL_CONTROLLER_ENCODER_COUNTS_PER_REV 8896.0f
#define WHEEL_CONTROLLER_COMMAND_DEADBAND_PERMILLE 5

#define WHEEL_CONTROLLER_FLAG_ENABLED 0x01U
#define WHEEL_CONTROLLER_FLAG_CLOSED_LOOP 0x02U
#define WHEEL_CONTROLLER_FLAG_FLASH_SETTINGS_VALID 0x04U
#define WHEEL_CONTROLLER_FLAG_LAST_SAVE_OK 0x08U

typedef struct {
  float target_rpm;
  float measured_rpm;
  float pid_output;
  int16_t applied_pwm;
  int32_t delta_counts;
  uint32_t raw_counter;
} wheel_controller_wheel_status_t;

typedef struct {
  wheel_controller_wheel_status_t left;
  wheel_controller_wheel_status_t right;
  uint8_t flags;
} wheel_controller_status_t;

void WheelController_Init(void);
void WheelController_Update20ms(int16_t left_command_permille,
                                int16_t right_command_permille,
                                bool output_enabled);
void WheelController_Stop(void);
void WheelController_GetStatus(wheel_controller_status_t *status);
void WheelController_GetSettings(controller_settings_t *settings);
bool WheelController_ApplySettings(const controller_settings_t *settings,
                                   bool persist_to_flash);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_WHEEL_CONTROLLER_HPP_ */
