#ifndef MODULE_INC_POWER_MANAGER_HPP_
#define MODULE_INC_POWER_MANAGER_HPP_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_MANAGER_SHUTDOWN_HOLD_MS 3000U
#define POWER_MANAGER_WAKE_HOLD_MS 1000U
#define POWER_MANAGER_WAKE_PULSE_MS 1000U

typedef enum {
  POWER_MANAGER_STATE_RUNNING = 0,
  POWER_MANAGER_STATE_LOW_POWER = 1,
  POWER_MANAGER_STATE_WAKE_PULSE = 2,
} power_manager_state_t;

typedef struct {
  power_manager_state_t state;
  bool button_pressed;
  bool main_power_enabled;
  bool lebancat_wake_asserted;
  uint32_t press_ms;
} power_manager_status_t;

void PowerManager_Init(void);
void PowerManager_Update10ms(void);
void PowerManager_RequestShutdown(void);
void PowerManager_WakeLebanCatPulse(uint32_t pulse_ms);
void PowerManager_GetStatus(power_manager_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_POWER_MANAGER_HPP_ */
