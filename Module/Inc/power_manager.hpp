#ifndef MODULE_INC_POWER_MANAGER_HPP_
#define MODULE_INC_POWER_MANAGER_HPP_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Button timing */
#define POWER_MANAGER_SHUTDOWN_HOLD_MS 3000U
#define POWER_MANAGER_WAKE_HOLD_MS 1000U
#define POWER_MANAGER_WAKE_PULSE_MS 1000U

/* Shutdown hand-shake with the LubanCat.
 * On a shutdown request the STM32 raises SHUTDOWN_REQUESTED in the 0x86
 * power status at once and waits for the host to ack (0x05 action 1). After
 * the ack it still gives the host ACK_GRACE_MS to finish halting before the
 * main rail is cut. With no ack the rail is cut at HOST_TIMEOUT_MS anyway. */
#define POWER_MANAGER_HOST_ACK_GRACE_MS 8000U
#define POWER_MANAGER_HOST_TIMEOUT_MS 30000U
/* Upper bound on the final light fade before the rail is cut, in case the
 * WS2812 task is not running. */
#define POWER_MANAGER_LIGHTS_OFF_MAX_MS 1500U

typedef enum {
  POWER_MANAGER_STATE_RUNNING = 0,
  POWER_MANAGER_STATE_LOW_POWER = 1,
  POWER_MANAGER_STATE_WAKE_PULSE = 2,
  POWER_MANAGER_STATE_SHUTDOWN_PENDING = 3, /* host told, waiting to cut */
  POWER_MANAGER_STATE_LIGHTS_OFF = 4,       /* final fade, then cut */
} power_manager_state_t;

typedef enum {
  POWER_MANAGER_REASON_NONE = 0,
  POWER_MANAGER_REASON_BUTTON = 1,
  POWER_MANAGER_REASON_HOST = 2,
  POWER_MANAGER_REASON_FORCED = 3,
} power_manager_shutdown_reason_t;

typedef struct {
  power_manager_state_t state;
  power_manager_shutdown_reason_t shutdown_reason;
  bool button_pressed;
  bool main_power_enabled;
  bool lebancat_wake_asserted;
  bool shutdown_requested;
  bool host_ack_received;
  uint32_t press_ms;
  uint32_t shutdown_elapsed_ms; /* since the request, 0 when idle */
} power_manager_status_t;

void PowerManager_Init(void);
void PowerManager_Update10ms(void);

/* Begin the graceful shutdown: motors off, host notified, lights dim. */
void PowerManager_RequestShutdown(power_manager_shutdown_reason_t reason);
/* Host has finished (or is about to finish) its own shutdown. */
void PowerManager_HostShutdownAck(void);
/* Abort a pending shutdown while the host is still up. */
void PowerManager_CancelShutdown(void);
/* Cut the main rail now, no hand-shake (old behaviour). */
void PowerManager_ForcePowerOff(void);
void PowerManager_WakeLebanCatPulse(uint32_t pulse_ms);

void PowerManager_GetStatus(power_manager_status_t *status);
/* True while motion commands from the host should be applied. */
bool PowerManager_MotionAllowed(void);
/* True once per state change; the UART layer uses it to push a 0x86
 * status frame immediately instead of waiting for the 50 ms period. */
bool PowerManager_ConsumeStatusEvent(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_POWER_MANAGER_HPP_ */
