#ifndef MODULE_INC_STATUS_LIGHTS_HPP_
#define MODULE_INC_STATUS_LIGHTS_HPP_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STATUS_LIGHT_LEBANCAT_LED_INDEX 0U
#define STATUS_LIGHT_UART_LED_INDEX 1U
#define STATUS_LIGHT_SLEEP_ROS_LED_INDEX 2U
#define STATUS_LIGHT_BLINK_PERIOD_MS 500U

typedef struct {
  bool lebancat_network_ok;
  bool uart_ok;
  bool lebancat_sleeping;
  bool ros2_running;
} status_lights_state_t;

void StatusLights_Init(void);
void StatusLights_SetState(const status_lights_state_t *state);
void StatusLights_Update50ms(void);
void StatusLights_GetState(status_lights_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_STATUS_LIGHTS_HPP_ */
