#include "status_lights.hpp"
#include "ws2812.h"
#include "stm32f4xx_hal.h"

namespace {

status_lights_state_t g_state = {};

void set_pair(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  if (index >= LED_NUM) {
    return;
  }

  ws2812_set_led_buf(ws2812_buf_front, index, r, g, b);
  ws2812_set_led_buf(ws2812_buf_back, index, r, g, b);
}

} // namespace

void StatusLights_Init(void) { g_state = {}; }

void StatusLights_SetState(const status_lights_state_t *state) {
  if (state == nullptr) {
    return;
  }
  g_state = *state;
}

void StatusLights_Update50ms(void) {
  set_pair(STATUS_LIGHT_LEBANCAT_LED_INDEX, 0U,
           g_state.lebancat_network_ok ? 80U : 0U, 0U);
  set_pair(STATUS_LIGHT_UART_LED_INDEX, 0U, g_state.uart_ok ? 80U : 0U, 0U);

  bool show_yellow = false;
  if (g_state.lebancat_sleeping) {
    show_yellow =
        ((HAL_GetTick() / STATUS_LIGHT_BLINK_PERIOD_MS) % 2U) == 0U;
  } else if (g_state.ros2_running) {
    show_yellow = true;
  }

  set_pair(STATUS_LIGHT_SLEEP_ROS_LED_INDEX, show_yellow ? 90U : 0U,
           show_yellow ? 55U : 0U, 0U);
  ws2812_show_dual();
}

void StatusLights_GetState(status_lights_state_t *state) {
  if (state == nullptr) {
    return;
  }
  *state = g_state;
}
