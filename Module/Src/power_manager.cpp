#include "power_manager.hpp"
#include "hardware_pins.hpp"
#include "motor.hpp"

namespace {

power_manager_status_t g_status = {};
uint32_t g_press_start_ms = 0U;
uint32_t g_wake_pulse_end_ms = 0U;

void configure_pins(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = POWER_BUTTON_N_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(POWER_BUTTON_N_GPIO_Port, &gpio);

  gpio = {};
  gpio.Pin = LEBANCAT_WAKE_Pin | MAIN_POWER_EN_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);
}

bool read_button_pressed(void) {
  return HAL_GPIO_ReadPin(POWER_BUTTON_N_GPIO_Port, POWER_BUTTON_N_Pin) ==
         GPIO_PIN_RESET;
}

void set_main_power(bool enabled) {
  HAL_GPIO_WritePin(MAIN_POWER_EN_GPIO_Port, MAIN_POWER_EN_Pin,
                    enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
  g_status.main_power_enabled = enabled;
}

void set_lebancat_wake(bool asserted) {
  HAL_GPIO_WritePin(LEBANCAT_WAKE_GPIO_Port, LEBANCAT_WAKE_Pin,
                    asserted ? GPIO_PIN_SET : GPIO_PIN_RESET);
  g_status.lebancat_wake_asserted = asserted;
}

} // namespace

void PowerManager_Init(void) {
  configure_pins();
  g_status = {};
  g_status.state = POWER_MANAGER_STATE_RUNNING;
  g_press_start_ms = 0U;
  g_wake_pulse_end_ms = 0U;
  set_main_power(true);
  set_lebancat_wake(false);
}

void PowerManager_Update10ms(void) {
  uint32_t now = HAL_GetTick();
  bool pressed = read_button_pressed();

  if (pressed && !g_status.button_pressed) {
    g_press_start_ms = now;
  }

  g_status.button_pressed = pressed;
  g_status.press_ms = pressed ? (now - g_press_start_ms) : 0U;

  if (g_status.state == POWER_MANAGER_STATE_WAKE_PULSE) {
    if ((int32_t)(now - g_wake_pulse_end_ms) >= 0) {
      set_lebancat_wake(false);
      g_status.state = POWER_MANAGER_STATE_RUNNING;
    }
    return;
  }

  if (!pressed) {
    return;
  }

  if ((g_status.state == POWER_MANAGER_STATE_RUNNING) &&
      (g_status.press_ms >= POWER_MANAGER_SHUTDOWN_HOLD_MS)) {
    PowerManager_RequestShutdown();
  } else if ((g_status.state == POWER_MANAGER_STATE_LOW_POWER) &&
             (g_status.press_ms >= POWER_MANAGER_WAKE_HOLD_MS)) {
    PowerManager_WakeLebanCatPulse(POWER_MANAGER_WAKE_PULSE_MS);
  }
}

void PowerManager_RequestShutdown(void) {
  motor_set_left_pwm(0.0f);
  motor_set_right_pwm(0.0f);
  Motor_SetWheelDriversEnabled(false);
  set_lebancat_wake(false);
  set_main_power(false);
  g_status.state = POWER_MANAGER_STATE_LOW_POWER;
}

void PowerManager_WakeLebanCatPulse(uint32_t pulse_ms) {
  if (pulse_ms == 0U) {
    pulse_ms = POWER_MANAGER_WAKE_PULSE_MS;
  }

  set_main_power(true);
  Motor_SetWheelDriversEnabled(true);
  set_lebancat_wake(true);
  g_wake_pulse_end_ms = HAL_GetTick() + pulse_ms;
  g_status.state = POWER_MANAGER_STATE_WAKE_PULSE;
}

void PowerManager_GetStatus(power_manager_status_t *status) {
  if (status == nullptr) {
    return;
  }
  *status = g_status;
}
