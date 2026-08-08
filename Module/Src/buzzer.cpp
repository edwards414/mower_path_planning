#include "buzzer.hpp"
#include "hardware_pins.hpp"

namespace {

buzzer_status_t g_status = {};
uint32_t g_beep_end_ms = 0U;

void configure_pin(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = Active_Buzzer_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Active_Buzzer_GPIO_Port, &gpio);
}

void write_buzzer(bool on) {
  HAL_GPIO_WritePin(Active_Buzzer_GPIO_Port, Active_Buzzer_Pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  g_status.active = on;
}

} // namespace

void Buzzer_Init(void) {
  configure_pin();
  g_status = {};
  g_beep_end_ms = 0U;
  write_buzzer(false);
}

void Buzzer_Set(bool on) {
  g_status.timed_beep_active = false;
  g_status.beep_remaining_ms = 0U;
  g_beep_end_ms = 0U;
  write_buzzer(on);
}

void Buzzer_Beep(uint32_t duration_ms) {
  if (duration_ms == 0U) {
    Buzzer_Set(false);
    return;
  }

  g_status.timed_beep_active = true;
  g_status.beep_remaining_ms = duration_ms;
  g_beep_end_ms = HAL_GetTick() + duration_ms;
  write_buzzer(true);
}

void Buzzer_Update10ms(void) {
  if (!g_status.timed_beep_active) {
    return;
  }

  uint32_t now = HAL_GetTick();
  if ((int32_t)(now - g_beep_end_ms) >= 0) {
    g_status.timed_beep_active = false;
    g_status.beep_remaining_ms = 0U;
    write_buzzer(false);
    return;
  }

  g_status.beep_remaining_ms = g_beep_end_ms - now;
}

void Buzzer_GetStatus(buzzer_status_t *status) {
  if (status == nullptr) {
    return;
  }
  *status = g_status;
}
