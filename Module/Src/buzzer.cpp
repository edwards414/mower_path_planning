#include "buzzer.hpp"
#include "hardware_pins.hpp"
#include "tim.h"

/* Passive buzzer on PB9 = TIM4_CH4. TIM4 runs at 2 kHz (shared with the
 * BLD120A PWM on CH3); a 50% duty on CH4 gives a 2 kHz tone, 0% is silent. */

namespace {

buzzer_status_t g_status = {};
uint32_t g_beep_end_ms = 0U;

void configure_pin(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = Buzzer_PWM_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(Buzzer_PWM_GPIO_Port, &gpio);

  TIM_OC_InitTypeDef oc = {};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &oc, BUZZER_TIM_CHANNEL);
  __HAL_TIM_SET_COMPARE(&htim4, BUZZER_TIM_CHANNEL, 0U);
  HAL_TIM_PWM_Start(&htim4, BUZZER_TIM_CHANNEL);
}

void write_buzzer(bool on) {
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
  __HAL_TIM_SET_COMPARE(&htim4, BUZZER_TIM_CHANNEL, on ? (period / 2U) : 0U);
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
