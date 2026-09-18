#include "mg996_servo.hpp"
#include "hardware_pins.hpp"
#include "tim.h"

namespace {

mg996_servo_status_t g_status = {MG996_SERVO_CENTER_PULSE_US, 0U, 0U, 0U, 0U};
uint32_t g_last_command_ms = 0U;
/* Read by the TIM10 ISR on every update event: true = emit this pulse. */
volatile bool g_output_active = false;

uint16_t clamp_pulse(uint16_t pulse_us) {
  if (pulse_us < MG996_SERVO_MIN_PULSE_US) {
    return MG996_SERVO_MIN_PULSE_US;
  }
  if (pulse_us > MG996_SERVO_MAX_PULSE_US) {
    return MG996_SERVO_MAX_PULSE_US;
  }
  return pulse_us;
}

void pin_low(void) {
  HAL_GPIO_WritePin(MG996_PWM_GPIO_Port, MG996_PWM_Pin, GPIO_PIN_RESET);
}

/* Setters run from the UART parser task, the limit/timeout logic from the
 * board tick and the edges from the TIM10 ISR: guard the flag updates. */
uint32_t enter_critical(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void exit_critical(uint32_t primask) { __set_PRIMASK(primask); }

void configure_pin(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = MG996_PWM_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MG996_PWM_GPIO_Port, &gpio);
  pin_low();
}

/* Recompute whether pulses should go out and publish it to the ISR. A
 * pulse already in flight is cut short by pulling the pin low here; the
 * servo just sees one short pulse, which is harmless. */
void refresh_output(void) {
  bool active = ((g_status.flags & MG996_SERVO_FLAG_ENABLED) != 0U) &&
                ((g_status.flags & MG996_SERVO_FLAG_LIMIT_ACTIVE) == 0U);
  g_output_active = active;
  if (active) {
    g_status.flags |= MG996_SERVO_FLAG_OUTPUT_ACTIVE;
  } else {
    g_status.flags &= (uint8_t)~MG996_SERVO_FLAG_OUTPUT_ACTIVE;
    pin_low();
  }
}

} // namespace

void Mg996Servo_Init(void) {
  configure_pin();
  g_status.pulse_us = MG996_SERVO_CENTER_PULSE_US;
  g_status.hold_timeout_ms = 0U;
  g_status.command_age_ms = 0U;
  g_status.flags = 0U;
  g_status.last_rx_seq = 0U;
  g_last_command_ms = HAL_GetTick();
  g_output_active = false;

  /* MX_TIM10_Init (tim.c) has set 1 us ticks, a 20 ms period and CH1 in
   * output-compare "timing" mode (no pin). CCR1 is preloaded so a new
   * pulse width only takes effect at the next period edge and can never
   * skip the compare match of the pulse in progress. */
  TIM10->CCMR1 |= TIM_CCMR1_OC1PE;
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, g_status.pulse_us);
  __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
  __HAL_TIM_ENABLE_IT(&htim10, TIM_IT_UPDATE | TIM_IT_CC1);
  (void)HAL_TIM_Base_Start(&htim10);
}

void Mg996Servo_SetPulseUs(uint16_t pulse_us, uint16_t hold_timeout_ms,
                           uint8_t rx_seq) {
  uint32_t primask = enter_critical();
  g_status.pulse_us = clamp_pulse(pulse_us);
  g_status.hold_timeout_ms = hold_timeout_ms;
  g_status.last_rx_seq = rx_seq;
  g_last_command_ms = HAL_GetTick();
  g_status.flags |= MG996_SERVO_FLAG_ENABLED;
  g_status.flags &= (uint8_t)~MG996_SERVO_FLAG_TIMED_OUT;
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, g_status.pulse_us);
  refresh_output();
  exit_critical(primask);
}

void Mg996Servo_SetAngleDeg(float angle_deg) {
  if (angle_deg < 0.0f) {
    angle_deg = 0.0f;
  }
  if (angle_deg > 180.0f) {
    angle_deg = 180.0f;
  }

  float span =
      (float)(MG996_SERVO_MAX_PULSE_US - MG996_SERVO_MIN_PULSE_US);
  uint16_t pulse =
      (uint16_t)((float)MG996_SERVO_MIN_PULSE_US + (span * angle_deg / 180.0f));
  Mg996Servo_SetPulseUs(pulse, g_status.hold_timeout_ms,
                        g_status.last_rx_seq);
}

void Mg996Servo_SetLimitActive(bool active) {
  uint32_t primask = enter_critical();
  bool was_active = (g_status.flags & MG996_SERVO_FLAG_LIMIT_ACTIVE) != 0U;
  if (active != was_active) {
    if (active) {
      g_status.flags |= MG996_SERVO_FLAG_LIMIT_ACTIVE;
    } else {
      g_status.flags &= (uint8_t)~MG996_SERVO_FLAG_LIMIT_ACTIVE;
    }
    refresh_output();
  }
  exit_critical(primask);
}

void Mg996Servo_Disable(void) {
  uint32_t primask = enter_critical();
  g_status.flags &= (uint8_t)~MG996_SERVO_FLAG_ENABLED;
  refresh_output();
  exit_critical(primask);
}

void Mg996Servo_Update10ms(void) {
  uint32_t primask = enter_critical();
  uint32_t age = HAL_GetTick() - g_last_command_ms;
  g_status.command_age_ms = (age > 0xFFFFU) ? 0xFFFFU : (uint16_t)age;

  if (((g_status.flags & MG996_SERVO_FLAG_ENABLED) != 0U) &&
      (g_status.hold_timeout_ms != 0U) && (age >= g_status.hold_timeout_ms)) {
    g_status.flags &= (uint8_t)~MG996_SERVO_FLAG_ENABLED;
    g_status.flags |= MG996_SERVO_FLAG_TIMED_OUT;
    refresh_output();
  }
  exit_critical(primask);
}

void Mg996Servo_GetStatus(mg996_servo_status_t *status) {
  if (status == nullptr) {
    return;
  }
  uint32_t primask = enter_critical();
  *status = g_status;
  exit_critical(primask);
}

/* Runs at TIM10 priority. Update (counter wrap, every 20 ms) starts the
 * pulse, CH1 compare (CNT == pulse_us) ends it. The two never coincide
 * because pulse_us is clamped well below the period. */
void Mg996Servo_TimerIrq(void) {
  uint32_t sr = TIM10->SR;
  if ((sr & TIM_SR_UIF) != 0U) {
    TIM10->SR = ~TIM_SR_UIF;
    if (g_output_active) {
      HAL_GPIO_WritePin(MG996_PWM_GPIO_Port, MG996_PWM_Pin, GPIO_PIN_SET);
    }
  }
  if ((sr & TIM_SR_CC1IF) != 0U) {
    TIM10->SR = ~TIM_SR_CC1IF;
    pin_low();
  }
}
