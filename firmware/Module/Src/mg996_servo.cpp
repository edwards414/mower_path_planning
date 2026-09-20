#include "mg996_servo.hpp"
#include "hardware_pins.hpp"
#include "tim.h"

namespace {

mg996_servo_status_t g_status = {MG996_SERVO_CENTER_PULSE_US, 0U, 0U, 0U, 0U};
/* Where the host wants the pulse to end up; g_status.pulse_us slews here. */
uint16_t g_target_us = MG996_SERVO_CENTER_PULSE_US;
uint32_t g_last_command_ms = 0U;
/* Limit switch debounce: a reading has to repeat on two consecutive 10 ms
 * ticks before it becomes the debounced state. */
bool g_up_raw = false;
bool g_dn_raw = false;
bool g_up_pressed = false;
bool g_dn_pressed = false;
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

void configure_pins(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = MG996_PWM_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MG996_PWM_GPIO_Port, &gpio);
  pin_low();

  /* Limit switches close to GND; the internal pull-up idles them high. */
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Pin = SERVO_LIMIT_UP_Pin;
  HAL_GPIO_Init(SERVO_LIMIT_UP_GPIO_Port, &gpio);
  gpio.Pin = SERVO_LIMIT_DN_Pin;
  HAL_GPIO_Init(SERVO_LIMIT_DN_GPIO_Port, &gpio);
}

bool limit_pressed(GPIO_TypeDef *port, uint16_t pin) {
  return HAL_GPIO_ReadPin(port, pin) == SERVO_LIMIT_PRESSED_LEVEL;
}

/* Sample both switches; returns true when the debounced state changed. */
bool sample_limits(void) {
  bool up = limit_pressed(SERVO_LIMIT_UP_GPIO_Port, SERVO_LIMIT_UP_Pin);
  bool dn = limit_pressed(SERVO_LIMIT_DN_GPIO_Port, SERVO_LIMIT_DN_Pin);
  bool changed = false;
  if ((up == g_up_raw) && (up != g_up_pressed)) {
    g_up_pressed = up;
    changed = true;
  }
  if ((dn == g_dn_raw) && (dn != g_dn_pressed)) {
    g_dn_pressed = dn;
    changed = true;
  }
  g_up_raw = up;
  g_dn_raw = dn;
  return changed;
}

/* Is a move from the current pulse to `target` heading for the UP switch? */
bool moves_up(uint16_t target) {
#if MG996_SERVO_UP_IS_LONGER_PULSE
  return target > g_status.pulse_us;
#else
  return target < g_status.pulse_us;
#endif
}

/* Target that steps MG996_SERVO_LIMIT_BACKOFF_US away from the UP (or DOWN)
 * switch from the current pulse, clamped to the pulse range. */
uint16_t backoff_target(bool from_up) {
#if MG996_SERVO_UP_IS_LONGER_PULSE
  bool shorten = from_up;
#else
  bool shorten = !from_up;
#endif
  uint16_t pulse = g_status.pulse_us;
  if (shorten) {
    return (pulse > MG996_SERVO_MIN_PULSE_US + MG996_SERVO_LIMIT_BACKOFF_US)
               ? (uint16_t)(pulse - MG996_SERVO_LIMIT_BACKOFF_US)
               : MG996_SERVO_MIN_PULSE_US;
  }
  return (pulse < MG996_SERVO_MAX_PULSE_US - MG996_SERVO_LIMIT_BACKOFF_US)
             ? (uint16_t)(pulse + MG996_SERVO_LIMIT_BACKOFF_US)
             : MG996_SERVO_MAX_PULSE_US;
}

/* Refuse motion into a pressed switch: the target becomes a short back-off
 * away from it, so the servo settles just clear of the end stop instead of
 * leaning on it. LIMIT_ACTIVE tells the host its last target was cut
 * short; the next 0x07 clears it. Both switches pressed (travel shorter
 * than the back-off, or a wiring fault) holds the current pulse rather than
 * ping-ponging between the two back-offs. */
void apply_limits(void) {
  if (g_target_us == g_status.pulse_us) {
    return;
  }
  if (g_up_pressed && g_dn_pressed) {
    g_target_us = g_status.pulse_us;
    g_status.flags |= MG996_SERVO_FLAG_LIMIT_ACTIVE;
    return;
  }
  bool up = moves_up(g_target_us);
  if ((up && g_up_pressed) || (!up && g_dn_pressed)) {
    g_target_us = backoff_target(up);
    g_status.flags |= MG996_SERVO_FLAG_LIMIT_ACTIVE;
  }
}

/* One 10 ms step of the applied pulse towards the target. */
void slew_pulse(void) {
  uint16_t pulse = g_status.pulse_us;
  if (pulse < g_target_us) {
    uint16_t room = (uint16_t)(g_target_us - pulse);
    pulse = (uint16_t)(pulse + ((room < MG996_SERVO_SLEW_US_PER_10MS)
                                    ? room
                                    : MG996_SERVO_SLEW_US_PER_10MS));
  } else if (pulse > g_target_us) {
    uint16_t room = (uint16_t)(pulse - g_target_us);
    pulse = (uint16_t)(pulse - ((room < MG996_SERVO_SLEW_US_PER_10MS)
                                    ? room
                                    : MG996_SERVO_SLEW_US_PER_10MS));
  }
  if (pulse != g_status.pulse_us) {
    g_status.pulse_us = pulse;
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pulse);
  }
}

void publish_limit_flags(void) {
  g_status.flags &= (uint8_t)~(MG996_SERVO_FLAG_LIMIT_UP | MG996_SERVO_FLAG_LIMIT_DN);
  if (g_up_pressed) {
    g_status.flags |= MG996_SERVO_FLAG_LIMIT_UP;
  }
  if (g_dn_pressed) {
    g_status.flags |= MG996_SERVO_FLAG_LIMIT_DN;
  }
}

/* Recompute whether pulses should go out and publish it to the ISR. A
 * pulse already in flight is cut short by pulling the pin low here; the
 * servo just sees one short pulse, which is harmless. */
void refresh_output(void) {
  bool active = (g_status.flags & MG996_SERVO_FLAG_ENABLED) != 0U;
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
  configure_pins();
  g_status.pulse_us = MG996_SERVO_CENTER_PULSE_US;
  g_target_us = MG996_SERVO_CENTER_PULSE_US;
  g_status.hold_timeout_ms = 0U;
  g_status.command_age_ms = 0U;
  g_status.flags = 0U;
  g_status.last_rx_seq = 0U;
  g_last_command_ms = HAL_GetTick();
  g_output_active = false;
  /* Seed the debounce so a switch already pressed at boot counts at once. */
  g_up_raw = g_up_pressed =
      limit_pressed(SERVO_LIMIT_UP_GPIO_Port, SERVO_LIMIT_UP_Pin);
  g_dn_raw = g_dn_pressed =
      limit_pressed(SERVO_LIMIT_DN_GPIO_Port, SERVO_LIMIT_DN_Pin);
  publish_limit_flags();

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
  g_target_us = clamp_pulse(pulse_us);
  g_status.hold_timeout_ms = hold_timeout_ms;
  g_status.last_rx_seq = rx_seq;
  g_last_command_ms = HAL_GetTick();
  g_status.flags |= MG996_SERVO_FLAG_ENABLED;
  g_status.flags &=
      (uint8_t)~(MG996_SERVO_FLAG_TIMED_OUT | MG996_SERVO_FLAG_LIMIT_ACTIVE);
  /* Into a pressed switch: clamp now so the host sees it in the very next
   * status frame rather than after the first slew tick. */
  apply_limits();
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

void Mg996Servo_GetLimits(bool *up_pressed, bool *dn_pressed) {
  uint32_t primask = enter_critical();
  if (up_pressed != nullptr) {
    *up_pressed = g_up_pressed;
  }
  if (dn_pressed != nullptr) {
    *dn_pressed = g_dn_pressed;
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

  if (sample_limits()) {
    publish_limit_flags();
  }

  if (((g_status.flags & MG996_SERVO_FLAG_ENABLED) != 0U) &&
      (g_status.hold_timeout_ms != 0U) && (age >= g_status.hold_timeout_ms)) {
    g_status.flags &= (uint8_t)~MG996_SERVO_FLAG_ENABLED;
    g_status.flags |= MG996_SERVO_FLAG_TIMED_OUT;
    refresh_output();
  }

  /* Only move while pulses are going out: a released servo is not where
   * the pulse says, so slewing the number would just be fiction. */
  if (g_output_active) {
    apply_limits();
    slew_pulse();
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
