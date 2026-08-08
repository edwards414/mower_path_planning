#include "mg996_servo.hpp"
#include "hardware_pins.hpp"

namespace {

mg996_servo_status_t g_status = {MG996_SERVO_CENTER_PULSE_US, 0U};

uint16_t clamp_pulse(uint16_t pulse_us) {
  if (pulse_us < MG996_SERVO_MIN_PULSE_US) {
    return MG996_SERVO_MIN_PULSE_US;
  }
  if (pulse_us > MG996_SERVO_MAX_PULSE_US) {
    return MG996_SERVO_MAX_PULSE_US;
  }
  return pulse_us;
}

void configure_pin(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = MG996_PWM_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MG996_PWM_GPIO_Port, &gpio);
  HAL_GPIO_WritePin(MG996_PWM_GPIO_Port, MG996_PWM_Pin, GPIO_PIN_RESET);
}

void enable_dwt_counter(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
  uint32_t cycles = (SystemCoreClock / 1000000U) * us;
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) {
  }
}

} // namespace

void Mg996Servo_Init(void) {
  configure_pin();
  enable_dwt_counter();
  g_status.pulse_us = MG996_SERVO_CENTER_PULSE_US;
  g_status.flags = 0U;
}

void Mg996Servo_SetPulseUs(uint16_t pulse_us) {
  g_status.pulse_us = clamp_pulse(pulse_us);
  g_status.flags |= MG996_SERVO_FLAG_ENABLED;
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
  Mg996Servo_SetPulseUs(pulse);
}

void Mg996Servo_SetLimitActive(bool active) {
  if (active) {
    g_status.flags |= MG996_SERVO_FLAG_LIMIT_ACTIVE;
    HAL_GPIO_WritePin(MG996_PWM_GPIO_Port, MG996_PWM_Pin, GPIO_PIN_RESET);
  } else {
    g_status.flags &= (uint8_t)~MG996_SERVO_FLAG_LIMIT_ACTIVE;
  }
}

void Mg996Servo_Disable(void) {
  g_status.flags &= (uint8_t)~MG996_SERVO_FLAG_ENABLED;
  HAL_GPIO_WritePin(MG996_PWM_GPIO_Port, MG996_PWM_Pin, GPIO_PIN_RESET);
}

void Mg996Servo_Update20ms(void) {
  if ((g_status.flags & MG996_SERVO_FLAG_ENABLED) == 0U) {
    HAL_GPIO_WritePin(MG996_PWM_GPIO_Port, MG996_PWM_Pin, GPIO_PIN_RESET);
    return;
  }
  if ((g_status.flags & MG996_SERVO_FLAG_LIMIT_ACTIVE) != 0U) {
    HAL_GPIO_WritePin(MG996_PWM_GPIO_Port, MG996_PWM_Pin, GPIO_PIN_RESET);
    return;
  }

  HAL_GPIO_WritePin(MG996_PWM_GPIO_Port, MG996_PWM_Pin, GPIO_PIN_SET);
  delay_us(g_status.pulse_us);
  HAL_GPIO_WritePin(MG996_PWM_GPIO_Port, MG996_PWM_Pin, GPIO_PIN_RESET);
}

void Mg996Servo_GetStatus(mg996_servo_status_t *status) {
  if (status == nullptr) {
    return;
  }
  *status = g_status;
}
