/*
 * motor.c
 *
 *  Created on: 2025年12月27日
 *      Author: fxrbindi
 */
#include "motor.hpp"
#include "hardware_pins.hpp"
#include "wheel_controller.hpp"

#ifndef BTS7960_Motor_EN_Pin
#define BTS7960_Motor_EN_Pin GPIO_PIN_4
#define BTS7960_Motor_EN_GPIO_Port GPIOA
#endif

Motor motor_L;
Motor motor_R;
LawerMowerMoter Cutting_Motor;
volatile motor_open_loop_command_t g_motor_command;

namespace {
volatile motor_open_loop_status_t g_motor_status;

uint32_t motor_enter_critical() {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void motor_exit_critical(uint32_t primask) {
  if (primask == 0U) {
    __enable_irq();
  }
}

int16_t clamp_permille(int16_t command) {
  if (command > MOTOR_COMMAND_PERMILLE_LIMIT) {
    return MOTOR_COMMAND_PERMILLE_LIMIT;
  }
  if (command < -MOTOR_COMMAND_PERMILLE_LIMIT) {
    return -MOTOR_COMMAND_PERMILLE_LIMIT;
  }
  return command;
}

int16_t clamp_pwm(int16_t pwm) {
  if (pwm > MOTOR_PWM_MAX_COUNTS) {
    return MOTOR_PWM_MAX_COUNTS;
  }
  if (pwm < -MOTOR_PWM_MAX_COUNTS) {
    return -MOTOR_PWM_MAX_COUNTS;
  }
  return pwm;
}

uint16_t saturating_u16(uint32_t value) {
  if (value > 0xFFFFU) {
    return 0xFFFFU;
  }
  return (uint16_t)value;
}

motor_open_loop_command_t motor_load_command_snapshot(void) {
  motor_open_loop_command_t snapshot = {};
  snapshot.left_command_permille = g_motor_command.left_command_permille;
  snapshot.right_command_permille = g_motor_command.right_command_permille;
  snapshot.command_timeout_ms = g_motor_command.command_timeout_ms;
  snapshot.valid = g_motor_command.valid;
  snapshot.last_rx_seq = g_motor_command.last_rx_seq;
  snapshot.last_update_ms = g_motor_command.last_update_ms;
  return snapshot;
}

motor_open_loop_status_t motor_load_status_snapshot(void) {
  motor_open_loop_status_t snapshot = {};
  snapshot.commanded_left_permille = g_motor_status.commanded_left_permille;
  snapshot.commanded_right_permille = g_motor_status.commanded_right_permille;
  snapshot.applied_left_pwm = g_motor_status.applied_left_pwm;
  snapshot.applied_right_pwm = g_motor_status.applied_right_pwm;
  snapshot.command_age_ms = g_motor_status.command_age_ms;
  snapshot.flags = g_motor_status.flags;
  snapshot.last_rx_seq = g_motor_status.last_rx_seq;
  return snapshot;
}

void motor_store_status(const motor_open_loop_status_t *status) {
  if (status == NULL) {
    return;
  }

  g_motor_status.commanded_left_permille = status->commanded_left_permille;
  g_motor_status.commanded_right_permille = status->commanded_right_permille;
  g_motor_status.applied_left_pwm = status->applied_left_pwm;
  g_motor_status.applied_right_pwm = status->applied_right_pwm;
  g_motor_status.command_age_ms = status->command_age_ms;
  g_motor_status.flags = status->flags;
  g_motor_status.last_rx_seq = status->last_rx_seq;
}

void configure_shared_enable_pin(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = BTS7960_Motor_EN_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTS7960_Motor_EN_GPIO_Port, &gpio);
}
} // namespace

void MowerMotor_Init() {
  configure_shared_enable_pin();

  // start define wheel motor
  motor_L.htim = &htim2;
  motor_L.channel_L = TIM_CHANNEL_1;
  motor_L.channel_R = TIM_CHANNEL_2;
  motor_L.EN_L_Port = BTS7960_Motor_EN_GPIO_Port;
  motor_L.EN_L_Pin = BTS7960_Motor_EN_Pin;
  motor_L.EN_R_Port = BTS7960_Motor_EN_GPIO_Port;
  motor_L.EN_R_Pin = BTS7960_Motor_EN_Pin;

  motor_R.htim = &htim2;
  motor_R.channel_L = TIM_CHANNEL_3;
  motor_R.channel_R = TIM_CHANNEL_4;
  motor_R.EN_L_Port = BTS7960_Motor_EN_GPIO_Port;
  motor_R.EN_L_Pin = BTS7960_Motor_EN_Pin;
  motor_R.EN_R_Port = BTS7960_Motor_EN_GPIO_Port;
  motor_R.EN_R_Pin = BTS7960_Motor_EN_Pin;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  Motor_SetWheelDriversEnabled(true);

  //    start define cutting motor
  Cutting_Motor.htim = &htim4;
  Cutting_Motor.channel = TIM_CHANNEL_3;
  Cutting_Motor.Dir_Port = GPIOB;
  Cutting_Motor.Dir_Pin = Lawer_Mower_Mower_Pin;

  __HAL_TIM_SET_COMPARE(&htim4, Cutting_Motor.channel, 0);
  // Enable both motors
  HAL_TIM_PWM_Start(&htim4, Cutting_Motor.channel);

  WheelController_Init();

  g_motor_command.valid = false;
  g_motor_command.left_command_permille = 0;
  g_motor_command.right_command_permille = 0;
  g_motor_command.command_timeout_ms = MOTOR_DEFAULT_COMMAND_TIMEOUT_MS;
  g_motor_command.last_rx_seq = 0U;
  g_motor_command.last_update_ms = 0U;

  g_motor_status.commanded_left_permille = 0;
  g_motor_status.commanded_right_permille = 0;
  g_motor_status.applied_left_pwm = 0;
  g_motor_status.applied_right_pwm = 0;
  g_motor_status.command_age_ms = 0U;
  g_motor_status.flags = MOTOR_STATUS_FLAG_COMMAND_TIMEOUT;
  g_motor_status.last_rx_seq = 0U;
}

void motor_set_right_pwm(float pwm) {
  int16_t signed_pwm = clamp_pwm((int16_t)(pwm * MOTOR_RIGHT_DIRECTION_SIGN));
  // ---- Right Motor ----
  if (signed_pwm >= 0) {
    __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_L,
                          (uint32_t)signed_pwm);
    __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_R, 0);
  } else {
    __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_L, 0);
    __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_R,
                          (uint32_t)(-signed_pwm));
  }
}

void motor_set_left_pwm(float pwm) {
  int16_t signed_pwm = clamp_pwm((int16_t)(pwm * MOTOR_LEFT_DIRECTION_SIGN));
  // ---- Left Motor ----
  if (signed_pwm >= 0) {
    __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_L,
                          (uint32_t)signed_pwm);
    __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_R, 0);
  } else {
    __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_L, 0);
    __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_R,
                          (uint32_t)(-signed_pwm));
  }
}

bool Motor_HasDriverAlarm(void) {
  return (HAL_GPIO_ReadPin(RR_Motor_Alarm_GPIO_Port, RR_Motor_Alarm_Pin) ==
              GPIO_PIN_SET) ||
         (HAL_GPIO_ReadPin(RL_Motor_Alarm_GPIO_Port, RL_Motor_Alarm_Pin) ==
              GPIO_PIN_SET) ||
         (HAL_GPIO_ReadPin(LR_Motor_Alarm_GPIO_Port, LR_Motor_Alarm_Pin) ==
              GPIO_PIN_SET) ||
         (HAL_GPIO_ReadPin(LL_Motor_Alarm_GPIO_Port, LL_Motor_Alarm_Pin) ==
              GPIO_PIN_SET);
}

void Motor_SetWheelDriversEnabled(bool enabled) {
  HAL_GPIO_WritePin(BTS7960_Motor_EN_GPIO_Port, BTS7960_Motor_EN_Pin,
                    enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_SetOpenLoopCommand(int16_t left_command_permille,
                              int16_t right_command_permille,
                              uint16_t command_timeout_ms, uint8_t rx_seq) {
  uint32_t primask = motor_enter_critical();

  g_motor_command.left_command_permille = clamp_permille(left_command_permille);
  g_motor_command.right_command_permille =
      clamp_permille(right_command_permille);
  g_motor_command.command_timeout_ms =
      (command_timeout_ms == 0U) ? MOTOR_DEFAULT_COMMAND_TIMEOUT_MS
                                 : command_timeout_ms;
  g_motor_command.valid = true;
  g_motor_command.last_rx_seq = rx_seq;
  g_motor_command.last_update_ms = HAL_GetTick();

  motor_exit_critical(primask);
}

void Motor_GetStatusSnapshot(motor_open_loop_status_t *status) {
  if (status == NULL) {
    return;
  }

  uint32_t primask = motor_enter_critical();
  *status = motor_load_status_snapshot();
  motor_exit_critical(primask);
}

void control_update_50hz(void) {
  motor_open_loop_command_t command_snapshot = {};
  uint32_t primask = motor_enter_critical();
  command_snapshot = motor_load_command_snapshot();
  motor_exit_critical(primask);

  uint32_t now = HAL_GetTick();
  uint32_t command_age_ms =
      command_snapshot.valid ? (now - command_snapshot.last_update_ms) : 0xFFFFFFFFU;

  bool timeout = (!command_snapshot.valid) ||
                 (command_age_ms > command_snapshot.command_timeout_ms);
  bool alarm = Motor_HasDriverAlarm();

  bool output_enabled = !timeout;
#if MOTOR_ALARM_DISABLES_OUTPUT
  output_enabled = output_enabled && !alarm;
#endif
  WheelController_Update20ms(command_snapshot.left_command_permille,
                             command_snapshot.right_command_permille,
                             output_enabled);

  wheel_controller_status_t wheel_status = {};
  WheelController_GetStatus(&wheel_status);

  motor_open_loop_status_t status = {};
  status.commanded_left_permille =
      command_snapshot.valid ? command_snapshot.left_command_permille
                             : (int16_t)0;
  status.commanded_right_permille =
      command_snapshot.valid ? command_snapshot.right_command_permille
                             : (int16_t)0;
  status.applied_left_pwm = wheel_status.left.applied_pwm;
  status.applied_right_pwm = wheel_status.right.applied_pwm;
  status.command_age_ms = saturating_u16(command_age_ms);
  status.flags = 0U;
  status.last_rx_seq = command_snapshot.last_rx_seq;

  if (command_snapshot.valid) {
    status.flags |= MOTOR_STATUS_FLAG_COMMAND_VALID;
  }
  if (timeout) {
    status.flags |= MOTOR_STATUS_FLAG_COMMAND_TIMEOUT;
  }
  if (alarm) {
    status.flags |= MOTOR_STATUS_FLAG_DRIVER_ALARM;
  }

  primask = motor_enter_critical();
  motor_store_status(&status);
  motor_exit_critical(primask);
}

void Grass_cutting_motor(uint16_t pwm, uint8_t dir) {
  /* BLD120A EN is hardwired to COM; BRK is the only fast stop. Brake whenever
   * the commanded duty is 0 (includes timeout), release before applying PWM. */
  if (pwm == 0U) {
    __HAL_TIM_SET_COMPARE(Cutting_Motor.htim, Cutting_Motor.channel, 0U);
    HAL_GPIO_WritePin(BLD120A_BRK_GPIO_Port, BLD120A_BRK_Pin, GPIO_PIN_RESET);
    return;
  }
  HAL_GPIO_WritePin(BLD120A_BRK_GPIO_Port, BLD120A_BRK_Pin, GPIO_PIN_SET);
  /* Single-direction blade (2026-09-10): this motor/driver only runs with F/R
   * shorted to COM (PB7 low). The other direction is dead (hall/phase mapping
   * to be checked), so dir == 1 (forward) drives PB7 low and dir == 0 is
   * refused by braking instead of running the dead direction. */
  if (dir == 1) {
    HAL_GPIO_WritePin(Cutting_Motor.Dir_Port, Cutting_Motor.Dir_Pin,
                      GPIO_PIN_RESET);
  } else {
    __HAL_TIM_SET_COMPARE(Cutting_Motor.htim, Cutting_Motor.channel, 0U);
    HAL_GPIO_WritePin(BLD120A_BRK_GPIO_Port, BLD120A_BRK_Pin, GPIO_PIN_RESET);
    return;
  }
  __HAL_TIM_SET_COMPARE(Cutting_Motor.htim, Cutting_Motor.channel, pwm);
}
