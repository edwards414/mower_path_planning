/*
 * motor.c
 *
 *  Created on: 2025年12月27日
 *      Author: fxrbindi
 */
#include "motor.hpp"

#define PWM_MAX 200.0f
#define CMD_TIMEOUT_MS 200

Motor motor_L;
Motor motor_R;
LawerMowerMoter Cutting_Motor;
volatile velocity_command_t m_velocity_cmd;

void MowerMotor_Init() {
  // start define wheel motor
  motor_L.htim = &htim2;
  motor_L.channel_L = TIM_CHANNEL_1;
  motor_L.channel_R = TIM_CHANNEL_2;
  motor_L.EN_L_Port = GPIOA;
  motor_L.EN_L_Pin = LL_Motor_EN_Pin;
  motor_L.EN_R_Port = GPIOA;
  motor_L.EN_R_Pin = LR_Motor_EN_Pin;

  motor_R.htim = &htim2;
  motor_R.channel_L = TIM_CHANNEL_3;
  motor_R.channel_R = TIM_CHANNEL_4;
  motor_R.EN_L_Port = GPIOA;
  motor_R.EN_L_Pin = RL_Motor_EN_Pin;
  motor_R.EN_R_Port = GPIOA;
  motor_R.EN_R_Pin = RR_Motor_EN_Pin;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(motor_L.EN_L_Port, motor_L.EN_L_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor_L.EN_R_Port, motor_L.EN_R_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor_R.EN_L_Port, motor_R.EN_L_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor_R.EN_R_Port, motor_R.EN_R_Pin, GPIO_PIN_SET);

  //    start define cutting motor
  Cutting_Motor.htim = &htim4;
  Cutting_Motor.channel = TIM_CHANNEL_1;
  Cutting_Motor.Dir_Port = GPIOB;
  Cutting_Motor.Dir_Pin = Lawer_Mower_Mower_Pin;

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  // Enable both motors
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

float clampPwm(float pwm) {
  if (pwm > PWM_MAX)
    return PWM_MAX;
  if (pwm < -PWM_MAX)
    return -PWM_MAX;
  return pwm;
}

void motor_set_right_pwm(float pwm) {
  pwm = clampPwm(pwm);
  // ---- Right Motor ----
  if (pwm >= 0.0f) {
    __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_L,
                          static_cast<uint32_t>(pwm));
    __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_R, 0);
  } else {
    __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_L, 0);
    __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_R,
                          static_cast<uint32_t>(-pwm));
  }
}

void motor_set_left_pwm(float pwm) {
  pwm = clampPwm(pwm);
  // ---- Right Motor ----
  if (pwm >= 0.0f) {
    __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_L,
                          static_cast<uint32_t>(pwm));
    __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_R, 0);
  } else {
    __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_L, 0);
    __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_R,
                          static_cast<uint32_t>(-pwm));
  }
}

void control_update_50hz(void) {
  uint32_t now = HAL_GetTick();

  int16_t target_right_;
  int16_t target_left_;

  if (!m_velocity_cmd.valid ||
      (now - m_velocity_cmd.last_update_ms) > CMD_TIMEOUT_MS) {
    target_left_ = 0;
    target_right_ = 0;
  } else {
    target_left_ = m_velocity_cmd.left_pwm;
    target_right_ = m_velocity_cmd.right_pwm;
  }

  motor_set_right_pwm(target_right_);
  motor_set_left_pwm(target_left_);
}

void Grass_cutting_motor(uint16_t pwm, uint8_t dir) {
  if (dir == 1) {
    HAL_GPIO_WritePin(Cutting_Motor.Dir_Port, Cutting_Motor.Dir_Pin,
                      GPIO_PIN_SET);

  } else {
    HAL_GPIO_WritePin(Cutting_Motor.Dir_Port, Cutting_Motor.Dir_Pin,
                      GPIO_PIN_RESET);
  }
  __HAL_TIM_SET_COMPARE(Cutting_Motor.htim, Cutting_Motor.channel, pwm);
}
