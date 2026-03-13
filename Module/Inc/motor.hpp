/*
 * motor.h
 *
 *  Created on: 2025年12月27日
 *      Author: fxrbindi
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include "gpio.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t channel_R;
  uint32_t channel_L;
  GPIO_TypeDef *EN_R_Port;
  uint16_t EN_R_Pin;
  GPIO_TypeDef *EN_L_Port;
  uint16_t EN_L_Pin;
} Motor;

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *Dir_Port;
  uint16_t Dir_Pin;
} LawerMowerMoter;

typedef struct {
  uint16_t left_pwm;
  uint16_t right_pwm;
  bool valid;
  uint32_t last_update_ms;

} velocity_command_t;

void MowerMotor_Init(void);

void control_update_50hz(void);

void motor_set_right_pwm(float pwm);
void motor_set_left_pwm(float pwm);
void Grass_cutting_motor(uint16_t pwm, uint8_t dir);

extern volatile velocity_command_t m_velocity_cmd;

extern Motor motor_L;
extern Motor motor_R;

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_HPP_ */
