/*
 * motor.h
 *
 *  Created on: 2025年12月27日
 *      Author: fxrbindi
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include <stdbool.h>
#include <stdint.h>
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
  int16_t left_command_permille;
  int16_t right_command_permille;
  uint16_t command_timeout_ms;
  bool valid;
  uint8_t last_rx_seq;
  uint32_t last_update_ms;
} motor_open_loop_command_t;

typedef struct {
  int16_t commanded_left_permille;
  int16_t commanded_right_permille;
  int16_t applied_left_pwm;
  int16_t applied_right_pwm;
  uint16_t command_age_ms;
  uint8_t flags;
  uint8_t last_rx_seq;
} motor_open_loop_status_t;

#define MOTOR_PWM_MAX_COUNTS 200
#define MOTOR_DEFAULT_COMMAND_TIMEOUT_MS 200U
#define MOTOR_COMMAND_PERMILLE_LIMIT 1000

#define MOTOR_STATUS_FLAG_COMMAND_VALID 0x01U
#define MOTOR_STATUS_FLAG_COMMAND_TIMEOUT 0x02U
#define MOTOR_STATUS_FLAG_DRIVER_ALARM 0x04U

void MowerMotor_Init(void);

void control_update_50hz(void);
void Motor_SetOpenLoopCommand(int16_t left_command_permille,
                              int16_t right_command_permille,
                              uint16_t command_timeout_ms, uint8_t rx_seq);
void Motor_GetStatusSnapshot(motor_open_loop_status_t *status);
bool Motor_HasDriverAlarm(void);
void Motor_SetWheelDriversEnabled(bool enabled);

void motor_set_right_pwm(float pwm);
void motor_set_left_pwm(float pwm);
void Grass_cutting_motor(uint16_t pwm, uint8_t dir);

extern volatile motor_open_loop_command_t g_motor_command;

extern Motor motor_L;
extern Motor motor_R;

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_HPP_ */
