/*
 * motor.h
 *
 *  Created on: 2025年12月27日
 *      Author: fxrbindi
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include "tim.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel_R;
	uint32_t channel_L;
	GPIO_TypeDef* EN_R_Port;
	uint16_t EN_R_Pin;
	GPIO_TypeDef* EN_L_Port;
	uint16_t EN_L_Pin;
} Motor;


typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	GPIO_TypeDef* Dir_Port;
	uint16_t Dir_Pin;
} LawerMowerMoter;

void MowerMotor_Init(void);
void Motor_SetSpeed(uint16_t motor_l_pwm, uint8_t L_dir, uint16_t motor_r_pwm, uint8_t R_dir);
void Grass_cutting_motor(uint16_t pwm, uint8_t dir);
uint16_t radps_to_pwm(float omega);


extern Motor motor_L;
extern Motor motor_R;


#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_HPP_ */
