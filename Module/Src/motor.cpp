/*
 * motor.c
 *
 *  Created on: 2025年12月27日
 *      Author: fxrbindi
 */
#include "motor.hpp"

#define MAX_RADPS 59.76f
#define PWM_MAX   200.0f
#define RADPS_TO_PWM (PWM_MAX / MAX_RADPS)


Motor motor_L;
Motor motor_R;
LawerMowerMoter Cutting_Motor;
void MowerMotor_Init()
{
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

    HAL_GPIO_WritePin(GPIOA, LL_Motor_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LR_Motor_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, RL_Motor_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, RR_Motor_EN_Pin, GPIO_PIN_RESET);
//    start define cutting motor
    Cutting_Motor.htim = &htim4;
    Cutting_Motor.channel = TIM_CHANNEL_1;
    Cutting_Motor.Dir_Port = GPIOB;
	Cutting_Motor.Dir_Pin = Lawer_Mower_Mower_Pin;

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

}

void Motor_SetSpeed(uint16_t motor_l_pwm, uint8_t L_dir,
                    uint16_t motor_r_pwm, uint8_t R_dir)
{
    // Enable both motors
    HAL_GPIO_WritePin(motor_L.EN_L_Port, motor_L.EN_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_L.EN_R_Port, motor_L.EN_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_R.EN_L_Port, motor_R.EN_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_R.EN_R_Port, motor_R.EN_R_Pin, GPIO_PIN_SET);

    // ---- Left Motor ----
    if (L_dir) {
        __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_L, motor_l_pwm);
        __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_R, 0);
    } else {
        __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_L, 0);
        __HAL_TIM_SET_COMPARE(motor_L.htim, motor_L.channel_R, motor_l_pwm);
    }

    // ---- Right Motor ----
    if (R_dir) {
        __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_L, motor_r_pwm);
        __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_R, 0);
    } else {
        __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_L, 0);
        __HAL_TIM_SET_COMPARE(motor_R.htim, motor_R.channel_R, motor_r_pwm);
    }
}
uint16_t radps_to_pwm(float omega)
{
    omega = fminf(fmaxf(omega, -MAX_RADPS), MAX_RADPS);
    return (uint16_t)(fabsf(omega) * RADPS_TO_PWM);
}

void Grass_cutting_motor(uint16_t pwm, uint8_t dir)
{
	if(dir == 1){
		HAL_GPIO_WritePin(Cutting_Motor.Dir_Port,Cutting_Motor.Dir_Pin , GPIO_PIN_SET);

	}
	else{
		HAL_GPIO_WritePin(Cutting_Motor.Dir_Port,Cutting_Motor.Dir_Pin , GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(Cutting_Motor.htim, Cutting_Motor.channel, pwm);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	RR_Motor_Alarm_Pin|RL_Motor_Alarm_Pin|LR_Motor_Alarm_Pin|LL_Motor_Alarm_Pin;
    if (GPIO_Pin == LL_Motor_Alarm_Pin)   // PA3
    {
        printf("LL_Motor_Alerm call\r\n");
    }
    else if (GPIO_Pin == LR_Motor_Alarm_Pin)   // PA3
    {
        printf("LR_Motor_Alerm call\r\n");
    }
    else if (GPIO_Pin == RL_Motor_Alarm_Pin)   // PA3
    {
        printf("RL_Motor_Alerm call\r\n");
    }
    else if (GPIO_Pin == RR_Motor_Alarm_Pin)   // PA3
    {
        printf("LR_Motor_Alerm call\r\n");
    }

}
