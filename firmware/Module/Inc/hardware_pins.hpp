#ifndef MODULE_INC_HARDWARE_PINS_HPP_
#define MODULE_INC_HARDWARE_PINS_HPP_

#include "stm32f4xx_hal.h"

/* PB9: passive buzzer, TIM4_CH4 PWM tone (2 kHz). */
#ifndef Buzzer_PWM_Pin
#define Buzzer_PWM_Pin GPIO_PIN_9
#define Buzzer_PWM_GPIO_Port GPIOB
#endif
#define BUZZER_TIM_CHANNEL TIM_CHANNEL_4

#ifndef MG996_PWM_Pin
#define MG996_PWM_Pin GPIO_PIN_10
#define MG996_PWM_GPIO_Port GPIOB
#endif

#ifndef ADC_MUX_S0_Pin
#define ADC_MUX_S0_Pin GPIO_PIN_2
#define ADC_MUX_S0_GPIO_Port GPIOB
#endif

#ifndef ADC_MUX_S1_Pin
#define ADC_MUX_S1_Pin GPIO_PIN_11
#define ADC_MUX_S1_GPIO_Port GPIOB
#endif

#ifndef POWER_BUTTON_N_Pin
#define POWER_BUTTON_N_Pin GPIO_PIN_0
#define POWER_BUTTON_N_GPIO_Port GPIOB
#endif

#ifndef LEBANCAT_WAKE_Pin
#define LEBANCAT_WAKE_Pin GPIO_PIN_14
#define LEBANCAT_WAKE_GPIO_Port GPIOC
#endif

#ifndef MAIN_POWER_EN_Pin
#define MAIN_POWER_EN_Pin GPIO_PIN_15
#define MAIN_POWER_EN_GPIO_Port GPIOC
#endif

#ifndef CAN_CS_Pin
#define CAN_CS_Pin GPIO_PIN_12
#define CAN_CS_GPIO_Port GPIOA
#endif

#ifndef CAN_INT_Pin
#define CAN_INT_Pin GPIO_PIN_11
#define CAN_INT_GPIO_Port GPIOA
#endif

/* PC13: BLD120A BRK, open-drain. Low = brake, high (high-Z) = release. */
#ifndef BLD120A_BRK_Pin
#define BLD120A_BRK_Pin GPIO_PIN_13
#define BLD120A_BRK_GPIO_Port GPIOC
#endif

#endif /* MODULE_INC_HARDWARE_PINS_HPP_ */
