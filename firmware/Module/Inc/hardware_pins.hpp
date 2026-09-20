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

/* MG996 travel limit microswitches, one per end of the mechanism. PB2 and
 * PA6 used to be the analog mux select lines; the mux, the dividers and the
 * MG996 current sensor were never fitted (the pack voltage comes from the
 * RS485 meter, 0x89), so the pins are inputs now. Each switch sits between
 * its pin and GND and the internal pull-up does the rest: no 5 V anywhere
 * near the switch. PB1 (the old ADC_MUX_OUT) is spare. */
#ifndef SERVO_LIMIT_UP_Pin
#define SERVO_LIMIT_UP_Pin GPIO_PIN_2
#define SERVO_LIMIT_UP_GPIO_Port GPIOB
#endif
#ifndef SERVO_LIMIT_DN_Pin
#define SERVO_LIMIT_DN_Pin GPIO_PIN_6
#define SERVO_LIMIT_DN_GPIO_Port GPIOA
#endif
/* Pin level while the switch is pressed. NO contact to GND = RESET; wire an
 * NC contact instead (broken wire reads as "pressed") and set this to SET. */
#ifndef SERVO_LIMIT_PRESSED_LEVEL
#define SERVO_LIMIT_PRESSED_LEVEL GPIO_PIN_RESET
#endif
/* 1 = a longer pulse drives the mechanism towards the UP switch. Flip to 0
 * if the horn is mounted the other way round. */
#ifndef MG996_SERVO_UP_IS_LONGER_PULSE
#define MG996_SERVO_UP_IS_LONGER_PULSE 1
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

/* PA11/PA12: USART6 TX/RX to the MAX485 TTL module (DI / RO).
 * PA5: MAX485 DE and RE tied together. High = drive the bus, low = listen.
 * Set CHARGER_RS485_USE_DE_PIN to 0 for an auto-direction transceiver.
 * PA7 is free since the SPI-CAN was dropped. */
#define CHARGER_RS485_USE_DE_PIN 1
#ifndef RS485_DE_Pin
#define RS485_DE_Pin GPIO_PIN_5
#define RS485_DE_GPIO_Port GPIOA
#endif

/* PC13: BLD120A BRK, open-drain. Low = brake, high (high-Z) = release. */
#ifndef BLD120A_BRK_Pin
#define BLD120A_BRK_Pin GPIO_PIN_13
#define BLD120A_BRK_GPIO_Port GPIOC
#endif

#endif /* MODULE_INC_HARDWARE_PINS_HPP_ */
