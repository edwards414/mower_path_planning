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

/* S1 lives on PA6: PB11 is not bonded out on the UFQFPN48 STM32F411CEU6
 * (Black Pill), so the original PB11 assignment could never toggle. */
#ifndef ADC_MUX_S1_Pin
#define ADC_MUX_S1_Pin GPIO_PIN_6
#define ADC_MUX_S1_GPIO_Port GPIOA
#endif

/* Which analog mux channels are actually populated, as a bit mask of
 * (1 << ANALOG_MONITOR_CHANNEL_*). A channel that is not populated is never
 * sampled and its 0x8A *_VALID flag stays clear, so a floating PB1 cannot
 * masquerade as a battery voltage or trip the MG996 current limit.
 * 2026-09-19: no mux, no dividers and no MG996 current sensor are fitted;
 * the pack voltage (and, once its shunt is wired, current) comes from the
 * RS485 meter (0x89). Set to 0x0F once the mux and dividers exist. */
#ifndef ANALOG_MONITOR_CHANNEL_MASK
#define ANALOG_MONITOR_CHANNEL_MASK 0x00U
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
 * PA6/PA7 are free since the SPI-CAN was dropped. */
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
