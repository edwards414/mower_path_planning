/*
 * ws2812.h
 *
 *  Created on: 2025年12月27日
 *      Author: fxrbindi
 */

#ifndef MODULE_INC_WS2812_H_
#define MODULE_INC_WS2812_H_

#include "string.h"
#include "tim.h"
#include "cmsis_os2.h"

#define Code1 80
#define Code0 40
#define CodeReset 0
#define RESET_LENGTH 50


#define LED_NUM_FRONT  32   /* PB5 / TIM3_CH2: 4 x 8 LEDs in series */
#define LED_NUM_BACK   16   /* PB4 / TIM3_CH1 */
#define LED_NUM        LED_NUM_BACK   /* animation index range (front is mapped 2:1) */
#define BITS_PER_LED  24
#define RESET_SLOTS   60
#define BUF_LEN_FRONT (LED_NUM_FRONT * BITS_PER_LED + RESET_SLOTS)
#define BUF_LEN_BACK  (LED_NUM_BACK * BITS_PER_LED + RESET_SLOTS)
#define BUF_LEN       BUF_LEN_BACK


void WS2812_Test(void);
void ws2812_set_led_buf(uint16_t *buf, int led, uint8_t r, uint8_t g, uint8_t b);
void ws2812_clear_all(void);
void ws2812_show_dual(void);
void ws2812_flow_dual(uint8_t r, uint8_t g, uint8_t b, int delay_ms);
void ws2812_turn_right_dual(int delay_ms);
void ws2812_turn_left_dual(int delay_ms);
void ws2812_all_on(uint8_t r, uint8_t g, uint8_t b);
//
extern uint16_t ws2812_buf_front[BUF_LEN_FRONT];  // CH2 = 前燈, 32 顆
extern uint16_t ws2812_buf_back[BUF_LEN_BACK];     // CH1 = 後燈, 16 顆
/* When non-NULL, ws2812_show_dual() clocks this buffer out on the back strip
 * instead of ws2812_buf_back, which stays the untouched base layer (the
 * rear-light overlay in led_effects.cpp composes into it). */
extern uint16_t *ws2812_back_override;

#endif /* MODULE_INC_WS2812_H_ */
