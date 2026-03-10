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


#define LED_NUM        16
#define BITS_PER_LED  24
#define RESET_SLOTS   60
#define BUF_LEN (LED_NUM * BITS_PER_LED + RESET_SLOTS)


void WS2812_Test(void);
void ws2812_set_led_buf(uint16_t *buf, int led, uint8_t r, uint8_t g, uint8_t b);
void ws2812_clear_all(void);
void ws2812_show_dual(void);
void ws2812_flow_dual(uint8_t r, uint8_t g, uint8_t b, int delay_ms);
void ws2812_turn_right_dual(int delay_ms);
void ws2812_turn_left_dual(int delay_ms);
void ws2812_all_on(uint8_t r, uint8_t g, uint8_t b);
//
extern uint16_t ws2812_buf_front[BUF_LEN];  // CH2 = 前燈
extern uint16_t ws2812_buf_back[BUF_LEN];

#endif /* MODULE_INC_WS2812_H_ */
