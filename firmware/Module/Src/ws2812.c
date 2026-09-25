/*
 * ws2812.c
 *
 *  Created on: 2025年12月27日
 *      Author: fxrbindi
 */

#include "ws2812.h"

// 外部 DMA handle（用於調試）
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;
extern DMA_HandleTypeDef hdma_tim3_ch2;

uint16_t ws2812_buf_front[BUF_LEN_FRONT];
uint16_t ws2812_buf_back[BUF_LEN_BACK];
uint16_t *ws2812_back_override = NULL;

void WS2812_Test(void) {
  static uint16_t data[24 + RESET_LENGTH] = {
      // 24位数据：8位绿色 + 8位红色 + 8位蓝色
      Code1, Code1, Code0, Code1, Code1,
      Code1, Code1, Code1, // 绿色 = 255
      Code0, Code0, Code0, Code0, Code0,
      Code0, Code0, Code0, // 红色 = 0
      Code1, Code1, Code1, Code1, Code0,
      Code1, Code1, Code1 // 蓝色 = 255
                          //
                          //			Code1,Code1,Code1,Code1,Code1,Code1,Code1,Code1,
                          //// 绿色 = 255
                          //			Code0,Code0,Code0,Code0,Code0,Code0,Code0,Code0,
                          //// 红色 = 0
                          //			Code1,Code0,Code1,Code0,Code1,Code1,Code0,Code0,
  };
  // 或者使用循环填充
  for (int i = 24; i < 24 + RESET_LENGTH; i++) {
    data[i] = CodeReset;
  }
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)data,
                        sizeof(data) / sizeof(uint16_t));
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)data,
                        sizeof(data) / sizeof(uint16_t));
}

void ws2812_set_led_buf(uint16_t *buf, int led, uint8_t r, uint8_t g,
                        uint8_t b) {
  int idx = led * 24;

  uint8_t color[3] = {g, r, b}; // GRB

  for (int c = 0; c < 3; c++) {
    for (int bit = 7; bit >= 0; bit--) {
      if (color[c] & (1 << bit))
        buf[idx++] = Code1;
      else
        buf[idx++] = Code0;
    }
  }
}

void ws2812_clear_all(void) {
  // 應該填充 Code0，而不是 0；最後的 reset slots 才設為 0
  for (int i = 0; i < BUF_LEN_FRONT; i++) {
    ws2812_buf_front[i] = (i < BUF_LEN_FRONT - RESET_SLOTS) ? Code0 : 0;
  }
  for (int i = 0; i < BUF_LEN_BACK; i++) {
    ws2812_buf_back[i] = (i < BUF_LEN_BACK - RESET_SLOTS) ? Code0 : 0;
  }
}

void ws2812_show_dual(void) {
  uint16_t *back =
      (ws2812_back_override != NULL) ? ws2812_back_override : ws2812_buf_back;

  // reset slot
  for (int i = 0; i < RESET_SLOTS; i++) {
    ws2812_buf_front[LED_NUM_FRONT * BITS_PER_LED + i] = 0;
    ws2812_buf_back[LED_NUM_BACK * BITS_PER_LED + i] = 0;
    back[LED_NUM_BACK * BITS_PER_LED + i] = 0;
  }

  // 先停止之前的 DMA 传输
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);

  // 短暂延迟确保停止完成
  for (volatile int i = 0; i < 1000; i++)
    ;

  // 確保定時器在運行
  if (!(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE))) {
    HAL_TIM_Base_Start(&htim3);
  }

  // 後燈 CH1
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)back,
                        BUF_LEN_BACK);

  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)ws2812_buf_front,
                        BUF_LEN_FRONT);
}

void ws2812_flow_dual(uint8_t r, uint8_t g, uint8_t b, int delay_ms) {
  static int pos = 0;

  // 清空
  ws2812_clear_all();

  // 前燈
  ws2812_set_led_buf(ws2812_buf_front, pos, r, g, b);

  // 後燈
  ws2812_set_led_buf(ws2812_buf_back, pos, r, g, b);

  // 顯示
  ws2812_show_dual();

  // 移動
  pos++;
  if (pos >= LED_NUM)
    pos = 0;

  osDelay(delay_ms);
}

void ws2812_turn_left_dual(int delay_ms) {
  static int pos = 0;

  ws2812_clear_all();

  // 前燈
  ws2812_set_led_buf(ws2812_buf_front, pos, 255, 120, 0);
  ws2812_set_led_buf(ws2812_buf_front, (pos - 1 + LED_NUM) % LED_NUM, 120, 60,
                     0);

  // 後燈
  ws2812_set_led_buf(ws2812_buf_back, pos, 255, 120, 0);
  ws2812_set_led_buf(ws2812_buf_back, (pos - 1 + LED_NUM) % LED_NUM, 120, 60,
                     0);

  ws2812_show_dual();

  pos++;
  if (pos >= LED_NUM)
    pos = 0;

  osDelay(delay_ms);
}

void ws2812_turn_right_dual(int delay_ms) {
  static int pos = LED_NUM - 1;

  ws2812_clear_all();

  // 前燈
  ws2812_set_led_buf(ws2812_buf_front, pos, 255, 120, 0);
  ws2812_set_led_buf(ws2812_buf_front, (pos + 1) % LED_NUM, 120, 60, 0);

  // 後燈
  ws2812_set_led_buf(ws2812_buf_back, pos, 255, 120, 0);
  ws2812_set_led_buf(ws2812_buf_back, (pos + 1) % LED_NUM, 120, 60, 0);

  ws2812_show_dual();

  pos--;
  if (pos < 0)
    pos = LED_NUM - 1;

  osDelay(delay_ms);
}

void ws2812_all_on(uint8_t r, uint8_t g, uint8_t b) {
  // 清空 buffer
  ws2812_clear_all();

  // 填滿每一顆 LED
  for (int i = 0; i < LED_NUM_FRONT; i++) {
    ws2812_set_led_buf(ws2812_buf_front, i, r, g, b);
  }
  for (int i = 0; i < LED_NUM_BACK; i++) {
    ws2812_set_led_buf(ws2812_buf_back, i, r, g, b);
  }

  // 顯示
  ws2812_show_dual();
}
