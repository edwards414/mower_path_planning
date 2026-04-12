/*
 * uart_interface.hpp
 *
 *  Created on: 2026年1月11日
 *      Author: fxrbindi
 */
#ifndef INC_UART_INTERFACE_HPP_
#define INC_UART_INTERFACE_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "motor.hpp"
#include "ws2812.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include <stdint.h>

#define UART_RX_DMA_BUF_SIZE 256U
#define UART_TX_QUEUE_LEN 16U
#define UART_TX_MAX_SIZE 128U

#define UART_FRAME_SOF_0 0xA5U
#define UART_FRAME_SOF_1 0x5AU
#define UART_PROTOCOL_VERSION 0x01U
#define UART_FRAME_TYPE_MOTOR_OPEN_LOOP_COMMAND 0x01U
#define UART_FRAME_TYPE_LAWER_MOTOR_COMMAND 0x02U
#define UART_FRAME_TYPE_WS2812_COMMAND 0x03U
#define UART_FRAME_TYPE_MOTOR_STATUS 0x81U
#define UART_FRAME_TYPE_LAWER_MOTOR_STATUS 0x82U
#define UART_FRAME_TYPE_WS2812_STATUS 0x83U
#define UART_MAX_PAYLOAD_SIZE 32U
#define UART_STATUS_PERIOD_MS 50U

#define UART_WS2812_MODE_CLEAR 0x00U
#define UART_WS2812_MODE_ALL_ON 0x01U
#define UART_WS2812_MODE_FLOW 0x02U
#define UART_WS2812_MODE_TURN_LEFT 0x03U
#define UART_WS2812_MODE_TURN_RIGHT 0x04U
#define UART_WS2812_MODE_SHOW 0x05U

typedef struct {
  uint16_t start;
  uint16_t len;
} UartChunk_t;

typedef struct {
  uint8_t data[UART_TX_MAX_SIZE];
  uint16_t len;
} UartMsg_t;

typedef struct __attribute__((packed)) {
  int16_t left_command_permille;
  int16_t right_command_permille;
  uint16_t command_timeout_ms;
  uint16_t reserved;
} motor_open_loop_command_payload_t;

typedef struct __attribute__((packed)) {
  int16_t commanded_left_permille;
  int16_t commanded_right_permille;
  int16_t applied_left_pwm;
  int16_t applied_right_pwm;
  uint16_t command_age_ms;
  uint8_t flags;
  uint8_t last_rx_seq;
} motor_status_payload_t;

typedef struct __attribute__((packed)) {
  int16_t command_permille;
  uint16_t command_timeout_ms;
  uint16_t reserved0;
  uint16_t reserved1;
} lawer_motor_command_payload_t;

typedef struct __attribute__((packed)) {
  int16_t commanded_permille;
  int16_t applied_pwm;
  uint16_t command_age_ms;
  uint8_t flags;
  uint8_t last_rx_seq;
} lawer_motor_status_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t mode;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t effect_period_ms;
  uint8_t reserved0;
  uint8_t reserved1;
} ws2812_command_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t mode;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t effect_period_ms;
  uint8_t flags;
  uint8_t last_rx_seq;
} ws2812_status_payload_t;

extern volatile uint16_t uart_last_pos;
extern uint8_t uart_rx_dma[UART_RX_DMA_BUF_SIZE];
extern osMessageQueueId_t uartRxQueue;
extern osMessageQueueId_t uartTxQueue;
extern osThreadId_t uartTxTaskHandle;

void uart_server(void);

void UartParserTask(void *arg);
void MotorTask(void *arg);
void UartTxTask(void *arg);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
int _write(int file, char *ptr, int len);

#ifdef __cplusplus
}
#endif

#endif
