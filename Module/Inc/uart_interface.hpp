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
#include "math.h"
#include "motor.hpp"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "ws2812.h"
#include <stdio.h>
#include <stdlib.h>

#define MOTOR_DIR_FWD 1
#define MOTOR_DIR_REV 0
#define UART_RX_DMA_BUF_SIZE 256

#define UART_TX_QUEUE_LEN 10
#define UART_TX_MAX_SIZE 128

typedef struct {
  uint16_t start;
  uint16_t len;
} UartChunk_t;

typedef enum {
  PKT_MOTOR,
  PKT_LED,
  PKT_LAWER_MOWER_MOTOR,
  PKT_PID,
  PKT_UNKNOWN
} PacketType_t;

typedef struct {
  PacketType_t type;
  uint8_t argc;
  int argv[6]; // 增加到6个参数支持更多LED命令
} Packet_t;

typedef struct {
  uint8_t data[UART_TX_MAX_SIZE];
  uint16_t len;
} UartMsg_t;

typedef enum {
  WS2812_CMD_CLEAR,
  WS2812_CMD_ALL_ON,
  WS2812_CMD_FLOW,
  WS2812_CMD_TURN_LEFT,
  WS2812_CMD_TURN_RIGHT,
  WS2812_CMD_SHOW
} ws2812_cmd_t;

typedef struct {
  ws2812_cmd_t cmd;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t delay_ms;
} ws2812_msg_t;

extern volatile uint16_t uart_last_pos;
extern uint8_t uart_rx_dma[UART_RX_DMA_BUF_SIZE];
extern osMessageQueueId_t uartRxQueue, dispatcherQueue, motorQueue, uartTxQueue,
    ledQueue, LawerMowerMotorQueue;
extern osThreadId_t uartTxTaskHandle;

void uart_server(void);

void UartParserTask(void *arg);
void DispatcherTask(void *arg);
void MotorTask(void *arg);
void UartTxTask(void *arg);
void LedTask(void *arg);
void LawerMowerMotorTask(void *arg);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
int _write(int file, char *ptr, int len);

#ifdef __cplusplus
}

#endif

#endif
