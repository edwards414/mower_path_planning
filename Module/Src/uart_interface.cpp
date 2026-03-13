#include "uart_interface.hpp"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint16_t uart_last_pos = 0;
uint8_t uart_rx_dma[UART_RX_DMA_BUF_SIZE];

void uart_server(void) {
  HAL_UART_Receive_DMA(&huart1, uart_rx_dma, UART_RX_DMA_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

bool parse_packet(uint8_t *buf, uint16_t len, Packet_t *pkt) {
  (void)len;

  if (!buf || !pkt)
    return false;
  if (buf[0] != '@')
    return false;

  char *p = (char *)buf + 1;
  printf("parse_packet command : %s ", buf);
  switch (*p) {
  case 'M':
    pkt->type = PKT_MOTOR;
    break;
  case 'L':
    pkt->type = PKT_LED;
    break;
  case 'G':
    pkt->type = PKT_LAWER_MOWER_MOTOR;
    break;
  // case 'P': pkt->type = PKT_PID;   break;
  default:
    return false;
  }

  p++; // skip command char

  pkt->argc = 0;

  while (*p && pkt->argc < 6) // 增加到6支持更多参数
  {
    while (*p == ' ')
      p++;
    if (*p == '\0' || *p == '\r' || *p == '\n')
      break;

    char *endp = NULL;
    int v = strtof(p, &endp);
    if (endp == p)
      break; // 轉換失敗
    pkt->argv[pkt->argc++] = v;
    p = endp;
  }

  // Motor 指令至少要 2 個參數 (L,R)
  if (pkt->type == PKT_MOTOR && pkt->argc < 2)
    return false;

  return true;
}

void UartParserTask(void *arg) {
  (void)arg;

  // 在排程器啟動後才啟動 DMA 接收 + IDLE 中斷
  // 確保 uartRxQueue 已建立，ISR 中的 osMessageQueuePut 才安全
  UartChunk_t chunk;
  uint8_t buf[UART_RX_DMA_BUF_SIZE + 1]; // +1 給字串結尾 0
  for (;;) {
    osMessageQueueGet(uartRxQueue, &chunk, NULL, osWaitForever);
    //     printf("uartparsertask is runnung\r\n"); /* 移除：避免高頻塞爆 TX
    //     queue
    //    // */ if (chunk.len > UART_RX_DMA_BUF_SIZE) continue;
    if (chunk.len == 0 || chunk.len > UART_RX_DMA_BUF_SIZE)
      continue;

    memcpy(buf, &uart_rx_dma[chunk.start], chunk.len);
    buf[chunk.len] = '\0'; // 安全終止
    //        HAL_UART_Transmit_DMA(&huart1, buf, chunk.len);

    printf("[UART RX] len=%d, data=\"%s\"\r\n", chunk.len, buf);
    Packet_t pkt;
    if (parse_packet(buf, (uint16_t)chunk.len, &pkt)) {
      if(osMessageQueuePut(dispatcherQueue, &pkt, 0, 0) != osOK)
      {
    	  printf("dispatcherQueue error \r\n");
      }
    }
  }
}

// 将 Packet_t 转换成 ws2812_msg_t
bool parse_led_command(Packet_t *pkt, ws2812_msg_t *led_msg) {
  if (!pkt || !led_msg)
    return false;
  if (pkt->type != PKT_LED)
    return false;

  // 参数格式: @L <cmd> <r> <g> <b> <delay_ms>
  // cmd: 0=clear, 1=all_on, 2=flow, 3=turn_left, 4=turn_right
  if (pkt->argc < 1)
    return false;

  int cmd = pkt->argv[0];

  switch (cmd) {
  case 0: // clear
    led_msg->cmd = WS2812_CMD_CLEAR;
    break;

  case 1: // all_on - 需要 r,g,b
    if (pkt->argc < 4)
      return false;
    led_msg->cmd = WS2812_CMD_ALL_ON;
    led_msg->r = pkt->argv[1];
    led_msg->g = pkt->argv[2];
    led_msg->b = pkt->argv[3];
    break;

  case 2: // flow - 需要 r,g,b,delay
    if (pkt->argc < 5)
      return false;
    led_msg->cmd = WS2812_CMD_FLOW;
    led_msg->r = pkt->argv[1];
    led_msg->g = pkt->argv[2];
    led_msg->b = pkt->argv[3];
    led_msg->delay_ms = pkt->argv[4];
    break;

  case 3: // turn_left - 需要 delay
    if (pkt->argc < 2)
      return false;
    led_msg->cmd = WS2812_CMD_TURN_LEFT;
    led_msg->delay_ms = pkt->argv[1];
    break;

  case 4: // turn_right - 需要 delay
    if (pkt->argc < 2)
      return false;
    led_msg->cmd = WS2812_CMD_TURN_RIGHT;
    led_msg->delay_ms = pkt->argv[1];
    break;

  default:
    return false;
  }

  return true;
}

void DispatcherTask(void *arg) {
  (void)arg;

  Packet_t pkt;

  for (;;) {
    osMessageQueueGet(dispatcherQueue, &pkt, NULL, osWaitForever);
    // */
    switch (pkt.type) {
    case PKT_MOTOR:
    	m_velocity_cmd.left_pwm = pkt.argv[0];
		m_velocity_cmd.right_pwm = pkt.argv[1];
		m_velocity_cmd.valid = true;
		m_velocity_cmd.last_update_ms = HAL_GetTick();
      break;

//    case PKT_LED: {
//      ws2812_msg_t led_msg;
//      if (parse_led_command(&pkt, &led_msg)) {
//        osMessageQueuePut(ledQueue, &led_msg, 0, 0);
//        printf("LED cmd=%d, r=%d, g=%d, b=%d, delay=%d\r\n", led_msg.cmd,
//               led_msg.r, led_msg.g, led_msg.b, led_msg.delay_ms);
//      }
//      break;
//    }
//    case PKT_LAWER_MOWER_MOTOR:
//      osMessageQueuePut(LawerMowerMotorQueue, &pkt, 0, 0);
//      printf("Lawn Mower cmd sent: pwm=%d\r\n", pkt.argv[0]);
//      break; // 修复：添加缺失的 break

    default:
      break;
    }
  }
}

void LawerMowerMotorTask(void *arg) {
  (void)arg;

  Packet_t pkt;

  printf("LawerMowerMotorTask started!\r\n");

  for (;;) {
    // 修复：从正确的队列获取数据
    osMessageQueueGet(LawerMowerMotorQueue, &pkt, NULL, osWaitForever);

    if (pkt.type != PKT_LAWER_MOWER_MOTOR) { //|| pkt.argc < 2
      printf("Lawn Mower: pkg type err (got %d)\r\n", pkt.type);
      continue;
    }

    if (pkt.argc < 1) {
      printf("Lawn Mower: insufficient arguments\r\n");
      continue;
    }

    int pwm = pkt.argv[0];

    printf("Lawn Mower: PWM=%d\r\n", pwm);

    uint8_t dir = (pwm >= 0) ? MOTOR_DIR_FWD : MOTOR_DIR_REV;

    uint16_t motorPWM = (uint16_t)fabsf(pwm);
    Grass_cutting_motor(motorPWM, dir);
  }
}

void MotorTask(void *arg) {
  (void)arg;

  printf("Motor task started!\r\n");
  for (;;) {
	  control_update_50hz();
	  osDelay(20);
  }
}

void LedTask(void *arg) {
  (void)arg;
  ws2812_msg_t msg;
  ws2812_msg_t current_mode; // 修复：改为结构体类型

  // 初始化为默认状态
  current_mode.cmd = WS2812_CMD_CLEAR;
  current_mode.r = 0;
  current_mode.g = 0;
  current_mode.b = 0;
  current_mode.delay_ms = 100;

  printf("LedTask started!\r\n");

  for (;;) {
    // 尝试获取新命令（非阻塞，10ms超时）
    osStatus_t status = osMessageQueueGet(ledQueue, &msg, 0, 10);

    if (status == osOK) {
      // 收到新命令，更新当前模式
      current_mode = msg;
      printf("LedTask received: cmd=%d, r=%d, g=%d, b=%d, delay=%d\r\n",
             msg.cmd, msg.r, msg.g, msg.b, msg.delay_ms);
    }

    // 根据当前模式执行相应的效果
    switch (current_mode.cmd) {
    case WS2812_CMD_CLEAR:
      // 清除模式：只在收到命令时执行一次
      if (status == osOK) {
        printf("LED: Clear\r\n");
        ws2812_clear_all();
        ws2812_show_dual();
      } else {
        osDelay(100); // 空闲状态，延迟一下
      }
      break;

    case WS2812_CMD_ALL_ON:
      // 全亮模式：只在收到命令时执行一次
      if (status == osOK) {
        printf("LED: All On\r\n");
        ws2812_all_on(current_mode.r, current_mode.g, current_mode.b);
      } else {
        osDelay(100); // 空闲状态，延迟一下
      }
      break;

    case WS2812_CMD_FLOW:
      // 流水灯模式：持续执行
      if (status == osOK) {
        printf("LED: Flow (continuous mode)\r\n");
      }
      ws2812_flow_dual(current_mode.r, current_mode.g, current_mode.b,
                       current_mode.delay_ms);
      break;

    case WS2812_CMD_TURN_LEFT:
      // 左转灯模式：持续执行
      if (status == osOK) {
        printf("LED: Turn Left (continuous mode)\r\n");
      }
      ws2812_turn_left_dual(current_mode.delay_ms);
      break;

    case WS2812_CMD_TURN_RIGHT:
      // 右转灯模式：持续执行
      if (status == osOK) {
        printf("LED: Turn Right (continuous mode)\r\n");
      }
      ws2812_turn_right_dual(current_mode.delay_ms);
      break;

    case WS2812_CMD_SHOW:
      // 显示命令：只在收到命令时执行一次
      if (status == osOK) {
        printf("LED: Show\r\n");
        ws2812_show_dual();
      } else {
        osDelay(100);
      }
      break;

    default:
      if (status == osOK) {
        printf("LED: Unknown command %d\r\n", current_mode.cmd);
      }
      osDelay(100);
      break;
    }
  }
}

void UartTxTask(void *arg) {
  (void)arg;

  UartMsg_t msg;

  for (;;) {
    if (osMessageQueueGet(uartTxQueue, &msg, NULL, osWaitForever) == osOK) {
      HAL_UART_Transmit_DMA(&huart1, msg.data, msg.len);
      // 等待 DMA 完成
      osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {

    osThreadFlagsSet(uartTxTaskHandle, 0x01);
  }
}

int _write(int file, char *ptr, int len) {
  // 检查是否在 FreeRTOS 运行中
  if (osKernelGetState() != osKernelRunning) {
    // 未启动或在中断中，使用阻塞发送
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 1000);
    return len;
  }

  UartMsg_t msg;

  if ((size_t)len > sizeof(msg.data))
    len = sizeof(msg.data);

  msg.len = len;
  memcpy(msg.data, ptr, len);

  // 使用超时而不是 osWaitForever
  if (osMessageQueuePut(uartTxQueue, &msg, 0, 0) != osOK) {
    // 队列满，降级到阻塞发送
//        HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
    return len;
  }

  return len;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    // Clear UART error flags
    __HAL_UART_CLEAR_OREFLAG(huart); // Overrun
    __HAL_UART_CLEAR_PEFLAG(huart);  // Parity
    __HAL_UART_CLEAR_FEFLAG(huart);  // Frame
    __HAL_UART_CLEAR_NEFLAG(huart);  // Noise
    printf("UART error Callback\n");
    // Optional: restart receive interrupt
    HAL_UART_Receive_DMA(&huart1, uart_rx_dma, UART_RX_DMA_BUF_SIZE);
  }
}
