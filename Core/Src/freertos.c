/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_interface.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t uartParserHandle, duspatcherHandle, motorHandle, uartTxTaskHandle,
    LedTaskHandle, LawerMowerMotorHandle;
osMessageQueueId_t uartRxQueue, dispatcherQueue, motorQueue, uartTxQueue,
    ledQueue, LawerMowerMotorQueue;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
const osThreadAttr_t uartParserAttr = {
    .name = "uart_parser", .priority = osPriorityHigh, .stack_size = 1024};

const osThreadAttr_t dispatcherAttr = {
    .name = "dispatcher", .priority = osPriorityHigh, .stack_size = 1024};

const osThreadAttr_t motorAttr = {
    .name = "motor", .priority = osPriorityNormal, .stack_size = 1024};

const osThreadAttr_t uartTxAttr = {
    .name = "uart_tx", .priority = osPriorityNormal, .stack_size = 2048};

const osThreadAttr_t ledAttr = {
    .name = "led", .priority = osPriorityNormal, .stack_size = 1024};

const osThreadAttr_t LawerMowerMotorAttr = {.name = "LawerMowerMotor",
                                            .priority = osPriorityNormal,
                                            .stack_size = 1024};

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  uartRxQueue = osMessageQueueNew(32, sizeof(UartChunk_t),
                                  NULL); /* 加大：16->32，處理 burst */
  dispatcherQueue =
      osMessageQueueNew(16, sizeof(Packet_t), NULL);         /* 加大：8->16 */
  motorQueue = osMessageQueueNew(8, sizeof(Packet_t), NULL); /* 加大：4->8 */
  LawerMowerMotorQueue =
      osMessageQueueNew(8, sizeof(Packet_t), NULL);            /* 加大：4->8 */
  ledQueue = osMessageQueueNew(8, sizeof(ws2812_msg_t), NULL); /* 加大：4->8 */

  uartTxQueue = osMessageQueueNew(UART_TX_QUEUE_LEN * 2, sizeof(UartMsg_t),
                                  NULL); /* 加大 TX queue */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//
  uartParserHandle = osThreadNew(UartParserTask, NULL, &uartParserAttr);
  duspatcherHandle = osThreadNew(DispatcherTask, NULL, &dispatcherAttr);
  motorHandle = osThreadNew(MotorTask, NULL, &motorAttr);
  uartTxTaskHandle = osThreadNew(UartTxTask, NULL, &uartTxAttr);
//  LedTaskHandle = osThreadNew(LedTask, NULL, &ledAttr);
//  LawerMowerMotorHandle = osThreadNew(LawerMowerMotorTask, NULL, &LawerMowerMotorAttr);


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	printf("start task \r\n");
  for (;;) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

