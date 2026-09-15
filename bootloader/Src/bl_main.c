/*
 * UART bootloader for the mower STM32F411CE board.
 *
 * Lives in flash sectors 0-1 (0x08000000, 32 KB). Talks the same binary
 * framing as the application on USART1 (PB6 TX / PA10 RX, 115200 8N1) so the
 * host can reuse its serial link to update the application in sectors 2-6.
 *
 * Boot decision:
 *   1. app wrote BOOT_REQUEST_MAGIC to the RAM mailbox and reset  -> stay
 *   2. no valid application image                                 -> stay
 *   3. otherwise wait BOOT_GRACE_PERIOD_MS for a BL_PING; if none -> jump
 *
 * Runs from the 16 MHz HSI. No PLL, no interrupts except SysTick; UART is
 * polled. Everything is undone before jumping so the app starts from a
 * reset-like state.
 */
#include "bl_flash.h"
#include "bl_protocol.h"
#include "boot_shared.h"
#include "stm32f4xx_hal.h"

/* Motor-driver pins that must stay inactive while we sit in the bootloader.
 * Mirrors Core/Src/gpio.c in the app. */
#define BRAKE_GPIO_PORT GPIOC /* BLD120A BRK, open-drain, low = brake */
#define BRAKE_PIN GPIO_PIN_13
#define WHEEL_EN_GPIO_PORT GPIOA /* BTS7960 EN x4, low = disabled */
#define WHEEL_EN_PINS (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)

static UART_HandleTypeDef huart1;
static bl_session_t g_session;

/* ---- hardware ------------------------------------------------------------*/

static void gpio_init(void) {
  GPIO_InitTypeDef init = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Blade brake engaged (also lights the BlackPill LED on PC13). */
  HAL_GPIO_WritePin(BRAKE_GPIO_PORT, BRAKE_PIN, GPIO_PIN_RESET);
  init.Pin = BRAKE_PIN;
  init.Mode = GPIO_MODE_OUTPUT_OD;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BRAKE_GPIO_PORT, &init);

  /* Wheel drivers disabled. */
  HAL_GPIO_WritePin(WHEEL_EN_GPIO_PORT, WHEEL_EN_PINS, GPIO_PIN_RESET);
  init.Pin = WHEEL_EN_PINS;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(WHEEL_EN_GPIO_PORT, &init);
}

static void uart_init(void) {
  GPIO_InitTypeDef init = {0};

  __HAL_RCC_USART1_CLK_ENABLE();

  init.Mode = GPIO_MODE_AF_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  init.Alternate = GPIO_AF7_USART1;
  init.Pin = GPIO_PIN_10; /* PA10 RX */
  HAL_GPIO_Init(GPIOA, &init);
  init.Pin = GPIO_PIN_6; /* PB6 TX */
  HAL_GPIO_Init(GPIOB, &init);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200U;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  (void)HAL_UART_Init(&huart1);
}

void BlUart_Send(const uint8_t *data, uint16_t len) {
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 200U);
}

/* Poll the receiver; returns true and stores the byte when one is waiting. */
static bool uart_poll(uint8_t *byte) {
  uint32_t sr = USART1->SR;
  if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U) {
    /* Error flags are cleared by reading SR then DR. */
    (void)USART1->DR;
    return false;
  }
  if ((sr & USART_SR_RXNE) != 0U) {
    *byte = (uint8_t)(USART1->DR & 0xFFU);
    return true;
  }
  return false;
}

/* ---- boot request mailbox -------------------------------------------------*/

static bool take_boot_request(void) {
  bool requested = (*BOOT_SHARED_MAGIC_PTR == BOOT_REQUEST_MAGIC);
  *BOOT_SHARED_MAGIC_PTR = 0U;
  return requested;
}

/* ---- jump to application --------------------------------------------------*/

typedef void (*app_entry_t)(void);

static void jump_to_app(void) {
  uint32_t app_sp = *(volatile uint32_t *)BOOT_APP_START_ADDRESS;
  app_entry_t app_entry =
      (app_entry_t)(*(volatile uint32_t *)(BOOT_APP_START_ADDRESS + 4U));

  /* Let the RUN_APP acknowledgement leave the shift register. */
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
  }

  __disable_irq();

  (void)HAL_UART_DeInit(&huart1);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

  (void)HAL_RCC_DeInit();
  (void)HAL_DeInit();
  /* HAL_DeInit() force-resets AHB1, which puts every GPIO back to floating
   * input and would release the blade brake / wheel EN until the app's
   * MX_GPIO_Init runs. Re-assert them; the app leaves GPIO clocks on anyway. */
  gpio_init();

  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;

  for (uint32_t i = 0U; i < 8U; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

  SCB->VTOR = BOOT_APP_START_ADDRESS;
  __set_CONTROL(0U); /* privileged, MSP */
  __set_MSP(app_sp);
  __DSB();
  __ISB();
  __enable_irq(); /* PRIMASK clear, exactly like after a real reset */

  app_entry();

  for (;;) {
  }
}

/* ---- main ---------------------------------------------------------------*/

int main(void) {
  /* Read the mailbox first: nothing below may clobber it before we look. */
  bool requested = take_boot_request();

  HAL_Init();
  gpio_init();
  uart_init();

  g_session.entered_by_request = requested;
  g_session.app_valid = BlFlash_AppLooksValid();
  g_session.stay = requested || !g_session.app_valid;
  g_session.run_app = false;
  g_session.erased = false;
  g_session.erased_size = 0U;
  BlProtocol_Init(&g_session);

  uint32_t started = HAL_GetTick();

  for (;;) {
    uint8_t byte;
    while (uart_poll(&byte)) {
      BlProtocol_FeedByte(byte);
    }

    if (g_session.run_app) {
      jump_to_app();
    }

    if (!g_session.stay &&
        ((HAL_GetTick() - started) >= BOOT_GRACE_PERIOD_MS)) {
      jump_to_app();
    }
  }
}

/* HAL hooks ---------------------------------------------------------------*/

void SysTick_Handler(void) { HAL_IncTick(); }

void NMI_Handler(void) {
  for (;;) {
  }
}

void HardFault_Handler(void) {
  for (;;) {
  }
}

void MemManage_Handler(void) {
  for (;;) {
  }
}

void BusFault_Handler(void) {
  for (;;) {
  }
}

void UsageFault_Handler(void) {
  for (;;) {
  }
}

void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
