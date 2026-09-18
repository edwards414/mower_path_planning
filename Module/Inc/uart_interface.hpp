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
#define UART_FRAME_TYPE_PID_CONFIG_COMMAND 0x04U
#define UART_FRAME_TYPE_POWER_COMMAND 0x05U
/* 0x06 (INFO_REQUEST) and 0x87 (FIRMWARE_INFO) are taken by the build
 * identity frames of the mower_path_planning/firmware build; keep clear. */
#define UART_FRAME_TYPE_SERVO_COMMAND 0x07U
#define UART_FRAME_TYPE_MOTOR_STATUS 0x81U
#define UART_FRAME_TYPE_LAWER_MOTOR_STATUS 0x82U
#define UART_FRAME_TYPE_WS2812_STATUS 0x83U
#define UART_FRAME_TYPE_PID_CONFIG_STATUS 0x84U
#define UART_FRAME_TYPE_WHEEL_FEEDBACK_STATUS 0x85U
#define UART_FRAME_TYPE_POWER_STATUS 0x86U
#define UART_FRAME_TYPE_CHARGER_STATUS 0x89U
#define UART_FRAME_TYPE_SERVO_STATUS 0x88U
#define UART_FRAME_TYPE_ANALOG_STATUS 0x8AU
#define UART_MAX_PAYLOAD_SIZE 32U
#define UART_STATUS_PERIOD_MS 50U

#define UART_WS2812_MODE_CLEAR 0x00U
#define UART_WS2812_MODE_ALL_ON 0x01U
#define UART_WS2812_MODE_FLOW 0x02U
#define UART_WS2812_MODE_TURN_LEFT 0x03U
#define UART_WS2812_MODE_TURN_RIGHT 0x04U
#define UART_WS2812_MODE_SHOW 0x05U

/* 0x05 power command actions */
#define UART_POWER_ACTION_NONE 0x00U
#define UART_POWER_ACTION_HOST_SHUTDOWN_ACK 0x01U /* host is halting, cut after grace */
#define UART_POWER_ACTION_REQUEST_SHUTDOWN 0x02U  /* host wants the same flow as the button */
#define UART_POWER_ACTION_CANCEL_SHUTDOWN 0x03U
#define UART_POWER_ACTION_FORCE_POWER_OFF 0x04U   /* cut the rail now, no hand-shake */

/* 0x86 power status flags */
#define UART_POWER_STATUS_FLAG_BUTTON_PRESSED 0x01U
#define UART_POWER_STATUS_FLAG_MAIN_POWER_ENABLED 0x02U
#define UART_POWER_STATUS_FLAG_SHUTDOWN_REQUESTED 0x04U
#define UART_POWER_STATUS_FLAG_HOST_ACK_RECEIVED 0x08U
#define UART_POWER_STATUS_FLAG_WAKE_ASSERTED 0x10U

/* 0x88 servo status flags (mirror MG996_SERVO_FLAG_*) */
#define UART_SERVO_STATUS_FLAG_ENABLED 0x01U
#define UART_SERVO_STATUS_FLAG_LIMIT_ACTIVE 0x02U
#define UART_SERVO_STATUS_FLAG_OUTPUT_ACTIVE 0x04U
#define UART_SERVO_STATUS_FLAG_TIMED_OUT 0x08U

/* 0x8A analog status flags */
#define UART_ANALOG_STATUS_FLAG_MAIN_BATTERY_VALID 0x01U
#define UART_ANALOG_STATUS_FLAG_AON_BATTERY_VALID 0x02U
#define UART_ANALOG_STATUS_FLAG_BOARD_TEMP_VALID 0x04U
#define UART_ANALOG_STATUS_FLAG_MG996_CURRENT_VALID 0x08U
#define UART_ANALOG_STATUS_FLAG_VDDA_CALIBRATED 0x10U /* VREFINT read OK */
#define UART_ANALOG_STATUS_FLAG_MG996_LIMIT_ACTIVE 0x20U

/* 0x89 charger status flags */
#define UART_CHARGER_STATUS_FLAG_ONLINE 0x01U        /* RS485 replies OK */
#define UART_CHARGER_STATUS_FLAG_CHARGING 0x02U      /* Iout above threshold */
#define UART_CHARGER_STATUS_FLAG_CV_PHASE 0x04U      /* Vout at set CV */
#define UART_CHARGER_STATUS_FLAG_INPUT_PRESENT 0x08U /* Vin present */
#define UART_CHARGER_STATUS_FLAG_EVER_SEEN 0x10U     /* replied at least once */

#define UART_PID_STATUS_FLAG_CLOSED_LOOP_ENABLED 0x01U
#define UART_PID_STATUS_FLAG_FLASH_VALID 0x02U
#define UART_PID_STATUS_FLAG_LAST_SAVE_OK 0x04U
#define UART_PID_STATUS_FLAG_LAST_APPLY_OK 0x08U

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

typedef struct {
  float left_kp;
  float left_ki;
  float left_kd;
  float right_kp;
  float right_ki;
  float right_kd;
  uint8_t persist_to_flash;
  uint8_t closed_loop_enabled;
  uint16_t reserved;
} pid_config_payload_t;

typedef struct {
  float left_kp;
  float left_ki;
  float left_kd;
  float right_kp;
  float right_ki;
  float right_kd;
  uint8_t flags;
  uint8_t last_rx_seq;
  uint16_t reserved;
} pid_config_status_payload_t;

typedef struct __attribute__((packed)) {
  int16_t left_target_rpm_x100;
  int16_t left_measured_rpm_x100;
  int16_t right_target_rpm_x100;
  int16_t right_measured_rpm_x100;
  int16_t left_pid_output;
  int16_t right_pid_output;
  int32_t left_total_counts;  /* accumulated encoder counts since boot */
  int32_t right_total_counts;
  uint8_t flags;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
} wheel_feedback_status_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t action;
  uint8_t reserved0;
  uint16_t reserved1;
} power_command_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t state;           /* power_manager_state_t */
  uint8_t flags;           /* UART_POWER_STATUS_FLAG_* */
  uint8_t shutdown_reason; /* power_manager_shutdown_reason_t */
  uint8_t last_rx_seq;     /* seq of the last accepted 0x05 */
  uint16_t press_ms;
  uint16_t shutdown_elapsed_ms;
} power_status_payload_t;

typedef struct __attribute__((packed)) {
  uint16_t pulse_us;        /* 500-2500; 0 = stop pulses (servo goes limp) */
  uint16_t hold_timeout_ms; /* 0 = hold until the next command */
  uint16_t reserved0;
  uint16_t reserved1;
} servo_command_payload_t;

typedef struct __attribute__((packed)) {
  uint16_t pulse_us;        /* current target pulse */
  uint16_t hold_timeout_ms;
  uint16_t command_age_ms;  /* since the last 0x07, saturates */
  uint8_t flags;            /* UART_SERVO_STATUS_FLAG_* */
  uint8_t last_rx_seq;      /* seq of the last accepted 0x07 */
} servo_status_payload_t;

typedef struct __attribute__((packed)) {
  uint16_t main_battery_cv;   /* 24 V main battery, x0.01 V; 0 if invalid */
  uint16_t aon_battery_cv;    /* 3.7 V AON battery, x0.01 V; 0 if invalid */
  int16_t board_temp_dc;      /* NTC board temperature, x0.1 C; INT16_MIN if invalid */
  uint16_t vdda_mv;           /* ADC reference used for the conversions, mV */
  uint16_t mg996_current_raw; /* raw 12-bit ADC counts of the current sensor */
  uint8_t flags;              /* UART_ANALOG_STATUS_FLAG_* */
  uint8_t reserved;
} analog_status_payload_t;

typedef struct __attribute__((packed)) {
  uint16_t vin_cv;    /* input voltage, x0.01 V */
  uint16_t vout_cv;   /* output / battery voltage, x0.01 V */
  uint16_t iout_ca;   /* charge current, x0.01 A */
  uint16_t set_cc_ca; /* configured CC limit, x0.01 A */
  uint16_t set_cv_cv; /* configured CV limit, x0.01 V */
  uint8_t flags;      /* UART_CHARGER_STATUS_FLAG_* */
  uint8_t comm_error_count; /* wraps; timeouts + bad frames since boot */
  uint16_t age_ms;    /* since the last valid reply, 0xFFFF = never */
  uint8_t last_exception_code; /* Modbus exception from the last failed poll */
  uint8_t reserved;
} charger_status_payload_t;

extern volatile uint16_t uart_last_pos;
extern uint8_t uart_rx_dma[UART_RX_DMA_BUF_SIZE];
extern osMessageQueueId_t uartRxQueue;
extern osMessageQueueId_t uartTxQueue;
extern osThreadId_t uartTxTaskHandle;

void uart_server(void);
/* Push a 0x86 power status frame now (also sent every 50 ms). */
void uart_send_power_status(void);
/* Push a 0x89 charger status frame (also sent every 50 ms). */
void uart_send_charger_status(void);
/* Push a 0x88 servo status frame (also sent every 50 ms). */
void uart_send_servo_status(void);
/* Push a 0x8A analog status frame (also sent every 50 ms). */
void uart_send_analog_status(void);

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
