#ifndef MODULE_INC_CHARGER_RS485_HPP_
#define MODULE_INC_CHARGER_RS485_HPP_

/* RS485 (Modbus RTU) poller for the 数控 30V5A CC/CV charger module on
 * USART6 (PA11 TX / PA12 RX, 9600 8N1). Holding registers 0-4:
 *   0 Vin      x0.01 V   input (adapter) voltage
 *   1 Vout     x0.01 V   output = battery terminal voltage
 *   2 Iout     x0.01 A   charge current
 *   3 set CC   x0.01 A
 *   4 set CV   x0.01 V
 * Non-blocking: one FC03 request every CHARGER_RS485_POLL_PERIOD_MS, the
 * reply is collected by the USART6 RX-to-idle interrupt and parsed on the
 * next 10 ms tick. With CHARGER_RS485_USE_DE_PIN the MAX485 DE/RE pin is
 * raised for the request and dropped from the TX-complete interrupt; an
 * echoed request on RX (RE tied low) is tolerated by the frame scanner. */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHARGER_RS485_SLAVE_ADDR 0x01U
#define CHARGER_RS485_REG_COUNT 5U
#define CHARGER_RS485_POLL_PERIOD_MS 500U
#define CHARGER_RS485_REPLY_TIMEOUT_MS 200U
/* Consecutive failed polls before `online` drops. */
#define CHARGER_RS485_OFFLINE_AFTER_FAILS 3U
/* Iout at or above this counts as charging (x0.01 A). */
#define CHARGER_RS485_CHARGING_MIN_CA 5U
/* Vout within this of set CV counts as the CV (top-off) phase (x0.01 V). */
#define CHARGER_RS485_CV_BAND_CV 10U
/* Vin at or above this counts as "input present" (x0.01 V). */
#define CHARGER_RS485_INPUT_PRESENT_MIN_CV 500U

typedef struct {
  uint16_t vin_cv;
  uint16_t vout_cv;
  uint16_t iout_ca;
  uint16_t set_cc_ca;
  uint16_t set_cv_cv;
  bool online;          /* last poll(s) answered with a valid frame */
  bool ever_seen;       /* at least one valid reply since boot */
  uint32_t last_ok_ms;  /* HAL tick of the last valid reply */
  uint8_t comm_error_count;   /* wraps; timeouts + bad frames */
  uint8_t consecutive_fails;
  uint8_t last_exception_code;
} charger_rs485_snapshot_t;

void ChargerRs485_Init(void);
/* Drive the poll state machine; call from the 10 ms board tick. */
void ChargerRs485_Update10ms(void);
/* Pause/resume polling (e.g. while the main rail is off). Paused polling
 * keeps the last snapshot but marks it offline after the usual timeout. */
void ChargerRs485_SetEnabled(bool enabled);
void ChargerRs485_GetSnapshot(charger_rs485_snapshot_t *snapshot);
/* USART6 TX complete (last stop bit out): drop DE. Called from the HAL
 * TxCplt callback in uart_interface.cpp. */
void ChargerRs485_OnTxComplete(void);

/* Derived state helpers used by the 0x89 status frame. */
bool ChargerRs485_IsCharging(const charger_rs485_snapshot_t *s);
bool ChargerRs485_IsCvPhase(const charger_rs485_snapshot_t *s);
bool ChargerRs485_IsInputPresent(const charger_rs485_snapshot_t *s);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_CHARGER_RS485_HPP_ */
