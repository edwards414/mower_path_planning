#include "board_modules.hpp"
#include "analog_monitor.hpp"
#include "buzzer.hpp"
#include "charger_rs485.hpp"
#include "mg996_servo.hpp"
#include "power_manager.hpp"
#include "status_lights.hpp"
#include "uart_interface.hpp"
#include "stm32f4xx_hal.h"

namespace {

uint32_t g_last_analog_tick_ms = 0U;

} // namespace

void BoardModules_Init(void) {
  Buzzer_Init();
  AnalogMonitor_Init();
  ChargerRs485_Init();
  Mg996Servo_Init();
  PowerManager_Init();
  StatusLights_Init();
  g_last_analog_tick_ms = HAL_GetTick();
}

void BoardModules_Update10ms(void) {
  uint32_t now = HAL_GetTick();

  Buzzer_Update10ms();
  PowerManager_Update10ms();
  if (PowerManager_ConsumeStatusEvent()) {
    /* State changed (long press, ack, rail cut...): tell the host now
     * rather than on the next 50 ms status tick. */
    uart_send_power_status();
  }

  /* The charger module hangs off the main rail: stop polling it while the
   * rail is off so the AON battery is not spent driving a dead bus. */
  power_manager_status_t pm = {};
  PowerManager_GetStatus(&pm);
  ChargerRs485_SetEnabled(pm.main_power_enabled);
  ChargerRs485_Update10ms();

  /* Pulses come from the TIM10 interrupt; this only ages the hold timeout. */
  Mg996Servo_Update10ms();

  if ((now - g_last_analog_tick_ms) >= 200U) {
    AnalogMonitor_Update();

    analog_monitor_snapshot_t analog = {};
    AnalogMonitor_GetSnapshot(&analog);
    if (analog.adc_available) {
      Mg996Servo_SetLimitActive(analog.mg996_current_limit);
    }

    g_last_analog_tick_ms = now;
  }
}
