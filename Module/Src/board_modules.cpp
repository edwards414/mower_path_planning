#include "board_modules.hpp"
#include "analog_monitor.hpp"
#include "buzzer.hpp"
#include "mg996_servo.hpp"
#include "power_manager.hpp"
#include "status_lights.hpp"
#include "stm32f4xx_hal.h"

namespace {

uint32_t g_last_servo_tick_ms = 0U;
uint32_t g_last_analog_tick_ms = 0U;

} // namespace

void BoardModules_Init(void) {
  Buzzer_Init();
  AnalogMonitor_Init();
  Mg996Servo_Init();
  PowerManager_Init();
  StatusLights_Init();
  g_last_servo_tick_ms = HAL_GetTick();
  g_last_analog_tick_ms = HAL_GetTick();
}

void BoardModules_Update10ms(void) {
  uint32_t now = HAL_GetTick();

  Buzzer_Update10ms();
  PowerManager_Update10ms();

  if ((now - g_last_servo_tick_ms) >= MG996_SERVO_PERIOD_MS) {
    Mg996Servo_Update20ms();
    g_last_servo_tick_ms = now;
  }

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
