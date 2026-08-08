#ifndef MODULE_INC_ANALOG_MONITOR_HPP_
#define MODULE_INC_ANALOG_MONITOR_HPP_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANALOG_MONITOR_CHANNEL_COUNT 4U

typedef enum {
  ANALOG_MONITOR_CHANNEL_MG996_CURRENT = 0,
  ANALOG_MONITOR_CHANNEL_BOARD_TEMP = 1,
  ANALOG_MONITOR_CHANNEL_MAIN_BATTERY = 2,
  ANALOG_MONITOR_CHANNEL_AON_BATTERY = 3,
} analog_monitor_channel_t;

typedef struct {
  uint16_t raw[ANALOG_MONITOR_CHANNEL_COUNT];
  float adc_voltage[ANALOG_MONITOR_CHANNEL_COUNT];
  bool valid[ANALOG_MONITOR_CHANNEL_COUNT];
  float board_temperature_c;
  float main_battery_v;
  float aon_battery_v;
  bool mg996_current_limit;
  bool adc_available;
} analog_monitor_snapshot_t;

void AnalogMonitor_Init(void);
bool AnalogMonitor_ReadChannel(analog_monitor_channel_t channel,
                               uint16_t *raw);
void AnalogMonitor_Update(void);
void AnalogMonitor_SetMg996CurrentLimitRaw(uint16_t threshold_raw);
void AnalogMonitor_GetSnapshot(analog_monitor_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_ANALOG_MONITOR_HPP_ */
