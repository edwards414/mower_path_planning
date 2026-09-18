#include "analog_monitor.hpp"
#include "hardware_pins.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_adc.h" /* VREFINT_CAL_ADDR / VREFINT_CAL_VREF */
#include <math.h>

#ifdef HAL_ADC_MODULE_ENABLED
extern "C" ADC_HandleTypeDef hadc1 __attribute__((weak));
#endif

namespace {

constexpr float ADC_REF_V = 3.3f; /* nominal; replaced by VREFINT reading */
constexpr float ADC_FULL_SCALE = 4095.0f;
constexpr float VDDA_MIN_PLAUSIBLE_V = 2.4f;
constexpr float VDDA_MAX_PLAUSIBLE_V = 3.6f;
constexpr float MAIN_BATTERY_DIVIDER_GAIN = (270000.0f + 33000.0f) / 33000.0f;
constexpr float AON_BATTERY_DIVIDER_GAIN = (100000.0f + 300000.0f) / 300000.0f;
constexpr float NTC_PULLUP_OHMS = 10000.0f;
constexpr float NTC_NOMINAL_OHMS = 10000.0f;
constexpr float NTC_BETA = 3950.0f;
constexpr float NTC_NOMINAL_K = 298.15f;

analog_monitor_snapshot_t g_snapshot = {};
uint16_t g_mg996_current_limit_raw = 4095U;
float g_vdda_v = ADC_REF_V;
bool g_vdda_calibrated = false;

void configure_mux_pins(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = ADC_MUX_S0_Pin;
  HAL_GPIO_Init(ADC_MUX_S0_GPIO_Port, &gpio);
  gpio.Pin = ADC_MUX_S1_Pin;
  HAL_GPIO_Init(ADC_MUX_S1_GPIO_Port, &gpio);
}

void select_mux_channel(analog_monitor_channel_t channel) {
  uint8_t raw_channel = (uint8_t)channel;
  HAL_GPIO_WritePin(ADC_MUX_S0_GPIO_Port, ADC_MUX_S0_Pin,
                    ((raw_channel & 0x01U) != 0U) ? GPIO_PIN_SET
                                                   : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ADC_MUX_S1_GPIO_Port, ADC_MUX_S1_Pin,
                    ((raw_channel & 0x02U) != 0U) ? GPIO_PIN_SET
                                                   : GPIO_PIN_RESET);
}

float raw_to_adc_voltage(uint16_t raw) {
  return ((float)raw * g_vdda_v) / ADC_FULL_SCALE;
}

/* The NTC pull-up is assumed to sit on the same rail the ADC references,
 * so only the divider ratio matters, not the absolute rail voltage. */
float ntc_voltage_to_c(float voltage) {
  if ((voltage <= 0.01f) || (voltage >= (g_vdda_v - 0.01f))) {
    return -273.15f;
  }

  float ntc_ohms = (NTC_PULLUP_OHMS * voltage) / (g_vdda_v - voltage);
  float inv_t = (1.0f / NTC_NOMINAL_K) +
                (logf(ntc_ohms / NTC_NOMINAL_OHMS) / NTC_BETA);
  return (1.0f / inv_t) - 273.15f;
}

bool read_adc_raw(uint16_t *raw) {
  if (raw == nullptr) {
    return false;
  }

#ifdef HAL_ADC_MODULE_ENABLED
  if (&hadc1 == nullptr) {
    return false;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return false;
  }
  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc1);
    return false;
  }

  *raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return true;
#else
  (void)raw;
  return false;
#endif
}

#ifdef HAL_ADC_MODULE_ENABLED
bool select_adc_channel(uint32_t channel, uint32_t sampling_time) {
  ADC_ChannelConfTypeDef config = {};
  config.Channel = channel;
  config.Rank = 1U;
  config.SamplingTime = sampling_time;
  return HAL_ADC_ConfigChannel(&hadc1, &config) == HAL_OK;
}
#endif

/* Measure VDDA through the internal VREFINT channel so the divider maths
 * does not assume a 3.3 V rail (the board's rail sits nearer 2.9 V, which
 * made every reading ~12 % high). Leaves the ADC back on the mux channel. */
void update_vdda(void) {
#ifdef HAL_ADC_MODULE_ENABLED
  if (&hadc1 == nullptr) {
    return;
  }

  bool ok = false;
  uint16_t raw = 0U;
  /* VREFINT needs >= 10 us sampling: 480 cycles at ADC clock 25 MHz. */
  if (select_adc_channel(ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_480CYCLES)) {
    HAL_Delay(1U); /* TSVREFE start-up on the first pass */
    ok = read_adc_raw(&raw) && (raw != 0U);
  }
  (void)select_adc_channel(ADC_CHANNEL_9, ADC_SAMPLETIME_84CYCLES);

  if (!ok) {
    return;
  }

  uint16_t cal = *VREFINT_CAL_ADDR;
  float vdda = ((float)VREFINT_CAL_VREF * 0.001f * (float)cal) / (float)raw;
  if ((cal == 0U) || (cal == 0xFFFFU) || (vdda < VDDA_MIN_PLAUSIBLE_V) ||
      (vdda > VDDA_MAX_PLAUSIBLE_V)) {
    return;
  }
  g_vdda_v = vdda;
  g_vdda_calibrated = true;
#endif
}

} // namespace

void AnalogMonitor_Init(void) {
  configure_mux_pins();
  g_snapshot = {};
  g_mg996_current_limit_raw = 4095U;
  g_vdda_v = ADC_REF_V;
  g_vdda_calibrated = false;
}

bool AnalogMonitor_ReadChannel(analog_monitor_channel_t channel,
                               uint16_t *raw) {
  if ((raw == nullptr) ||
      ((uint8_t)channel >= ANALOG_MONITOR_CHANNEL_COUNT)) {
    return false;
  }

  select_mux_channel(channel);
  HAL_Delay(1U);
  return read_adc_raw(raw);
}

void AnalogMonitor_Update(void) {
  bool any_valid = false;

  update_vdda();
  g_snapshot.vdda_v = g_vdda_v;
  g_snapshot.vdda_calibrated = g_vdda_calibrated;

  for (uint8_t i = 0U; i < ANALOG_MONITOR_CHANNEL_COUNT; ++i) {
    uint16_t raw = 0U;
    bool ok = AnalogMonitor_ReadChannel((analog_monitor_channel_t)i, &raw);
    g_snapshot.valid[i] = ok;
    if (ok) {
      any_valid = true;
      g_snapshot.raw[i] = raw;
      g_snapshot.adc_voltage[i] = raw_to_adc_voltage(raw);
    }
  }

  float board_voltage =
      g_snapshot.adc_voltage[ANALOG_MONITOR_CHANNEL_BOARD_TEMP];
  g_snapshot.board_temperature_c = g_snapshot.valid[ANALOG_MONITOR_CHANNEL_BOARD_TEMP]
                                       ? ntc_voltage_to_c(board_voltage)
                                       : -273.15f;
  g_snapshot.main_battery_v =
      g_snapshot.valid[ANALOG_MONITOR_CHANNEL_MAIN_BATTERY]
          ? (g_snapshot.adc_voltage[ANALOG_MONITOR_CHANNEL_MAIN_BATTERY] *
             MAIN_BATTERY_DIVIDER_GAIN)
          : 0.0f;
  g_snapshot.aon_battery_v =
      g_snapshot.valid[ANALOG_MONITOR_CHANNEL_AON_BATTERY]
          ? (g_snapshot.adc_voltage[ANALOG_MONITOR_CHANNEL_AON_BATTERY] *
             AON_BATTERY_DIVIDER_GAIN)
          : 0.0f;
  g_snapshot.mg996_current_limit =
      g_snapshot.valid[ANALOG_MONITOR_CHANNEL_MG996_CURRENT] &&
      (g_snapshot.raw[ANALOG_MONITOR_CHANNEL_MG996_CURRENT] >=
       g_mg996_current_limit_raw);
  g_snapshot.adc_available = any_valid;
}

void AnalogMonitor_SetMg996CurrentLimitRaw(uint16_t threshold_raw) {
  g_mg996_current_limit_raw = threshold_raw;
}

void AnalogMonitor_GetSnapshot(analog_monitor_snapshot_t *snapshot) {
  if (snapshot == nullptr) {
    return;
  }
  *snapshot = g_snapshot;
}
