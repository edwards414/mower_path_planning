#include "sleep_animation.hpp"
#include "boot_animation.hpp" /* BOOT_ANIMATION_MAX_BRIGHTNESS */
#include "buzzer.hpp"
extern "C" {
#include "ws2812.h"
}
#include "stm32f4xx_hal.h"

#include <math.h>

namespace {

/* ---- timeline (ms from start) ------------------------------------------ */
constexpr uint32_t kFrameMs = 20U;
constexpr uint32_t kExhaleEndMs = 1400U; /* white -> dim amber */
constexpr uint32_t kGatherEndMs = 2800U; /* amber retreats to the centre */
constexpr uint32_t kBreathPeriodMs = 3200U;
constexpr uint32_t kFadeMs = 900U; /* final fade after Finish() */

/* Warm amber core colour (pre-gamma). */
constexpr float kAmberR = 1.0f;
constexpr float kAmberG = 0.48f;
constexpr float kAmberB = 0.10f;

/* Brightness of the amber field right after the exhale, and the range the
 * core breathes within afterwards. */
constexpr float kExhaleLevel = 0.60f;
constexpr float kBreathHigh = 0.60f;
constexpr float kBreathLow = 0.24f;

/* Where the glow edge ends up after the gather (0 = centre, 1 = strip end)
 * and how wide the soft edge is. */
constexpr float kCoreEdge = 0.20f;
constexpr float kEdgeSoftness = 0.30f;

enum class Phase : uint8_t { Idle, Running, Fading };

Phase g_phase = Phase::Idle;
uint32_t g_start_ms = 0U;
uint32_t g_last_frame_ms = 0U;
uint32_t g_fade_start_ms = 0U;
/* Snapshot taken when Finish() is called so the fade starts from exactly
 * what is on the strips: overall level and how far the gather had got. */
float g_fade_from_level = 0.0f;
float g_fade_from_gather = 0.0f;
float g_fade_from_exhale = 0.0f;

struct BuzzerCue {
  uint32_t at_ms;
  uint32_t duration_ms;
};
constexpr BuzzerCue kCues[] = {
    {0U, 50U},
    {140U, 50U},
};
constexpr uint8_t kCueCount = sizeof(kCues) / sizeof(kCues[0]);
uint8_t g_next_cue = 0U;

/* ---- helpers ------------------------------------------------------------ */

float clamp01(float v) {
  if (v < 0.0f) {
    return 0.0f;
  }
  if (v > 1.0f) {
    return 1.0f;
  }
  return v;
}

float smoothstep(float p) {
  p = clamp01(p);
  return p * p * (3.0f - 2.0f * p);
}

/* Same perceptual gamma and brightness cap as the boot animation so the
 * two sequences match. */
uint8_t to_u8(float v) {
  v = clamp01(v);
  v = v * v;
  return (uint8_t)(v * (float)BOOT_ANIMATION_MAX_BRIGHTNESS + 0.5f);
}

void put(uint16_t *buf, int led, float r, float g, float b) {
  ws2812_set_led_buf(buf, led, to_u8(r), to_u8(g), to_u8(b));
}

/* Spatial profile of the glow for one LED. gather in [0,1]: 0 = whole
 * strip lit evenly, 1 = only the core around the centre. Soft edged so the
 * retreat never steps LED by LED. */
float glow_profile(int i, int led_count, float gather) {
  float centre = ((float)led_count - 1.0f) * 0.5f;
  float half = (float)led_count * 0.5f;
  float d = fabsf((float)i - centre) / half; /* 0 centre .. ~1 end */
  float edge = 1.0f + kEdgeSoftness - (1.0f + kEdgeSoftness - kCoreEdge) * gather;
  return smoothstep((edge - d) / kEdgeSoftness);
}

/* Breath level as a cosine that starts at its peak, so the hand-over from
 * the gather (which ends at kBreathHigh) has no step. */
float breath_level(uint32_t t_since_gather_ms) {
  float ph = (float)(t_since_gather_ms % kBreathPeriodMs) / (float)kBreathPeriodMs;
  float c = 0.5f + 0.5f * cosf(ph * 6.28318530718f);
  return kBreathLow + (kBreathHigh - kBreathLow) * c;
}

/* Render one strip. exhale: 0 = white, 1 = amber. level: overall brightness
 * of the amber field. gather: see glow_profile. */
void render_strip(uint16_t *buf, int led_count, float exhale, float level,
                  float gather) {
  float r_c = 1.0f + (kAmberR - 1.0f) * exhale;
  float g_c = 1.0f + (kAmberG - 1.0f) * exhale;
  float b_c = 1.0f + (kAmberB - 1.0f) * exhale;
  for (int i = 0; i < led_count; ++i) {
    float v = level * glow_profile(i, led_count, gather);
    put(buf, i, r_c * v, g_c * v, b_c * v);
  }
}

/* Work out the three parameters for time t in the running phase. */
void running_params(uint32_t t_ms, float *exhale, float *level, float *gather) {
  if (t_ms < kExhaleEndMs) {
    float m = smoothstep((float)t_ms / (float)kExhaleEndMs);
    *exhale = m;
    *level = 1.0f + (kExhaleLevel - 1.0f) * m;
    *gather = 0.0f;
  } else if (t_ms < kGatherEndMs) {
    float p = (float)(t_ms - kExhaleEndMs) / (float)(kGatherEndMs - kExhaleEndMs);
    *exhale = 1.0f;
    *level = kExhaleLevel;
    *gather = smoothstep(p);
  } else {
    *exhale = 1.0f;
    *level = breath_level(t_ms - kGatherEndMs);
    *gather = 1.0f;
  }
}

void render_both(float exhale, float level, float gather) {
  ws2812_clear_all();
  render_strip(ws2812_buf_front, LED_NUM_FRONT, exhale, level, gather);
  render_strip(ws2812_buf_back, LED_NUM_BACK, exhale, level, gather);
  ws2812_show_dual();
}

} // namespace

void SleepAnimation_Start(void) {
  g_phase = Phase::Running;
  g_next_cue = 0U;
  g_start_ms = HAL_GetTick();
  g_last_frame_ms = g_start_ms - kFrameMs; /* render first frame at once */
}

void SleepAnimation_Finish(void) {
  if (g_phase != Phase::Running) {
    return;
  }
  uint32_t now = HAL_GetTick();
  running_params(now - g_start_ms, &g_fade_from_exhale, &g_fade_from_level,
                 &g_fade_from_gather);
  g_fade_start_ms = now;
  g_phase = Phase::Fading;
  Buzzer_Beep(120U);
}

bool SleepAnimation_IsActive(void) { return g_phase != Phase::Idle; }

void SleepAnimation_Stop(void) {
  if (g_phase == Phase::Idle) {
    return;
  }
  g_phase = Phase::Idle;
  ws2812_clear_all();
  ws2812_show_dual();
}

bool SleepAnimation_Update(void) {
  if (g_phase == Phase::Idle) {
    return false;
  }

  uint32_t now = HAL_GetTick();
  if ((now - g_last_frame_ms) < kFrameMs) {
    return true;
  }
  g_last_frame_ms = now;

  if (g_phase == Phase::Running) {
    uint32_t t = now - g_start_ms;
    while ((g_next_cue < kCueCount) && (t >= kCues[g_next_cue].at_ms)) {
      Buzzer_Beep(kCues[g_next_cue].duration_ms);
      ++g_next_cue;
    }
    float exhale, level, gather;
    running_params(t, &exhale, &level, &gather);
    render_both(exhale, level, gather);
    return true;
  }

  /* Fading: ease from the snapshot down to black. The colour keeps cooling
   * toward amber and the glow keeps gathering if Finish() came early, so
   * an interrupted sequence still ends the same way. */
  uint32_t t = now - g_fade_start_ms;
  if (t >= kFadeMs) {
    g_phase = Phase::Idle;
    ws2812_clear_all();
    ws2812_show_dual();
    return false;
  }
  float p = smoothstep((float)t / (float)kFadeMs);
  float exhale = g_fade_from_exhale + (1.0f - g_fade_from_exhale) * p;
  float gather = g_fade_from_gather + (1.0f - g_fade_from_gather) * p;
  float level = g_fade_from_level * (1.0f - p);
  render_both(exhale, level, gather);
  return true;
}
