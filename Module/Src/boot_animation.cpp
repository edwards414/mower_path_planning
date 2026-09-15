#include "boot_animation.hpp"
#include "buzzer.hpp"
extern "C" {
#include "ws2812.h"
}
#include "stm32f4xx_hal.h"

#include <math.h>

namespace {

/* ---- timeline (ms from start) ------------------------------------------ */
constexpr uint32_t kFrameMs = 20U;
constexpr uint32_t kIgnitionEndMs = 1200U;
constexpr uint32_t kSettleEndMs = 2200U; /* ignition tail -> steady white */

bool g_active = false;
uint32_t g_start_ms = 0U;
uint32_t g_last_frame_ms = 0U;

/* Buzzer cues: two short chirps as the spark ignites, one long tone when
 * the white comes on. */
struct BuzzerCue {
  uint32_t at_ms;
  uint32_t duration_ms;
};
constexpr BuzzerCue kCues[] = {
    {0U, 60U},
    {160U, 60U},
    {kIgnitionEndMs, 220U},
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

/* Perceptual-ish gamma so fades look smooth on WS2812. */
uint8_t to_u8(float v) {
  v = clamp01(v);
  v = v * v;
  return (uint8_t)(v * (float)BOOT_ANIMATION_MAX_BRIGHTNESS + 0.5f);
}

void put(uint16_t *buf, int led, float r, float g, float b) {
  ws2812_set_led_buf(buf, led, to_u8(r), to_u8(g), to_u8(b));
}

/* ---- phase 1: ignition --------------------------------------------------
 * A spark grows from the centre outwards. Head is near-white, the tail
 * fades through cyan down to a dim cyan floor (never black, so the hand-over
 * to white has nothing to jump from). p in [0,1]. */
constexpr float kTailFloor = 0.22f;

void ignition_color(int i, int led_count, float p, float *r, float *g, float *b) {
  float centre = ((float)led_count - 1.0f) * 0.5f;
  float half = (float)led_count * 0.5f;
  /* ease-out so the spark bursts fast then slows at the ends */
  float e = 1.0f - (1.0f - p) * (1.0f - p);
  float head = e * (half + 2.0f);

  float d = fabsf((float)i - centre); /* distance from centre */
  float behind = head - d;            /* >0 once the head passed */
  float v;
  float white = 0.0f;
  if (behind < 0.0f) {
    v = 0.0f;
  } else if (behind < 1.5f) {
    v = 1.0f; /* bright head */
    white = 1.0f - behind / 1.5f;
  } else {
    float fade = clamp01(1.0f - (behind - 1.5f) / 6.0f);
    v = kTailFloor + (1.0f - kTailFloor) * fade;
  }
  *r = v * (0.15f + 0.85f * white);
  *g = v * (0.75f + 0.25f * white);
  *b = v;
}

void render_ignition(uint16_t *buf, int led_count, float p) {
  for (int i = 0; i < led_count; ++i) {
    float r, g, b;
    ignition_color(i, led_count, p, &r, &g, &b);
    put(buf, i, r, g, b);
  }
}

/* ---- phase 2: settle to steady white -----------------------------------
 * Per-LED cross-fade from the exact ignition end colour into an even white,
 * smoothstep-eased so both ends of the blend are gentle. p in [0,1]. */
void render_settle(uint16_t *buf, int led_count, float p) {
  float mix = p * p * (3.0f - 2.0f * p); /* smoothstep */
  for (int i = 0; i < led_count; ++i) {
    float r0, g0, b0;
    ignition_color(i, led_count, 1.0f, &r0, &g0, &b0);
    float r = r0 + (1.0f - r0) * mix;
    float g = g0 + (1.0f - g0) * mix;
    float b = b0 + (1.0f - b0) * mix;
    put(buf, i, r, g, b);
  }
}

void render_white(uint16_t *buf, int led_count) {
  for (int i = 0; i < led_count; ++i) {
    put(buf, i, 1.0f, 1.0f, 1.0f);
  }
}

void render_frame(uint32_t t_ms) {
  ws2812_clear_all();

  if (t_ms < kIgnitionEndMs) {
    float p = (float)t_ms / (float)kIgnitionEndMs;
    render_ignition(ws2812_buf_front, LED_NUM_FRONT, p);
    render_ignition(ws2812_buf_back, LED_NUM_BACK, p);
  } else if (t_ms < kSettleEndMs) {
    float p = (float)(t_ms - kIgnitionEndMs) / (float)(kSettleEndMs - kIgnitionEndMs);
    render_settle(ws2812_buf_front, LED_NUM_FRONT, p);
    render_settle(ws2812_buf_back, LED_NUM_BACK, p);
  } else {
    render_white(ws2812_buf_front, LED_NUM_FRONT);
    render_white(ws2812_buf_back, LED_NUM_BACK);
  }

  ws2812_show_dual();
}

} // namespace

void BootAnimation_Start(void) {
  g_active = true;
  g_next_cue = 0U;
  g_start_ms = HAL_GetTick();
  g_last_frame_ms = g_start_ms - kFrameMs; /* render first frame at once */
}

bool BootAnimation_IsActive(void) { return g_active; }

void BootAnimation_Stop(void) {
  if (!g_active) {
    return;
  }
  g_active = false;
  ws2812_clear_all();
  ws2812_show_dual();
}

bool BootAnimation_Update(void) {
  if (!g_active) {
    return false;
  }

  uint32_t now = HAL_GetTick();
  if ((now - g_last_frame_ms) < kFrameMs) {
    return true;
  }
  g_last_frame_ms = now;

  uint32_t t = now - g_start_ms;
  while ((g_next_cue < kCueCount) && (t >= kCues[g_next_cue].at_ms)) {
    Buzzer_Beep(kCues[g_next_cue].duration_ms);
    ++g_next_cue;
  }

  if (t >= kSettleEndMs) {
    /* Final frame: steady white. Hand the strips back to the UART path
     * without clearing, so the white stays until a command replaces it. */
    render_frame(t);
    g_active = false;
    return false;
  }

  render_frame(t);
  return true;
}
