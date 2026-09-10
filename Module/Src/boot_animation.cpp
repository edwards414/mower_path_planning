#include "boot_animation.hpp"
extern "C" {
#include "ws2812.h"
}
#include "stm32f4xx_hal.h"

#include <math.h>

namespace {

/* ---- timeline (ms from start) ------------------------------------------ */
constexpr uint32_t kFrameMs = 20U;
constexpr uint32_t kIgnitionEndMs = 1200U;
constexpr uint32_t kWaveEndMs = 2400U;
constexpr uint32_t kBreatheEndMs = 3600U;

bool g_active = false;
uint32_t g_start_ms = 0U;
uint32_t g_last_frame_ms = 0U;

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

/* h in [0,1), s,v in [0,1] -> r,g,b in [0,1] */
void hsv(float h, float s, float v, float *r, float *g, float *b) {
  h = h - floorf(h);
  float i = floorf(h * 6.0f);
  float f = h * 6.0f - i;
  float p = v * (1.0f - s);
  float q = v * (1.0f - s * f);
  float t = v * (1.0f - s * (1.0f - f));
  switch ((int)i % 6) {
  case 0: *r = v; *g = t; *b = p; break;
  case 1: *r = q; *g = v; *b = p; break;
  case 2: *r = p; *g = v; *b = t; break;
  case 3: *r = p; *g = q; *b = v; break;
  case 4: *r = t; *g = p; *b = v; break;
  default: *r = v; *g = p; *b = q; break;
  }
}

void put(uint16_t *buf, int led, float r, float g, float b) {
  ws2812_set_led_buf(buf, led, to_u8(r), to_u8(g), to_u8(b));
}

/* ---- phase 1: ignition --------------------------------------------------
 * A spark grows from the centre outwards. Head is near-white, the tail
 * fades through cyan into black. p in [0,1]. */
void render_ignition(uint16_t *buf, int led_count, float p) {
  float centre = ((float)led_count - 1.0f) * 0.5f;
  float half = (float)led_count * 0.5f;
  /* ease-out so the spark bursts fast then slows at the ends */
  float e = 1.0f - (1.0f - p) * (1.0f - p);
  float head = e * (half + 2.0f);

  for (int i = 0; i < led_count; ++i) {
    float d = fabsf((float)i - centre); /* distance from centre */
    float behind = head - d;            /* >0 once the head passed */
    float v;
    if (behind < 0.0f) {
      v = 0.0f;
    } else if (behind < 1.5f) {
      v = 1.0f; /* bright head */
    } else {
      v = clamp01(1.0f - (behind - 1.5f) / 6.0f); /* tail fade */
    }
    /* head is white, tail is cyan */
    float white = (behind >= 0.0f && behind < 1.5f) ? 1.0f : 0.0f;
    float r = v * (0.15f + 0.85f * white);
    float g = v * (0.75f + 0.25f * white);
    float b = v;
    put(buf, i, r, g, b);
  }
}

/* ---- phase 2: wave ------------------------------------------------------
 * Scrolling teal -> blue -> violet gradient, overall brightness ramps in
 * and the ignition afterglow fades out. */
void render_wave(uint16_t *buf, int led_count, float p, float scroll) {
  float ramp = clamp01(p * 2.5f); /* fully on after 40% of the phase */
  for (int i = 0; i < led_count; ++i) {
    float pos = (float)i / (float)led_count;
    /* hue window 0.48 (teal) .. 0.78 (violet), scrolling */
    float h = 0.48f + 0.30f * (0.5f + 0.5f * sinf((pos * 2.0f - scroll) * 6.2831853f));
    float r, g, b;
    hsv(h, 0.85f, 1.0f, &r, &g, &b);
    /* soft moving highlight */
    float hl = 0.5f + 0.5f * sinf((pos * 3.0f + scroll * 1.7f) * 6.2831853f);
    float v = ramp * (0.55f + 0.45f * hl);
    put(buf, i, r * v, g * v, b * v);
  }
}

/* ---- phase 3: breathe ---------------------------------------------------
 * Blend from the last wave colour into cool white, one breath, fade out. */
void render_breathe(uint16_t *buf, int led_count, float p) {
  /* brightness: rise to 1 at 35%, hold, then fall to 0 at 100% */
  float v;
  if (p < 0.35f) {
    v = 0.6f + 0.4f * (p / 0.35f);
  } else if (p < 0.55f) {
    v = 1.0f;
  } else {
    float q = (p - 0.55f) / 0.45f;
    v = (1.0f - q) * (1.0f - q);
  }
  float whiten = clamp01(p * 3.0f); /* colour -> white in first third */
  for (int i = 0; i < led_count; ++i) {
    float pos = (float)i / (float)led_count;
    float r, g, b;
    hsv(0.58f + 0.12f * pos, 0.85f, 1.0f, &r, &g, &b);
    r = r + (0.85f - r) * whiten;
    g = g + (0.90f - g) * whiten;
    b = b + (1.00f - b) * whiten;
    put(buf, i, r * v, g * v, b * v);
  }
}

void render_frame(uint32_t t_ms) {
  ws2812_clear_all();

  if (t_ms < kIgnitionEndMs) {
    float p = (float)t_ms / (float)kIgnitionEndMs;
    render_ignition(ws2812_buf_front, LED_NUM_FRONT, p);
    render_ignition(ws2812_buf_back, LED_NUM_BACK, p);
  } else if (t_ms < kWaveEndMs) {
    float p = (float)(t_ms - kIgnitionEndMs) / (float)(kWaveEndMs - kIgnitionEndMs);
    float scroll = (float)t_ms / 900.0f;
    render_wave(ws2812_buf_front, LED_NUM_FRONT, p, scroll);
    render_wave(ws2812_buf_back, LED_NUM_BACK, p, scroll);
  } else {
    float p = (float)(t_ms - kWaveEndMs) / (float)(kBreatheEndMs - kWaveEndMs);
    render_breathe(ws2812_buf_front, LED_NUM_FRONT, p);
    render_breathe(ws2812_buf_back, LED_NUM_BACK, p);
  }

  ws2812_show_dual();
}

} // namespace

void BootAnimation_Start(void) {
  g_active = true;
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
  if (t >= kBreatheEndMs) {
    BootAnimation_Stop();
    return false;
  }

  render_frame(t);
  return true;
}
