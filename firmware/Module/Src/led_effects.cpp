#include "led_effects.hpp"
extern "C" {
#include "ws2812.h"
}
#include "stm32f4xx_hal.h"

#include <math.h>

namespace {

constexpr uint32_t kFrameMs = 20U;
/* The 50 Hz task loop is osDelay(20) plus the work, so the tick delta
 * between calls jitters around 20 ms; gating at exactly 20 ms would drop
 * a frame every time it lands on 19 (a visible hitch in slow fades). */
constexpr uint32_t kFrameGateMs = 15U;
constexpr uint32_t kOrbitFadeInMs = 700U;
constexpr uint32_t kSolidFadeMinMs = 40U;

/* Orbit look. Distances are in LEDs of a 16-LED ring and scaled for the
 * 32-LED front strip so both strips show the same picture. */
constexpr float kTailFraction = 0.30f;   /* tail length as a fraction of the ring */
constexpr float kHeadSigma = 0.55f;      /* soft leading edge (anti-aliased head) */
constexpr float kTipSigma = 0.70f;       /* where the head turns hot/white */
constexpr float kTipWhite = 0.55f;       /* how white the tip gets */
constexpr float kFloor = 0.045f;         /* faint base glow, never fully dark */
constexpr float kFloorBreath = 0.025f;   /* +/- breathing on the floor */
constexpr float kBreathPeriodMs = 2800.0f;
constexpr float kPi = 3.14159265f;

/* Rear-light "recording" overlay: an asymmetric red breath on the back
 * strip only. The envelope is an eased inhale, a longer eased exhale and a
 * short rest at the floor; every joint has zero slope so there is no kink.
 * The floor keeps a dim ember (~6 % of peak output) instead of going dark,
 * and the dim end is ordered-dithered across the 16 LEDs so it glides in
 * 1/16-code steps instead of ticking between adjacent WS2812 codes. */
constexpr uint32_t kRearPeriodMs = 2400U;
constexpr float kRearRise = 0.36f;      /* fraction of the period */
constexpr float kRearFall = 0.50f;      /* the rest (0.14) sits on the floor */
constexpr float kRearFloor = 0.25f;     /* perceptual; squared = ~6 % output */
constexpr float kRearFadeInMs = 600.0f; /* cross-fade from / back to the base */
constexpr float kRearFadeOutMs = 900.0f;

struct Rgb {
  float r;
  float g;
  float b;
};

enum class Mode : uint8_t { kIdle, kOrbit, kSolid };

Mode g_mode = Mode::kIdle;
uint32_t g_last_frame_ms = 0U;

/* frame the current transition started from (linear 0..1) */
Rgb g_from_front[LED_NUM_FRONT];
Rgb g_from_back[LED_NUM_BACK];

/* orbit */
Rgb g_orbit_color = {1.0f, 0.70f, 0.0f};
uint32_t g_orbit_period_ms = LED_EFFECTS_ORBIT_DEFAULT_PERIOD_MS;
uint32_t g_orbit_t0 = 0U;      /* phase reference */
uint32_t g_orbit_fade_t0 = 0U; /* start of the fade-in from g_from_* */

/* solid */
Rgb g_solid_target = {0.0f, 0.0f, 0.0f};
uint32_t g_solid_t0 = 0U;
uint32_t g_solid_ms = 400U;

/* rear overlay; ws2812_buf_back stays the base layer, the composed frame
 * goes to g_rear_buf and ws2812_show_dual() clocks that out instead */
const Rgb kRearColor = {1.0f, 0.0f, 0.0f};
bool g_rear_on = false;
float g_rear_ramp = 0.0f; /* 0 = base only .. 1 = overlay only (linear) */
uint32_t g_rear_t0 = 0U;  /* breath phase reference */
uint32_t g_rear_last_ms = 0U;
uint16_t g_rear_buf[BUF_LEN_BACK];

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

float smoothstep01(float p) {
  p = clamp01(p);
  return p * p * (3.0f - 2.0f * p);
}

/* Perceptual-ish gamma (same as boot_animation) so fades look even. */
uint8_t to_u8(float v) {
  v = clamp01(v);
  v = v * v;
  return (uint8_t)(v * (float)LED_EFFECTS_MAX_BRIGHTNESS + 0.5f);
}

float from_u8(uint8_t v) {
  float lin = (float)v / (float)LED_EFFECTS_MAX_BRIGHTNESS;
  if (lin > 1.0f) {
    lin = 1.0f;
  }
  return sqrtf(lin);
}

void put(uint16_t *buf, int led, const Rgb &c) {
  ws2812_set_led_buf(buf, led, to_u8(c.r), to_u8(c.g), to_u8(c.b));
}

/* Read one LED back out of the DMA bit buffer (GRB, MSB first). */
Rgb decode(const uint16_t *buf, int led) {
  const uint16_t *p = buf + led * 24;
  uint8_t ch[3] = {0U, 0U, 0U}; /* g, r, b */
  for (int c = 0; c < 3; ++c) {
    uint8_t v = 0U;
    for (int bit = 0; bit < 8; ++bit) {
      v = (uint8_t)((v << 1) | ((p[c * 8 + bit] == Code1) ? 1U : 0U));
    }
    ch[c] = v;
  }
  Rgb out;
  out.r = from_u8(ch[1]);
  out.g = from_u8(ch[0]);
  out.b = from_u8(ch[2]);
  return out;
}

/* Snapshot what is on the strips right now as the start of a transition. */
void capture_from(void) {
  for (int i = 0; i < LED_NUM_FRONT; ++i) {
    g_from_front[i] = decode(ws2812_buf_front, i);
  }
  for (int i = 0; i < LED_NUM_BACK; ++i) {
    g_from_back[i] = decode(ws2812_buf_back, i);
  }
}

Rgb lerp(const Rgb &a, const Rgb &b, float p) {
  Rgb out;
  out.r = a.r + (b.r - a.r) * p;
  out.g = a.g + (b.g - a.g) * p;
  out.b = a.b + (b.b - a.b) * p;
  return out;
}

/* ---- orbit --------------------------------------------------------------
 * A comet whose head sits at a fractional position on the ring, with an
 * exponential tail behind it and a short gaussian edge ahead of it, plus a
 * slowly breathing floor. t_ms is time since the phase reference. */
Rgb orbit_pixel(int i, int led_count, float head, float floor_level) {
  const float n = (float)led_count;
  const float scale = n / (float)LED_NUM_BACK; /* 1 on the back ring, 2 on the front */
  float d = head - (float)i; /* how far behind the head this LED is */
  d = fmodf(d, n);
  if (d < 0.0f) {
    d += n;
  }
  const float ahead = n - d;

  float v = expf(-d / (kTailFraction * n));
  const float edge = expf(-(ahead * ahead) / (2.0f * kHeadSigma * kHeadSigma * scale * scale));
  if (edge > v) {
    v = edge;
  }
  const float tip = kTipWhite * expf(-(d * d) / (2.0f * kTipSigma * kTipSigma * scale * scale));

  Rgb c;
  c.r = v * (g_orbit_color.r + (1.0f - g_orbit_color.r) * tip) + floor_level * g_orbit_color.r;
  c.g = v * (g_orbit_color.g + (1.0f - g_orbit_color.g) * tip) + floor_level * g_orbit_color.g;
  c.b = v * (g_orbit_color.b + (1.0f - g_orbit_color.b) * tip) + floor_level * g_orbit_color.b;
  c.r = clamp01(c.r);
  c.g = clamp01(c.g);
  c.b = clamp01(c.b);
  return c;
}

void render_orbit(uint32_t now) {
  const uint32_t t = now - g_orbit_t0;
  const float phase = (float)(t % g_orbit_period_ms) / (float)g_orbit_period_ms;
  const float breath = sinf(2.0f * kPi * (float)t / kBreathPeriodMs);
  const float floor_level = kFloor + kFloorBreath * breath;
  const float fade = smoothstep01((float)(now - g_orbit_fade_t0) / (float)kOrbitFadeInMs);

  for (int i = 0; i < LED_NUM_FRONT; ++i) {
    Rgb c = orbit_pixel(i, LED_NUM_FRONT, phase * (float)LED_NUM_FRONT, floor_level);
    put(ws2812_buf_front, i, lerp(g_from_front[i], c, fade));
  }
  for (int i = 0; i < LED_NUM_BACK; ++i) {
    Rgb c = orbit_pixel(i, LED_NUM_BACK, phase * (float)LED_NUM_BACK, floor_level);
    put(ws2812_buf_back, i, lerp(g_from_back[i], c, fade));
  }
}

/* ---- solid ------------------------------------------------------------- */
bool render_solid(uint32_t now) {
  const float p = smoothstep01((float)(now - g_solid_t0) / (float)g_solid_ms);
  for (int i = 0; i < LED_NUM_FRONT; ++i) {
    put(ws2812_buf_front, i, lerp(g_from_front[i], g_solid_target, p));
  }
  for (int i = 0; i < LED_NUM_BACK; ++i) {
    put(ws2812_buf_back, i, lerp(g_from_back[i], g_solid_target, p));
  }
  return p < 1.0f;
}

/* ---- rear overlay ------------------------------------------------------- */
float ease_in_out_sine(float x) { return 0.5f - 0.5f * cosf(kPi * clamp01(x)); }

/* 0..1 breath envelope, t_ms since the phase reference */
float rear_breath(uint32_t t_ms) {
  const float p = (float)(t_ms % kRearPeriodMs) / (float)kRearPeriodMs;
  if (p < kRearRise) {
    return ease_in_out_sine(p / kRearRise);
  }
  if (p < kRearRise + kRearFall) {
    return 1.0f - ease_in_out_sine((p - kRearRise) / kRearFall);
  }
  return 0.0f;
}

/* Ordered-dither threshold per LED: 4-bit bit reversal so neighbouring LEDs
 * get far-apart thresholds and the strip brightens evenly, not end to end. */
float dither_threshold(int led) {
  const unsigned i = (unsigned)led & 0x0FU;
  const unsigned r =
      ((i & 1U) << 3) | ((i & 2U) << 1) | ((i & 4U) >> 1) | ((i & 8U) >> 3);
  return ((float)r + 0.5f) / 16.0f;
}

/* Same gamma and cap as to_u8(), but floor(x + threshold) instead of
 * round(x). An exact code stays exact (threshold < 1). */
uint8_t to_u8_dithered(float v, float threshold) {
  v = clamp01(v);
  return (uint8_t)(v * v * (float)LED_EFFECTS_MAX_BRIGHTNESS + threshold);
}

/* Compose the overlay over the base back strip into g_rear_buf and point
 * ws2812_show_dual() at it, or hand the strip back to the base once the
 * fade-out has finished. */
void render_rear(uint32_t now) {
  uint32_t dt = now - g_rear_last_ms;
  g_rear_last_ms = now;
  if (dt > 100U) {
    dt = kFrameMs; /* first frame, or after the strips were locked */
  }
  if (g_rear_on) {
    if (g_rear_ramp <= 0.0f) {
      g_rear_t0 = now; /* start on an inhale */
    }
    g_rear_ramp = clamp01(g_rear_ramp + (float)dt / kRearFadeInMs);
  } else {
    g_rear_ramp = clamp01(g_rear_ramp - (float)dt / kRearFadeOutMs);
  }
  if (g_rear_ramp <= 0.0f) {
    ws2812_back_override = NULL;
    return;
  }

  const float a = smoothstep01(g_rear_ramp);
  const float level =
      kRearFloor + (1.0f - kRearFloor) * rear_breath(now - g_rear_t0);
  Rgb over;
  over.r = level * kRearColor.r;
  over.g = level * kRearColor.g;
  over.b = level * kRearColor.b;
  for (int i = 0; i < LED_NUM_BACK; ++i) {
    const Rgb c = (a >= 1.0f) ? over : lerp(decode(ws2812_buf_back, i), over, a);
    const float th = dither_threshold(i);
    ws2812_set_led_buf(g_rear_buf, i, to_u8_dithered(c.r, th),
                       to_u8_dithered(c.g, th), to_u8_dithered(c.b, th));
  }
  ws2812_back_override = g_rear_buf;
}

} // namespace

void LedEffects_StartOrbit(uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms) {
  Rgb color;
  if ((r == 0U) && (g == 0U) && (b == 0U)) {
    color = {1.0f, 0.70f, 0.0f}; /* warm amber */
  } else {
    color = {(float)r / 255.0f, (float)g / 255.0f, (float)b / 255.0f};
  }
  uint32_t period = (period_ms == 0U) ? LED_EFFECTS_ORBIT_DEFAULT_PERIOD_MS : period_ms;
  if (period < 200U) {
    period = 200U;
  }

  if (g_mode == Mode::kOrbit) {
    /* live parameter update, keep the phase continuous */
    if (period != g_orbit_period_ms) {
      uint32_t now = HAL_GetTick();
      uint32_t elapsed = (now - g_orbit_t0) % g_orbit_period_ms;
      g_orbit_t0 = now - (uint32_t)((uint64_t)elapsed * period / g_orbit_period_ms);
      g_orbit_period_ms = period;
    }
    g_orbit_color = color;
    return;
  }

  uint32_t now = HAL_GetTick();
  capture_from();
  g_orbit_color = color;
  g_orbit_period_ms = period;
  g_orbit_t0 = now;
  g_orbit_fade_t0 = now;
  g_mode = Mode::kOrbit;
  g_last_frame_ms = now - kFrameMs;
}

void LedEffects_FadeToSolid(uint8_t r, uint8_t g, uint8_t b, uint16_t fade_ms) {
  uint32_t now = HAL_GetTick();
  capture_from();
  g_solid_target = {(float)r / 255.0f, (float)g / 255.0f, (float)b / 255.0f};
  g_solid_t0 = now;
  g_solid_ms = (fade_ms < kSolidFadeMinMs) ? kSolidFadeMinMs : fade_ms;
  g_mode = Mode::kSolid;
  g_last_frame_ms = now - kFrameMs;
}

bool LedEffects_IsActive(void) { return g_mode != Mode::kIdle; }

bool LedEffects_OrbitRunning(void) { return g_mode == Mode::kOrbit; }

void LedEffects_Stop(void) { g_mode = Mode::kIdle; }

void LedEffects_SetRearRecording(bool on) { g_rear_on = on; }

void LedEffects_ResetRear(void) {
  g_rear_on = false;
  g_rear_ramp = 0.0f;
  ws2812_back_override = NULL;
}

bool LedEffects_RearActive(void) { return ws2812_back_override != NULL; }

bool LedEffects_Update(void) {
  /* the overlay needs a frame every tick while it is on or fading out,
   * even over a static base (CLEAR / ALL_ON / legacy steppers) */
  const bool rear = g_rear_on || (ws2812_back_override != NULL);
  if ((g_mode == Mode::kIdle) && !rear) {
    return false;
  }
  uint32_t now = HAL_GetTick();
  if ((now - g_last_frame_ms) < kFrameGateMs) {
    return g_mode != Mode::kIdle;
  }
  g_last_frame_ms = now;

  bool base_active = false;
  if (g_mode == Mode::kOrbit) {
    render_orbit(now);
    base_active = true;
  } else if (g_mode == Mode::kSolid) {
    /* solid: last frame lands exactly on the target, then release */
    base_active = render_solid(now);
    if (!base_active) {
      g_mode = Mode::kIdle;
    }
  }
  render_rear(now);
  ws2812_show_dual();
  return base_active;
}
