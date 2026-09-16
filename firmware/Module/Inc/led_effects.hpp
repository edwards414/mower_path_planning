/*
 * led_effects.hpp
 *
 * Smooth, time-based WS2812 effects for the UART command path, in the same
 * non-blocking style as boot_animation / sleep_animation: LedEffects_Update()
 * renders one frame per 20 ms call and owns the strips while it returns true.
 *
 * Effects:
 *   ORBIT  a warm comet that glides around each strip on a faint breathing
 *          floor. Sub-LED head position and an exponential tail, so it moves
 *          smoothly instead of stepping LED by LED (mode 0x06 in
 *          LED_COMMAND_MODES.md; the host shows it in amber while the robot
 *          software is updating).
 *   SOLID  cross-fade from whatever is currently on the strips to one colour
 *          (used for CLEAR and ALL_ON so they no longer snap).
 *
 * Every transition starts from the colours actually on the strips (decoded
 * from the DMA buffers), so orbit -> solid, solid -> orbit and colour
 * changes never jump.
 */
#ifndef MODULE_INC_LED_EFFECTS_HPP_
#define MODULE_INC_LED_EFFECTS_HPP_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Peak per-channel brightness, same budget as the boot animation. */
#define LED_EFFECTS_MAX_BRIGHTNESS 110U
#define LED_EFFECTS_ORBIT_DEFAULT_PERIOD_MS 1600U

/* Start the orbit, or update its colour / period if it is already running
 * (no restart, the comet keeps its position). r=g=b=0 selects the default
 * amber. period_ms is one revolution; 0 selects the default. */
void LedEffects_StartOrbit(uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms);

/* Cross-fade from the current strip contents to a solid colour over
 * fade_ms, then release the strips (they stay at that colour). */
void LedEffects_FadeToSolid(uint8_t r, uint8_t g, uint8_t b, uint16_t fade_ms);

/* Render the next frame if it is time. Returns true while an effect or a
 * transition still owns the strips. */
bool LedEffects_Update(void);

bool LedEffects_IsActive(void);
bool LedEffects_OrbitRunning(void);

/* Abort immediately without touching the strips (another owner takes
 * over, e.g. the legacy FLOW/TURN steppers). */
void LedEffects_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_LED_EFFECTS_HPP_ */
