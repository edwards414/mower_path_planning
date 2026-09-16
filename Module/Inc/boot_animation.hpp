/*
 * boot_animation.hpp
 *
 * Power-on light show for the WS2812 strips (front 32 LEDs, back 16 LEDs).
 * Non-blocking: BootAnimation_Update() renders one frame per call and is
 * meant to be called from the 20 ms WS2812 runtime loop. While it returns
 * true the UART WS2812 command path must not touch the strips.
 *
 * Sequence (about 2.6 s, then steady):
 *   0. Pre-glow  : a faint cyan bloom swells at the centre of each strip.
 *   1. Ignition  : a cyan-white spark bursts from the centre outwards with
 *                  a long fading tail (cubic ease-out).
 *   2. Settle    : cross-fades into an even white and stays lit. The white
 *                  remains until a UART WS2812 command replaces it.
 *   Buzzer cues : two short chirps at ignition, one long tone as the
 *                  white comes on (needs Buzzer_Init() before the first
 *                  update).
 */
#ifndef MODULE_INC_BOOT_ANIMATION_HPP_
#define MODULE_INC_BOOT_ANIMATION_HPP_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Peak per-channel brightness (0-255). 48 LEDs at 255 white draw ~2.9 A,
 * so keep this modest for a 5 V rail shared with other modules. */
#define BOOT_ANIMATION_MAX_BRIGHTNESS 110U

/* Start (or restart) the animation. Safe to call from init code. */
void BootAnimation_Start(void);

/* Render the next frame if it is time. Returns true while the animation
 * still owns the strips, false once it has finished (strips left white). */
bool BootAnimation_Update(void);

bool BootAnimation_IsActive(void);

/* Abort immediately and blank the strips (e.g. on an emergency stop). */
void BootAnimation_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_BOOT_ANIMATION_HPP_ */
