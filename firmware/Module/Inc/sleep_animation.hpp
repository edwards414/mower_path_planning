/*
 * sleep_animation.hpp
 *
 * Shutdown light sequence for the WS2812 strips, the counterpart of
 * boot_animation. Non-blocking: SleepAnimation_Update() renders one frame
 * per call from the 20 ms WS2812 loop and owns the strips while it returns
 * true.
 *
 * Sequence:
 *   1. Exhale  : the running white cools into a dim warm amber.
 *   2. Gather  : the amber retreats from both ends into the centre of each
 *                strip, leaving a soft core glow.
 *   3. Breathe : the core breathes slowly for as long as the host takes to
 *                shut down (until SleepAnimation_Finish()).
 *   4. Fade    : eases to black from wherever the breath was, then the
 *                animation reports done so the rail can be cut.
 *   Buzzer cues: two soft chirps at the start, one short tone as the final
 *                fade begins.
 */
#ifndef MODULE_INC_SLEEP_ANIMATION_HPP_
#define MODULE_INC_SLEEP_ANIMATION_HPP_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start (or restart) the sequence; it breathes until Finish is called. */
void SleepAnimation_Start(void);

/* Begin the final fade to black. Safe to call at any point, even before the
 * gather has completed. */
void SleepAnimation_Finish(void);

/* Render the next frame if it is time. Returns true while the animation
 * still owns the strips, false once it has faded out (strips left dark). */
bool SleepAnimation_Update(void);

bool SleepAnimation_IsActive(void);

/* Abort immediately and blank the strips. */
void SleepAnimation_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_SLEEP_ANIMATION_HPP_ */
