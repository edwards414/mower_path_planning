#ifndef MODULE_INC_MG996_SERVO_HPP_
#define MODULE_INC_MG996_SERVO_HPP_

/* MG996 / MG996R servo on PB10, timed by TIM10 (1 us tick, 20 ms period).
 * The TIM10 update interrupt raises PB10, the CH1 compare interrupt drops
 * it after `pulse_us`, so the pulse is hardware-timed (jitter = interrupt
 * latency) and costs no CPU between edges. PB10 has no free timer channel
 * of its own, hence GPIO + timer interrupt instead of a PWM channel.
 *
 * The mechanism has a microswitch at each end of travel (hardware_pins.hpp:
 * SERVO_LIMIT_UP / SERVO_LIMIT_DN). The servo has no position feedback, so
 * the pulse is slewed towards the host's target at a rate the servo can
 * follow; the commanded pulse then tracks the real position closely enough
 * that "stop where you are" is meaningful. When a switch closes while the
 * pulse is moving towards it, the target is clamped to the current pulse
 * (LIMIT_ACTIVE) and further motion in that direction is refused until the
 * switch opens; the other direction always works. */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MG996_SERVO_MIN_PULSE_US 500U
#define MG996_SERVO_CENTER_PULSE_US 1500U
#define MG996_SERVO_MAX_PULSE_US 2500U
#define MG996_SERVO_PERIOD_US 20000U
/* Slew rate of the applied pulse, per 10 ms tick. 25 us / 10 ms = full
 * 500-2500 travel in 0.8 s, a little slower than an MG996R at 5 V
 * (~0.2 s / 60 deg = ~33 us / 10 ms) so the pulse never runs ahead of the
 * horn. */
#define MG996_SERVO_SLEW_US_PER_10MS 25U
/* How far to back away from a limit switch once it trips: ~4.5 deg of horn,
 * enough to unload the switch and the end stop. Tune on the real mechanism. */
#define MG996_SERVO_LIMIT_BACKOFF_US 50U

#define MG996_SERVO_FLAG_ENABLED 0x01U      /* host asked for pulses */
#define MG996_SERVO_FLAG_LIMIT_ACTIVE 0x02U /* last target cut short by a switch */
#define MG996_SERVO_FLAG_OUTPUT_ACTIVE 0x04U /* pulses are actually going out */
#define MG996_SERVO_FLAG_TIMED_OUT 0x08U    /* hold timeout expired */
#define MG996_SERVO_FLAG_LIMIT_UP 0x10U     /* UP switch pressed (debounced) */
#define MG996_SERVO_FLAG_LIMIT_DN 0x20U     /* DOWN switch pressed (debounced) */

typedef struct {
  uint16_t pulse_us; /* pulse currently on the wire (slewing to the target) */
  uint16_t hold_timeout_ms; /* 0 = hold until the next command */
  uint16_t command_age_ms;
  uint8_t flags;
  uint8_t last_rx_seq;
} mg996_servo_status_t;

void Mg996Servo_Init(void);
/* Limit switch sampling, slew step and timeout bookkeeping; call from the
 * 10 ms board tick. */
void Mg996Servo_Update10ms(void);
/* Start (or re-target) pulses. The applied pulse slews from where it is to
 * the target; from a released state it restarts at the last applied value
 * (centre after power-on). hold_timeout_ms = 0 keeps the position until the
 * next command; otherwise pulses stop that long after the last command so a
 * dead host lets the mechanism relax. */
void Mg996Servo_SetPulseUs(uint16_t pulse_us, uint16_t hold_timeout_ms,
                           uint8_t rx_seq);
void Mg996Servo_SetAngleDeg(float angle_deg);
/* Debounced state of the two limit switches. */
void Mg996Servo_GetLimits(bool *up_pressed, bool *dn_pressed);
void Mg996Servo_Disable(void);
void Mg996Servo_GetStatus(mg996_servo_status_t *status);
/* TIM1_UP_TIM10 interrupt body (stm32f4xx_it.c). */
void Mg996Servo_TimerIrq(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_MG996_SERVO_HPP_ */
