#ifndef MODULE_INC_MG996_SERVO_HPP_
#define MODULE_INC_MG996_SERVO_HPP_

/* MG996 / MG996R servo on PB10, timed by TIM10 (1 us tick, 20 ms period).
 * The TIM10 update interrupt raises PB10, the CH1 compare interrupt drops
 * it after `pulse_us`, so the pulse is hardware-timed (jitter = interrupt
 * latency) and costs no CPU between edges. PB10 has no free timer channel
 * of its own, hence GPIO + timer interrupt instead of a PWM channel. */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MG996_SERVO_MIN_PULSE_US 500U
#define MG996_SERVO_CENTER_PULSE_US 1500U
#define MG996_SERVO_MAX_PULSE_US 2500U
#define MG996_SERVO_PERIOD_US 20000U

#define MG996_SERVO_FLAG_ENABLED 0x01U      /* host asked for pulses */
#define MG996_SERVO_FLAG_LIMIT_ACTIVE 0x02U /* current limit: pulses held off */
#define MG996_SERVO_FLAG_OUTPUT_ACTIVE 0x04U /* pulses are actually going out */
#define MG996_SERVO_FLAG_TIMED_OUT 0x08U    /* hold timeout expired */

typedef struct {
  uint16_t pulse_us;
  uint16_t hold_timeout_ms; /* 0 = hold until the next command */
  uint16_t command_age_ms;
  uint8_t flags;
  uint8_t last_rx_seq;
} mg996_servo_status_t;

void Mg996Servo_Init(void);
/* Timeout bookkeeping; call from the 10 ms board tick. */
void Mg996Servo_Update10ms(void);
/* Start (or re-target) pulses. hold_timeout_ms = 0 keeps the position
 * until the next command; otherwise pulses stop that long after the last
 * command so a dead host lets the mechanism relax. */
void Mg996Servo_SetPulseUs(uint16_t pulse_us, uint16_t hold_timeout_ms,
                           uint8_t rx_seq);
void Mg996Servo_SetAngleDeg(float angle_deg);
void Mg996Servo_SetLimitActive(bool active);
void Mg996Servo_Disable(void);
void Mg996Servo_GetStatus(mg996_servo_status_t *status);
/* TIM1_UP_TIM10 interrupt body (stm32f4xx_it.c). */
void Mg996Servo_TimerIrq(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_MG996_SERVO_HPP_ */
