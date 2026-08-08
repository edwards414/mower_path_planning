#ifndef MODULE_INC_MG996_SERVO_HPP_
#define MODULE_INC_MG996_SERVO_HPP_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MG996_SERVO_MIN_PULSE_US 500U
#define MG996_SERVO_CENTER_PULSE_US 1500U
#define MG996_SERVO_MAX_PULSE_US 2500U
#define MG996_SERVO_PERIOD_MS 20U

#define MG996_SERVO_FLAG_ENABLED 0x01U
#define MG996_SERVO_FLAG_LIMIT_ACTIVE 0x02U

typedef struct {
  uint16_t pulse_us;
  uint8_t flags;
} mg996_servo_status_t;

void Mg996Servo_Init(void);
void Mg996Servo_SetPulseUs(uint16_t pulse_us);
void Mg996Servo_SetAngleDeg(float angle_deg);
void Mg996Servo_SetLimitActive(bool active);
void Mg996Servo_Disable(void);
void Mg996Servo_Update20ms(void);
void Mg996Servo_GetStatus(mg996_servo_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_MG996_SERVO_HPP_ */
