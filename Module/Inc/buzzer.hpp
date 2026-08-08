#ifndef MODULE_INC_BUZZER_HPP_
#define MODULE_INC_BUZZER_HPP_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool active;
  bool timed_beep_active;
  uint32_t beep_remaining_ms;
} buzzer_status_t;

void Buzzer_Init(void);
void Buzzer_Set(bool on);
void Buzzer_Beep(uint32_t duration_ms);
void Buzzer_Update10ms(void);
void Buzzer_GetStatus(buzzer_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_BUZZER_HPP_ */
