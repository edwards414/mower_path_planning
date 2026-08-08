#ifndef MODULE_INC_PID_TYPES_H_
#define MODULE_INC_PID_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral_min;
  float integral_max;
  float output_min;
  float output_max;
} pid_gains_t;

#ifdef __cplusplus
}
#endif

#endif /* MODULE_INC_PID_TYPES_H_ */
