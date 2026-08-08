#ifndef MODULE_INC_PID_CONTROLLER_HPP_
#define MODULE_INC_PID_CONTROLLER_HPP_

#include "pid_types.h"

namespace mower {

class PidController {
public:
  PidController();

  void Configure(const pid_gains_t &gains);
  void Reset();
  float Update(float setpoint, float measurement, float dt_s);

  const pid_gains_t &Gains() const;

private:
  pid_gains_t gains_;
  float integral_;
  float previous_error_;
  bool has_previous_error_;
};

} // namespace mower

#endif /* MODULE_INC_PID_CONTROLLER_HPP_ */
