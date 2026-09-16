#include "pid_controller.hpp"

namespace {

float clamp_float(float value, float min_value, float max_value) {
  if (value > max_value) {
    return max_value;
  }
  if (value < min_value) {
    return min_value;
  }
  return value;
}

} // namespace

namespace mower {

PidController::PidController()
    : gains_{0.0f, 0.0f, 0.0f, -200.0f, 200.0f, -200.0f, 200.0f},
      integral_(0.0f), previous_error_(0.0f), has_previous_error_(false) {}

void PidController::Configure(const pid_gains_t &gains) {
  gains_ = gains;
  if (gains_.integral_min > gains_.integral_max) {
    float tmp = gains_.integral_min;
    gains_.integral_min = gains_.integral_max;
    gains_.integral_max = tmp;
  }
  if (gains_.output_min > gains_.output_max) {
    float tmp = gains_.output_min;
    gains_.output_min = gains_.output_max;
    gains_.output_max = tmp;
  }
  integral_ = clamp_float(integral_, gains_.integral_min, gains_.integral_max);
}

void PidController::Reset() {
  integral_ = 0.0f;
  previous_error_ = 0.0f;
  has_previous_error_ = false;
}

float PidController::Update(float setpoint, float measurement, float dt_s) {
  if (dt_s <= 0.0f) {
    return 0.0f;
  }

  float error = setpoint - measurement;
  integral_ += error * dt_s;
  integral_ = clamp_float(integral_, gains_.integral_min, gains_.integral_max);

  float derivative = 0.0f;
  if (has_previous_error_) {
    derivative = (error - previous_error_) / dt_s;
  }
  previous_error_ = error;
  has_previous_error_ = true;

  float output = (gains_.kp * error) + (gains_.ki * integral_) +
                 (gains_.kd * derivative);
  return clamp_float(output, gains_.output_min, gains_.output_max);
}

const pid_gains_t &PidController::Gains() const { return gains_; }

} // namespace mower
