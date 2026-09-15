#include "wheel_controller.hpp"
#include "motor.hpp"
#include "pid_controller.hpp"
#include "tim.h"

namespace {

mower::PidController g_left_pid;
mower::PidController g_right_pid;
controller_settings_t g_runtime_settings;
wheel_controller_status_t g_status = {};
uint32_t g_left_previous_counter = 0U;
uint32_t g_right_previous_counter = 0U;
int32_t g_left_total_counts = 0;
int32_t g_right_total_counts = 0;
bool g_initialized = false;

uint32_t enter_critical(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void exit_critical(uint32_t primask) {
  if (primask == 0U) {
    __enable_irq();
  }
}

int16_t clamp_permille_local(int16_t command) {
  if (command > MOTOR_COMMAND_PERMILLE_LIMIT) {
    return MOTOR_COMMAND_PERMILLE_LIMIT;
  }
  if (command < -MOTOR_COMMAND_PERMILLE_LIMIT) {
    return -MOTOR_COMMAND_PERMILLE_LIMIT;
  }
  return command;
}

int16_t clamp_pwm_local(float pwm) {
  if (pwm > (float)MOTOR_PWM_MAX_COUNTS) {
    return MOTOR_PWM_MAX_COUNTS;
  }
  if (pwm < -(float)MOTOR_PWM_MAX_COUNTS) {
    return -MOTOR_PWM_MAX_COUNTS;
  }
  return (int16_t)pwm;
}

int16_t permille_to_pwm_local(int16_t command_permille) {
  int32_t scaled = (int32_t)clamp_permille_local(command_permille) *
                   (int32_t)MOTOR_PWM_MAX_COUNTS;
  return (int16_t)(scaled / MOTOR_COMMAND_PERMILLE_LIMIT);
}

float permille_to_rpm(int16_t command_permille) {
  return ((float)clamp_permille_local(command_permille) *
          g_runtime_settings.wheel_max_rpm) /
         (float)MOTOR_COMMAND_PERMILLE_LIMIT;
}

float counts_to_rpm(int32_t delta_counts) {
  constexpr float dt_s =
      (float)WHEEL_CONTROLLER_CONTROL_PERIOD_MS / 1000.0f;
  return ((float)delta_counts * 60.0f) /
         (WHEEL_CONTROLLER_ENCODER_COUNTS_PER_REV * dt_s);
}

int32_t timer16_delta(uint32_t current, uint32_t previous) {
  return (int32_t)(int16_t)((uint16_t)current - (uint16_t)previous);
}

int32_t timer32_delta(uint32_t current, uint32_t previous) {
  return (int32_t)(current - previous);
}

bool command_is_zero(int16_t command_permille) {
  return (command_permille <= WHEEL_CONTROLLER_COMMAND_DEADBAND_PERMILLE) &&
         (command_permille >= -WHEEL_CONTROLLER_COMMAND_DEADBAND_PERMILLE);
}

void configure_pid_from_settings(void) {
  g_left_pid.Configure(g_runtime_settings.left_wheel_pid);
  g_right_pid.Configure(g_runtime_settings.right_wheel_pid);
}

void update_status_flags(bool output_enabled) {
  settings_storage_status_t storage_status = {};
  SettingsStorage_GetStatus(&storage_status);

  g_status.flags = 0U;
  if (output_enabled) {
    g_status.flags |= WHEEL_CONTROLLER_FLAG_ENABLED;
  }
  if (g_runtime_settings.closed_loop_enabled != 0U) {
    g_status.flags |= WHEEL_CONTROLLER_FLAG_CLOSED_LOOP;
  }
  if (storage_status.flash_valid) {
    g_status.flags |= WHEEL_CONTROLLER_FLAG_FLASH_SETTINGS_VALID;
  }
  if (storage_status.last_save_ok) {
    g_status.flags |= WHEEL_CONTROLLER_FLAG_LAST_SAVE_OK;
  }
}

void apply_wheel_output(int16_t left_pwm, int16_t right_pwm) {
  motor_set_left_pwm((float)left_pwm);
  motor_set_right_pwm((float)right_pwm);
}

} // namespace

void WheelController_Init(void) {
  SettingsStorage_Init();
  SettingsStorage_GetSnapshot(&g_runtime_settings);
  configure_pid_from_settings();

  g_left_previous_counter = __HAL_TIM_GET_COUNTER(&htim5);
  g_right_previous_counter = __HAL_TIM_GET_COUNTER(&htim1);
  g_left_total_counts = 0;
  g_right_total_counts = 0;
  g_left_pid.Reset();
  g_right_pid.Reset();

  (void)HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  g_initialized = true;
  WheelController_Stop();
}

void WheelController_Update20ms(int16_t left_command_permille,
                                int16_t right_command_permille,
                                bool output_enabled) {
  if (!g_initialized) {
    WheelController_Init();
  }

  uint32_t left_counter = __HAL_TIM_GET_COUNTER(&htim5);
  uint32_t right_counter = __HAL_TIM_GET_COUNTER(&htim1);

  int32_t left_delta = WHEEL_CONTROLLER_LEFT_ENCODER_SIGN *
                       timer32_delta(left_counter, g_left_previous_counter);
  int32_t right_delta = WHEEL_CONTROLLER_RIGHT_ENCODER_SIGN *
                        timer16_delta(right_counter, g_right_previous_counter);

  g_left_previous_counter = left_counter;
  g_right_previous_counter = right_counter;
  g_left_total_counts += left_delta;
  g_right_total_counts += right_delta;

  float left_measured_rpm = counts_to_rpm(left_delta);
  float right_measured_rpm = counts_to_rpm(right_delta);
  float left_target_rpm = permille_to_rpm(left_command_permille);
  float right_target_rpm = permille_to_rpm(right_command_permille);

  int16_t left_pwm = 0;
  int16_t right_pwm = 0;
  float left_pid_output = 0.0f;
  float right_pid_output = 0.0f;

  if (output_enabled) {
    if (g_runtime_settings.closed_loop_enabled != 0U) {
      constexpr float dt_s =
          (float)WHEEL_CONTROLLER_CONTROL_PERIOD_MS / 1000.0f;

      if (!command_is_zero(left_command_permille)) {
        left_pid_output =
            g_left_pid.Update(left_target_rpm, left_measured_rpm, dt_s);
        left_pwm = clamp_pwm_local(left_pid_output);
      } else {
        g_left_pid.Reset();
      }

      if (!command_is_zero(right_command_permille)) {
        right_pid_output =
            g_right_pid.Update(right_target_rpm, right_measured_rpm, dt_s);
        right_pwm = clamp_pwm_local(right_pid_output);
      } else {
        g_right_pid.Reset();
      }
    } else {
      left_pwm = permille_to_pwm_local(left_command_permille);
      right_pwm = permille_to_pwm_local(right_command_permille);
      left_pid_output = (float)left_pwm;
      right_pid_output = (float)right_pwm;
    }
  } else {
    g_left_pid.Reset();
    g_right_pid.Reset();
  }

  apply_wheel_output(left_pwm, right_pwm);

  uint32_t primask = enter_critical();
  g_status.left.target_rpm = left_target_rpm;
  g_status.left.measured_rpm = left_measured_rpm;
  g_status.left.pid_output = left_pid_output;
  g_status.left.applied_pwm = left_pwm;
  g_status.left.delta_counts = left_delta;
  g_status.left.total_counts = g_left_total_counts;
  g_status.left.raw_counter = left_counter;
  g_status.right.target_rpm = right_target_rpm;
  g_status.right.measured_rpm = right_measured_rpm;
  g_status.right.pid_output = right_pid_output;
  g_status.right.applied_pwm = right_pwm;
  g_status.right.delta_counts = right_delta;
  g_status.right.total_counts = g_right_total_counts;
  g_status.right.raw_counter = right_counter;
  update_status_flags(output_enabled);
  exit_critical(primask);
}

void WheelController_Stop(void) {
  apply_wheel_output(0, 0);
  g_left_pid.Reset();
  g_right_pid.Reset();

  uint32_t primask = enter_critical();
  g_status.left.applied_pwm = 0;
  g_status.right.applied_pwm = 0;
  g_status.left.pid_output = 0.0f;
  g_status.right.pid_output = 0.0f;
  update_status_flags(false);
  exit_critical(primask);
}

void WheelController_GetStatus(wheel_controller_status_t *status) {
  if (status == NULL) {
    return;
  }

  uint32_t primask = enter_critical();
  *status = g_status;
  exit_critical(primask);
}

void WheelController_GetSettings(controller_settings_t *settings) {
  if (settings == NULL) {
    return;
  }
  SettingsStorage_GetSnapshot(settings);
}

bool WheelController_ApplySettings(const controller_settings_t *settings,
                                   bool persist_to_flash) {
  if (settings == NULL) {
    return false;
  }

  bool ok = SettingsStorage_Update(settings, persist_to_flash);
  SettingsStorage_GetSnapshot(&g_runtime_settings);
  configure_pid_from_settings();
  g_left_pid.Reset();
  g_right_pid.Reset();
  return ok;
}
