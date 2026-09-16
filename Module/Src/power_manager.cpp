#include "power_manager.hpp"
#include "boot_animation.hpp"
#include "buzzer.hpp"
#include "hardware_pins.hpp"
#include "mg996_servo.hpp"
#include "motor.hpp"
#include "sleep_animation.hpp"

namespace {

power_manager_status_t g_status = {};
uint32_t g_press_start_ms = 0U;
uint32_t g_wake_pulse_end_ms = 0U;
uint32_t g_shutdown_start_ms = 0U;
uint32_t g_host_ack_ms = 0U;
uint32_t g_lights_off_start_ms = 0U;
/* Set after a long press fires; the button must be released before another
 * long press is accepted, otherwise holding through 3 s shuts down and the
 * next 10 ms tick (press_ms already >= 1 s) wakes straight back up. */
bool g_press_consumed = false;
bool g_status_event = false;

void configure_pins(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = POWER_BUTTON_N_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(POWER_BUTTON_N_GPIO_Port, &gpio);

  gpio = {};
  gpio.Pin = LEBANCAT_WAKE_Pin | MAIN_POWER_EN_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);
}

bool read_button_pressed(void) {
  return HAL_GPIO_ReadPin(POWER_BUTTON_N_GPIO_Port, POWER_BUTTON_N_Pin) ==
         GPIO_PIN_RESET;
}

void set_main_power(bool enabled) {
  HAL_GPIO_WritePin(MAIN_POWER_EN_GPIO_Port, MAIN_POWER_EN_Pin,
                    enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
  g_status.main_power_enabled = enabled;
}

void set_lebancat_wake(bool asserted) {
  HAL_GPIO_WritePin(LEBANCAT_WAKE_GPIO_Port, LEBANCAT_WAKE_Pin,
                    asserted ? GPIO_PIN_SET : GPIO_PIN_RESET);
  g_status.lebancat_wake_asserted = asserted;
}

void set_state(power_manager_state_t state) {
  if (g_status.state != state) {
    g_status.state = state;
    g_status_event = true;
  }
}

void stop_all_motion(void) {
  motor_set_left_pwm(0.0f);
  motor_set_right_pwm(0.0f);
  Motor_SetWheelDriversEnabled(false);
  Grass_cutting_motor(0U, 0U); /* PWM 0 + BLD120A BRK low */
  Mg996Servo_Disable();
}

/* Last stage: let the sleep animation fade to black, then cut the rail. */
void begin_lights_off(void) {
  g_lights_off_start_ms = HAL_GetTick();
  SleepAnimation_Finish();
  set_state(POWER_MANAGER_STATE_LIGHTS_OFF);
}

void cut_main_power(void) {
  set_lebancat_wake(false);
  set_main_power(false);
  SleepAnimation_Stop();
  g_status.shutdown_requested = false;
  g_status.host_ack_received = false;
  g_status.shutdown_elapsed_ms = 0U;
  set_state(POWER_MANAGER_STATE_LOW_POWER);
}

} // namespace

void PowerManager_Init(void) {
  configure_pins();
  g_status = {};
  g_status.state = POWER_MANAGER_STATE_RUNNING;
  g_press_start_ms = 0U;
  g_wake_pulse_end_ms = 0U;
  g_shutdown_start_ms = 0U;
  g_host_ack_ms = 0U;
  g_press_consumed = false;
  g_status_event = true;
  set_main_power(true);
  set_lebancat_wake(false);
}

void PowerManager_Update10ms(void) {
  uint32_t now = HAL_GetTick();
  bool pressed = read_button_pressed();

  if (pressed && !g_status.button_pressed) {
    g_press_start_ms = now;
  }

  g_status.button_pressed = pressed;
  g_status.press_ms = pressed ? (now - g_press_start_ms) : 0U;
  if (!pressed) {
    g_press_consumed = false;
  }

  switch (g_status.state) {
  case POWER_MANAGER_STATE_WAKE_PULSE:
    if ((int32_t)(now - g_wake_pulse_end_ms) >= 0) {
      set_lebancat_wake(false);
      set_state(POWER_MANAGER_STATE_RUNNING);
    }
    return;

  case POWER_MANAGER_STATE_SHUTDOWN_PENDING:
    g_status.shutdown_elapsed_ms = now - g_shutdown_start_ms;
    if (g_status.host_ack_received &&
        ((now - g_host_ack_ms) >= POWER_MANAGER_HOST_ACK_GRACE_MS)) {
      begin_lights_off();
    } else if (g_status.shutdown_elapsed_ms >= POWER_MANAGER_HOST_TIMEOUT_MS) {
      begin_lights_off();
    }
    return;

  case POWER_MANAGER_STATE_LIGHTS_OFF:
    g_status.shutdown_elapsed_ms = now - g_shutdown_start_ms;
    if (!SleepAnimation_IsActive() ||
        ((now - g_lights_off_start_ms) >= POWER_MANAGER_LIGHTS_OFF_MAX_MS)) {
      cut_main_power();
    }
    return;

  default:
    break;
  }

  if (!pressed || g_press_consumed) {
    return;
  }

  if ((g_status.state == POWER_MANAGER_STATE_RUNNING) &&
      (g_status.press_ms >= POWER_MANAGER_SHUTDOWN_HOLD_MS)) {
    g_press_consumed = true;
    PowerManager_RequestShutdown(POWER_MANAGER_REASON_BUTTON);
  } else if ((g_status.state == POWER_MANAGER_STATE_LOW_POWER) &&
             (g_status.press_ms >= POWER_MANAGER_WAKE_HOLD_MS)) {
    g_press_consumed = true;
    PowerManager_WakeLebanCatPulse(POWER_MANAGER_WAKE_PULSE_MS);
  }
}

void PowerManager_RequestShutdown(power_manager_shutdown_reason_t reason) {
  if ((g_status.state != POWER_MANAGER_STATE_RUNNING) &&
      (g_status.state != POWER_MANAGER_STATE_WAKE_PULSE)) {
    return;
  }

  stop_all_motion();
  BootAnimation_Stop();
  SleepAnimation_Start();

  g_shutdown_start_ms = HAL_GetTick();
  g_status.shutdown_reason = reason;
  g_status.shutdown_requested = true;
  g_status.host_ack_received = false;
  g_status.shutdown_elapsed_ms = 0U;
  set_state(POWER_MANAGER_STATE_SHUTDOWN_PENDING);
  g_status_event = true; /* push 0x86 now even if state compare missed */
}

void PowerManager_HostShutdownAck(void) {
  if (g_status.state != POWER_MANAGER_STATE_SHUTDOWN_PENDING) {
    return;
  }
  if (!g_status.host_ack_received) {
    g_status.host_ack_received = true;
    g_host_ack_ms = HAL_GetTick();
    g_status_event = true;
  }
}

void PowerManager_CancelShutdown(void) {
  if (g_status.state != POWER_MANAGER_STATE_SHUTDOWN_PENDING) {
    return;
  }
  SleepAnimation_Stop();
  Motor_SetWheelDriversEnabled(true);
  g_status.shutdown_requested = false;
  g_status.host_ack_received = false;
  g_status.shutdown_reason = POWER_MANAGER_REASON_NONE;
  g_status.shutdown_elapsed_ms = 0U;
  set_state(POWER_MANAGER_STATE_RUNNING);
}

void PowerManager_ForcePowerOff(void) {
  if (g_status.state == POWER_MANAGER_STATE_LOW_POWER) {
    return;
  }
  stop_all_motion();
  BootAnimation_Stop();
  g_status.shutdown_reason = POWER_MANAGER_REASON_FORCED;
  cut_main_power();
}

void PowerManager_WakeLebanCatPulse(uint32_t pulse_ms) {
  if (pulse_ms == 0U) {
    pulse_ms = POWER_MANAGER_WAKE_PULSE_MS;
  }

  set_main_power(true);
  Motor_SetWheelDriversEnabled(true);
  set_lebancat_wake(true);
  g_status.shutdown_reason = POWER_MANAGER_REASON_NONE;
  g_wake_pulse_end_ms = HAL_GetTick() + pulse_ms;
  set_state(POWER_MANAGER_STATE_WAKE_PULSE);
  BootAnimation_Start();
}

void PowerManager_GetStatus(power_manager_status_t *status) {
  if (status == nullptr) {
    return;
  }
  *status = g_status;
}

bool PowerManager_MotionAllowed(void) {
  return (g_status.state == POWER_MANAGER_STATE_RUNNING) ||
         (g_status.state == POWER_MANAGER_STATE_WAKE_PULSE);
}

bool PowerManager_ConsumeStatusEvent(void) {
  bool event = g_status_event;
  g_status_event = false;
  return event;
}
