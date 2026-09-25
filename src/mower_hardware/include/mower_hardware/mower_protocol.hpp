// Frame builder / parser for the STM32 mower UART protocol
// (see UART_OPEN_LOOP_PROTOCOL.md at the repo root). No ROS dependencies.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mower_hardware {

constexpr uint8_t kSof0 = 0xA5;
constexpr uint8_t kSof1 = 0x5A;
constexpr uint8_t kProtocolVersion = 0x01;
constexpr size_t kFrameOverhead = 8;  // sof0 sof1 ver type seq len ... crc_lo crc_hi

enum FrameType : uint8_t {
  kWheelSpeedCommand = 0x01,
  kLawerMotorCommand = 0x02,
  kWs2812Command = 0x03,
  kPidConfigCommand = 0x04,
  kPowerCommand = 0x05,
  kInfoRequest = 0x06,
  kServoCommand = 0x07,
  kMotorStatus = 0x81,
  kLawerMotorStatus = 0x82,
  kWs2812Status = 0x83,
  kPidConfigStatus = 0x84,
  kWheelFeedbackStatus = 0x85,
  kPowerStatus = 0x86,
  kFirmwareInfo = 0x87,
  kServoStatus = 0x88,
  kChargerStatus = 0x89,
  // 0x8A (analog status) is retired: the ADC mux was never fitted.
};

// 0x03 payload mode (firmware/LED_COMMAND_MODES.md)
enum Ws2812Mode : uint8_t {
  kLedClear = 0x00,
  kLedAllOn = 0x01,
  kLedFlow = 0x02,
  kLedTurnLeft = 0x03,
  kLedTurnRight = 0x04,
  kLedShow = 0x05,
  kLedOrbit = 0x06,  // smooth comet around the strips (update indicator)
};

// 0x03 payload byte 6 `overlay`: drawn on top of the mode, back strip only
constexpr uint8_t kLedOverlayRearRecording = 0x01;  // red breath while a bag records
// 0x83 flags: bit0 = command valid; set when the firmware applies the overlay
constexpr uint8_t kLedStatusFlagRearRecording = 0x08;

// 0x87 payload build_flags
constexpr uint8_t kFwBuildFlagDirty = 0x01;
constexpr uint8_t kFwBuildFlagUnversioned = 0x02;

// 0x89 charger (charge-line V/A/temp meter) status flags
constexpr uint8_t kChargerFlagOnline = 0x01;        // RS485 replies OK
constexpr uint8_t kChargerFlagCurrentPresent = 0x02; // |current| above threshold (flow, not direction)
// 0x04 was CV_PHASE for the CC/CV module the firmware was first written
// for; the meter actually fitted has no CV setting and never sets it.
constexpr uint8_t kChargerFlagInputPresent = 0x08;  // line voltage present
constexpr uint8_t kChargerFlagEverSeen = 0x10;      // replied at least once

// 0x88 MG996 servo status flags
constexpr uint8_t kServoFlagEnabled = 0x01;       // host asked for pulses
constexpr uint8_t kServoFlagLimitActive = 0x02;   // last target cut short by a limit switch
constexpr uint8_t kServoFlagOutputActive = 0x04;  // pulses are going out
constexpr uint8_t kServoFlagTimedOut = 0x08;      // hold timeout expired
constexpr uint8_t kServoFlagLimitUp = 0x10;       // UP switch pressed (debounced)
constexpr uint8_t kServoFlagLimitDown = 0x20;     // DOWN switch pressed (debounced)

// 0x07 servo command pulse range; 0 releases the servo
constexpr uint16_t kServoMinPulseUs = 500;
constexpr uint16_t kServoMaxPulseUs = 2500;

// 0x05 payload action
enum PowerAction : uint8_t {
  kPowerActionNone = 0x00,
  kPowerActionHostShutdownAck = 0x01,  // host is halting; STM32 cuts the rail after its grace
  kPowerActionRequestShutdown = 0x02,  // same flow as a 3 s button press
  kPowerActionCancelShutdown = 0x03,
  kPowerActionForcePowerOff = 0x04,    // cut the rail now, no hand-shake
};

// 0x86 payload state (power_manager_state_t on the STM32)
enum PowerState : uint8_t {
  kPowerStateRunning = 0,
  kPowerStateLowPower = 1,
  kPowerStateWakePulse = 2,
  kPowerStateShutdownPending = 3,
  kPowerStateLightsOff = 4,
};

// 0x86 payload flags
constexpr uint8_t kPowerFlagButtonPressed = 0x01;
constexpr uint8_t kPowerFlagMainPowerEnabled = 0x02;
constexpr uint8_t kPowerFlagShutdownRequested = 0x04;
constexpr uint8_t kPowerFlagHostAckReceived = 0x08;
constexpr uint8_t kPowerFlagWakeAsserted = 0x10;

// 0x81 payload flags
constexpr uint8_t kStatusFlagCommandValid = 0x01;
constexpr uint8_t kStatusFlagCommandTimeout = 0x02;
constexpr uint8_t kStatusFlagDriverAlarm = 0x04;

// 0x85 payload flags
constexpr uint8_t kWheelFlagOutputEnabled = 0x01;
constexpr uint8_t kWheelFlagClosedLoop = 0x02;

struct WheelFeedback {
  double left_target_rpm = 0.0;
  double left_measured_rpm = 0.0;
  double right_target_rpm = 0.0;
  double right_measured_rpm = 0.0;
  int16_t left_pid_output = 0;
  int16_t right_pid_output = 0;
  int32_t left_total_counts = 0;
  int32_t right_total_counts = 0;
  uint8_t flags = 0;
  uint8_t seq = 0;
};

struct MotorStatus {
  int16_t commanded_left_permille = 0;
  int16_t commanded_right_permille = 0;
  int16_t applied_left_pwm = 0;
  int16_t applied_right_pwm = 0;
  uint16_t command_age_ms = 0;
  uint8_t flags = 0;
  uint8_t last_rx_seq = 0;
};

// 0x83: what the strips are currently showing
struct Ws2812Status {
  uint8_t mode = 0;
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  uint16_t effect_period_ms = 0;
  uint8_t flags = 0;
  uint8_t last_rx_seq = 0;
};

// 0x84 payload flags
constexpr uint8_t kPidFlagClosedLoop = 0x01;
constexpr uint8_t kPidFlagFlashValid = 0x02;
constexpr uint8_t kPidFlagLastSaveOk = 0x04;
constexpr uint8_t kPidFlagLastApplyOk = 0x08;

// 0x84: PID gains the wheel controller is running with
struct PidConfigStatus {
  float left_kp = 0.0f;
  float left_ki = 0.0f;
  float left_kd = 0.0f;
  float right_kp = 0.0f;
  float right_ki = 0.0f;
  float right_kd = 0.0f;
  uint8_t flags = 0;
  uint8_t last_rx_seq = 0;
  // low byte: HAL flash error of the last failed persist (0 = ok), high
  // byte: FLASH_SR error bits found pending before it; 0 on older firmware
  uint16_t flash_diag = 0;
};

// 0x04: gains to run with (and optionally persist). Same float layout as
// PidConfigStatus; the STM32 sanitises out-of-range values itself.
struct PidConfig {
  float left_kp = 0.0f;
  float left_ki = 0.0f;
  float left_kd = 0.0f;
  float right_kp = 0.0f;
  float right_ki = 0.0f;
  float right_kd = 0.0f;
  bool persist_to_flash = false;   // write sector 7 (do not do this at high rate)
  bool closed_loop_enabled = true; // false: 0x01 permille drives PWM duty directly
};

struct PowerStatus {
  uint8_t state = 0;
  uint8_t flags = 0;
  uint8_t shutdown_reason = 0;  // 0 none, 1 button, 2 host, 3 forced
  uint8_t last_rx_seq = 0;
  uint16_t press_ms = 0;
  uint16_t shutdown_elapsed_ms = 0;
  bool shutdown_requested() const { return flags & kPowerFlagShutdownRequested; }
};

// 0x87: build identity of the running application (firmware_version.h)
struct FirmwareInfo {
  uint8_t major = 0;
  uint8_t minor = 0;
  uint8_t patch = 0;
  uint8_t protocol_version = 0;
  uint32_t git_sha32 = 0;   // first 4 bytes of the commit hash, 0 = unknown
  uint32_t build_unix = 0;  // build time, 0 = unknown
  uint8_t build_flags = 0;  // kFwBuildFlag*
  bool dirty() const { return build_flags & kFwBuildFlagDirty; }
  bool unversioned() const { return build_flags & kFwBuildFlagUnversioned; }
  bool operator==(const FirmwareInfo & o) const
  {
    return major == o.major && minor == o.minor && patch == o.patch &&
           protocol_version == o.protocol_version && git_sha32 == o.git_sha32 &&
           build_unix == o.build_unix && build_flags == o.build_flags;
  }
  bool operator!=(const FirmwareInfo & o) const { return !(*this == o); }
  // "0.6.0", "0.6.0+abc12345.dirty" ... for logs
  std::string version_string() const;
  // JSON with all fields, what /mower_base/firmware_info carries
  std::string to_json() const;
};

// 0x89: RS485 voltage / current / temperature meter in the battery pack
// lead, polled by the STM32 every 500 ms. It only measures (no CC/CV
// settings); the current is 0 until its shunt is wired into the lead.
// Values are the last valid reply (or 0); trust them only when online().
struct ChargerStatus {
  uint16_t voltage_cv = 0;  // battery terminal voltage, x0.01 V
  uint16_t current_ca = 0;  // pack current magnitude, x0.01 A
  uint16_t temp_c = 0;      // meter temperature, degC
  uint16_t reg3 = 0;        // raw holding register 3, meaning unknown
  uint16_t reg4 = 0;        // raw holding register 4, meaning unknown
  uint8_t flags = 0;        // kChargerFlag*
  uint8_t comm_error_count = 0;
  uint16_t age_ms = 0xFFFF;  // since the last valid reply, 0xFFFF = never
  uint8_t last_exception_code = 0;
  bool online() const { return flags & kChargerFlagOnline; }
  bool current_present() const { return flags & kChargerFlagCurrentPresent; }
  bool input_present() const { return flags & kChargerFlagInputPresent; }
};

// 0x88: MG996 blade-lift servo, every 50 ms. pulse_us is the pulse on the
// wire, slewing towards the last 0x07 target; the STM32 stops and backs off
// a little when a limit microswitch trips (kServoFlagLimitActive).
struct ServoStatus {
  uint16_t pulse_us = 0;
  uint16_t hold_timeout_ms = 0;
  uint16_t command_age_ms = 0;
  uint8_t flags = 0;  // kServoFlag*
  uint8_t last_rx_seq = 0;
  bool enabled() const { return flags & kServoFlagEnabled; }
  bool limit_active() const { return flags & kServoFlagLimitActive; }
  bool output_active() const { return flags & kServoFlagOutputActive; }
  bool timed_out() const { return flags & kServoFlagTimedOut; }
  bool limit_up() const { return flags & kServoFlagLimitUp; }
  bool limit_down() const { return flags & kServoFlagLimitDown; }
};

// 0x82: BLD120A blade motor, every 50 ms. flags share the 0x81 layout
// (kStatusFlag*); the motor is one-directional, negative commands read as 0.
struct LawerMotorStatus {
  int16_t commanded_permille = 0;
  int16_t applied_pwm = 0;
  uint16_t command_age_ms = 0;
  uint8_t flags = 0;
  uint8_t last_rx_seq = 0;
};

uint16_t crc16_ccitt_false(const uint8_t * data, size_t len);

// Build a complete frame (SOF + header + payload + CRC).
std::vector<uint8_t> build_frame(uint8_t type, uint8_t seq, const uint8_t * payload, size_t len);

std::vector<uint8_t> build_wheel_speed_command(
  uint8_t seq, int16_t left_permille, int16_t right_permille, uint16_t timeout_ms);

std::vector<uint8_t> build_lawer_motor_command(
  uint8_t seq, int16_t permille, uint16_t timeout_ms);

std::vector<uint8_t> build_power_command(uint8_t seq, uint8_t action);
// pulse_us 500-2500 (0 = release); hold_timeout_ms 0 = hold until the next command
std::vector<uint8_t> build_servo_command(uint8_t seq, uint16_t pulse_us, uint16_t hold_timeout_ms);

std::vector<uint8_t> build_info_request(uint8_t seq);

std::vector<uint8_t> build_ws2812_command(
  uint8_t seq, uint8_t mode, uint8_t r, uint8_t g, uint8_t b, uint16_t effect_period_ms,
  uint8_t overlay = 0);

std::vector<uint8_t> build_pid_config_command(uint8_t seq, const PidConfig & cfg);

// Incremental parser: feed bytes, get callbacks for every CRC-valid frame.
class FrameParser {
public:
  using Callback = std::function<void(uint8_t type, uint8_t seq, const uint8_t * payload, size_t len)>;

  void feed(const uint8_t * data, size_t len, const Callback & on_frame);
  size_t crc_errors() const { return crc_errors_; }

private:
  std::vector<uint8_t> buf_;
  size_t crc_errors_ = 0;
};

// Decoders return false when the payload length does not match.
bool decode_wheel_feedback(uint8_t seq, const uint8_t * payload, size_t len, WheelFeedback & out);
bool decode_motor_status(const uint8_t * payload, size_t len, MotorStatus & out);
bool decode_power_status(const uint8_t * payload, size_t len, PowerStatus & out);
bool decode_ws2812_status(const uint8_t * payload, size_t len, Ws2812Status & out);
bool decode_pid_config_status(const uint8_t * payload, size_t len, PidConfigStatus & out);
bool decode_firmware_info(const uint8_t * payload, size_t len, FirmwareInfo & out);
bool decode_charger_status(const uint8_t * payload, size_t len, ChargerStatus & out);
bool decode_servo_status(const uint8_t * payload, size_t len, ServoStatus & out);
bool decode_lawer_motor_status(const uint8_t * payload, size_t len, LawerMotorStatus & out);

}  // namespace mower_hardware
