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
  kMotorStatus = 0x81,
  kLawerMotorStatus = 0x82,
  kWs2812Status = 0x83,
  kPidConfigStatus = 0x84,
  kWheelFeedbackStatus = 0x85,
  kPowerStatus = 0x86,
  kFirmwareInfo = 0x87,
  kServoStatus = 0x88,
  kChargerStatus = 0x89,
  kAnalogStatus = 0x8A,
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

// 0x87 payload build_flags
constexpr uint8_t kFwBuildFlagDirty = 0x01;
constexpr uint8_t kFwBuildFlagUnversioned = 0x02;

// 0x89 charger (charge-line V/A/temp meter) status flags
constexpr uint8_t kChargerFlagOnline = 0x01;        // RS485 replies OK
constexpr uint8_t kChargerFlagCharging = 0x02;      // current above threshold
// 0x04 was CV_PHASE for the CC/CV module the firmware was first written
// for; the meter actually fitted has no CV setting and never sets it.
constexpr uint8_t kChargerFlagInputPresent = 0x08;  // line voltage present
constexpr uint8_t kChargerFlagEverSeen = 0x10;      // replied at least once

// 0x8A analog status flags
constexpr uint8_t kAnalogFlagMainBatteryValid = 0x01;
constexpr uint8_t kAnalogFlagAonBatteryValid = 0x02;
constexpr uint8_t kAnalogFlagBoardTempValid = 0x04;
constexpr uint8_t kAnalogFlagMg996CurrentValid = 0x08;
constexpr uint8_t kAnalogFlagVddaCalibrated = 0x10;  // VREFINT read OK
constexpr uint8_t kAnalogFlagMg996LimitActive = 0x20;

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

// 0x89: RS485 voltage / current / temperature meter in the charger-to-
// battery line, polled by the STM32 every 500 ms. It only measures (no
// CC/CV settings). Values are the last valid reply (or 0); trust them only
// when online().
struct ChargerStatus {
  uint16_t voltage_cv = 0;  // charge line = battery terminal voltage, x0.01 V
  uint16_t current_ca = 0;  // charge current, x0.01 A
  uint16_t temp_c = 0;      // meter temperature, degC
  uint16_t reg3 = 0;        // raw holding register 3, meaning unknown
  uint16_t reg4 = 0;        // raw holding register 4, meaning unknown
  uint8_t flags = 0;        // kChargerFlag*
  uint8_t comm_error_count = 0;
  uint16_t age_ms = 0xFFFF;  // since the last valid reply, 0xFFFF = never
  uint8_t last_exception_code = 0;
  bool online() const { return flags & kChargerFlagOnline; }
  bool charging() const { return flags & kChargerFlagCharging; }
  bool input_present() const { return flags & kChargerFlagInputPresent; }
};

// 0x8A: slow analog channels behind the ADC mux, refreshed every 200 ms.
// A field is meaningful only when its kAnalogFlag*Valid bit is set.
struct AnalogStatus {
  uint16_t main_battery_cv = 0;    // 24 V main battery, x0.01 V
  uint16_t aon_battery_cv = 0;     // 3.7 V always-on battery, x0.01 V
  int16_t board_temp_dc = INT16_MIN;  // x0.1 C, INT16_MIN = invalid
  uint16_t vdda_mv = 0;            // ADC reference the STM32 used
  uint16_t mg996_current_raw = 0;  // 12-bit ADC counts
  uint8_t flags = 0;               // kAnalogFlag*
  bool main_battery_valid() const { return flags & kAnalogFlagMainBatteryValid; }
  bool aon_battery_valid() const { return flags & kAnalogFlagAonBatteryValid; }
  bool board_temp_valid() const { return flags & kAnalogFlagBoardTempValid; }
};

uint16_t crc16_ccitt_false(const uint8_t * data, size_t len);

// Build a complete frame (SOF + header + payload + CRC).
std::vector<uint8_t> build_frame(uint8_t type, uint8_t seq, const uint8_t * payload, size_t len);

std::vector<uint8_t> build_wheel_speed_command(
  uint8_t seq, int16_t left_permille, int16_t right_permille, uint16_t timeout_ms);

std::vector<uint8_t> build_lawer_motor_command(
  uint8_t seq, int16_t permille, uint16_t timeout_ms);

std::vector<uint8_t> build_power_command(uint8_t seq, uint8_t action);

std::vector<uint8_t> build_info_request(uint8_t seq);

std::vector<uint8_t> build_ws2812_command(
  uint8_t seq, uint8_t mode, uint8_t r, uint8_t g, uint8_t b, uint16_t effect_period_ms);

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
bool decode_analog_status(const uint8_t * payload, size_t len, AnalogStatus & out);

}  // namespace mower_hardware
