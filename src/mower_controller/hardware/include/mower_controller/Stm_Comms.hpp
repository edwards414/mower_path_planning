#ifndef MOWER_CONTROLLER_STM_COMMS_HPP
#define MOWER_CONTROLLER_STM_COMMS_HPP

#include <cstddef>
#include <cstdint>
#include <libserial/SerialPort.h>
#include <optional>
#include <string>
#include <vector>

constexpr double kMaxWheelCommandRadPerSec = 10.0;
constexpr double kMaxMowerBladeEffort = 100.0;
constexpr int16_t kCommandPermilleMax = 1000;
constexpr uint16_t kDefaultCommandTimeoutMs = 200;

class StmComms {
public:
  struct MotorStatus {
    uint8_t seq{0};
    int16_t commanded_left_permille{0};
    int16_t commanded_right_permille{0};
    int16_t applied_left_pwm{0};
    int16_t applied_right_pwm{0};
    uint16_t command_age_ms{0};
    uint8_t flags{0};
    uint8_t last_rx_seq{0};
  };

  StmComms() = default;

  StmComms(const std::string &serial_device, int32_t baud_rate,
           int32_t timeout_ms);

  void setup(const std::string &serial_device, int32_t baud_rate,
             int32_t timeout_ms);
  void setMotorValues(int left_permille, int right_permille,
                      uint16_t command_timeout_ms = kDefaultCommandTimeoutMs);
  void setMowerBladeValue(
      int permille, uint16_t command_timeout_ms = kDefaultCommandTimeoutMs);
  void setWs2812Mode(uint8_t mode, uint8_t red, uint8_t green, uint8_t blue,
                     uint16_t effect_period_ms);
  bool poll();
  std::optional<MotorStatus> getMotorStatus() const;
  bool is_connected() const;
  void setLedOK();
  void setLedError();

private:
  enum class FrameType : uint8_t {
    kMotorOpenLoopCommand = 0x01,
    kMowerBladeOpenLoopCommand = 0x02,
    kWs2812Command = 0x03,
    kMotorStatus = 0x81,
  };

  enum class Ws2812Mode : uint8_t {
    kClear = 0x00,
    kAllOn = 0x01,
    kFlow = 0x02,
    kTurnLeft = 0x03,
    kTurnRight = 0x04,
    kShow = 0x05,
  };

  static uint16_t compute_crc(const std::vector<uint8_t> &bytes);
  static void append_int16_le(std::vector<uint8_t> &payload, int16_t value);
  static void append_uint16_le(std::vector<uint8_t> &payload, uint16_t value);
  static uint16_t read_uint16_le(const std::vector<uint8_t> &payload,
                                 size_t offset);
  static int16_t read_int16_le(const std::vector<uint8_t> &payload,
                               size_t offset);

  void write_frame(FrameType type, const std::vector<uint8_t> &payload);
  void read_incoming_bytes();
  bool process_incoming_frames();
  bool handle_frame(uint8_t type, uint8_t seq,
                    const std::vector<uint8_t> &payload);

  LibSerial::SerialPort serial_conn_;
  int32_t timeout_ms_{1000};
  uint8_t next_sequence_{0};
  std::vector<uint8_t> rx_buffer_;
  MotorStatus motor_status_{};
  bool has_motor_status_{false};
};

#endif // MOWER_CONTROLLER_STM_COMMS_HPP
