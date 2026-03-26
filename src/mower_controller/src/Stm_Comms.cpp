#include "mower_controller/Stm_Comms.hpp"

#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

using namespace LibSerial;

namespace {

constexpr uint8_t kSof0 = 0xA5;
constexpr uint8_t kSof1 = 0x5A;
constexpr uint8_t kProtocolVersion = 0x01;
constexpr uint16_t kDefaultErrorLedPeriodMs = 200;
constexpr size_t kMinimumFrameSize = 8;
constexpr size_t kMotorStatusPayloadSize = 12;
constexpr std::array<uint8_t, 2> kFramePreamble{kSof0, kSof1};

} // namespace

StmComms::StmComms(const std::string &serial_device, int32_t baud_rate,
                   int32_t timeout_ms)
    : timeout_ms_(timeout_ms) {
  setup(serial_device, baud_rate, timeout_ms);
}

void StmComms::setup(const std::string &serial_device, int32_t baud_rate,
                     int32_t timeout_ms) {
  timeout_ms_ = timeout_ms;
  next_sequence_ = 0;
  rx_buffer_.clear();
  motor_status_ = {};
  has_motor_status_ = false;

  if (serial_conn_.IsOpen()) {
    serial_conn_.Close();
  }

  // Open the serial port
  serial_conn_.Open(serial_device);

  // Set baud rate
  BaudRate baud;
  switch (baud_rate) {
  case 9600:
    baud = BaudRate::BAUD_9600;
    break;
  case 19200:
    baud = BaudRate::BAUD_19200;
    break;
  case 38400:
    baud = BaudRate::BAUD_38400;
    break;
  case 57600:
    baud = BaudRate::BAUD_57600;
    break;
  case 115200:
    baud = BaudRate::BAUD_115200;
    break;
  case 230400:
    baud = BaudRate::BAUD_230400;
    break;
  default:
    baud = BaudRate::BAUD_115200;
    break;
  }
  serial_conn_.SetBaudRate(baud);

  // Set serial port parameters
  serial_conn_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
  serial_conn_.SetParity(Parity::PARITY_NONE);
  serial_conn_.SetStopBits(StopBits::STOP_BITS_1);
  serial_conn_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
}

uint16_t StmComms::compute_crc(const std::vector<uint8_t> &bytes) {
  uint16_t crc = 0xFFFF;

  for (const uint8_t byte : bytes) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1U);
      }
    }
  }

  return crc;
}

void StmComms::append_int16_le(std::vector<uint8_t> &payload, int16_t value) {
  payload.push_back(
      static_cast<uint8_t>(static_cast<uint16_t>(value) & 0x00FF));
  payload.push_back(static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) &
                                         0x00FF));
}

void StmComms::append_uint16_le(std::vector<uint8_t> &payload,
                                uint16_t value) {
  payload.push_back(static_cast<uint8_t>(value & 0x00FF));
  payload.push_back(static_cast<uint8_t>((value >> 8) & 0x00FF));
}

uint16_t StmComms::read_uint16_le(const std::vector<uint8_t> &payload,
                                  size_t offset) {
  return static_cast<uint16_t>(payload.at(offset)) |
         static_cast<uint16_t>(payload.at(offset + 1)) << 8U;
}

int16_t StmComms::read_int16_le(const std::vector<uint8_t> &payload,
                                size_t offset) {
  return static_cast<int16_t>(read_uint16_le(payload, offset));
}

void StmComms::write_frame(FrameType type,
                           const std::vector<uint8_t> &payload) {
  if (!serial_conn_.IsOpen()) {
    RCLCPP_WARN(rclcpp::get_logger("StmComms"),
                "Serial port is not open, dropping frame type 0x%02X",
                static_cast<unsigned int>(type));
    return;
  }

  if (payload.size() > 0xFF) {
    RCLCPP_WARN(rclcpp::get_logger("StmComms"),
                "Payload too large for frame type 0x%02X: %zu bytes",
                static_cast<unsigned int>(type), payload.size());
    return;
  }

  std::vector<uint8_t> frame;
  frame.reserve(8 + payload.size());

  frame.push_back(kSof0);
  frame.push_back(kSof1);
  frame.push_back(kProtocolVersion);
  frame.push_back(static_cast<uint8_t>(type));
  frame.push_back(next_sequence_++);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  const std::vector<uint8_t> crc_input(frame.begin() + 2, frame.end());
  const uint16_t crc = compute_crc(crc_input);
  frame.push_back(static_cast<uint8_t>(crc & 0x00FF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0x00FF));

  try {
    serial_conn_.Write(std::string(frame.begin(), frame.end()));
    serial_conn_.DrainWriteBuffer();
  } catch (const std::exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("StmComms"),
                "Error writing frame type 0x%02X: %s",
                static_cast<unsigned int>(type), e.what());
  }
}

void StmComms::setMotorValues(int left_permille, int right_permille,
                              uint16_t command_timeout_ms) {
  std::vector<uint8_t> payload;
  payload.reserve(8);
  append_int16_le(payload, static_cast<int16_t>(std::clamp(
                              left_permille, -static_cast<int>(kCommandPermilleMax),
                              static_cast<int>(kCommandPermilleMax))));
  append_int16_le(payload, static_cast<int16_t>(std::clamp(
                              right_permille, -static_cast<int>(kCommandPermilleMax),
                              static_cast<int>(kCommandPermilleMax))));
  append_uint16_le(payload, command_timeout_ms);
  append_uint16_le(payload, 0);

  write_frame(FrameType::kMotorOpenLoopCommand, payload);
}

void StmComms::setMowerBladeValue(int permille, uint16_t command_timeout_ms) {
  std::vector<uint8_t> payload;
  payload.reserve(8);
  append_int16_le(payload, static_cast<int16_t>(std::clamp(
                              permille, -static_cast<int>(kCommandPermilleMax),
                              static_cast<int>(kCommandPermilleMax))));
  append_uint16_le(payload, command_timeout_ms);
  append_uint16_le(payload, 0);
  append_uint16_le(payload, 0);

  write_frame(FrameType::kMowerBladeOpenLoopCommand, payload);
}

void StmComms::setWs2812Mode(uint8_t mode, uint8_t red, uint8_t green,
                             uint8_t blue, uint16_t effect_period_ms) {
  std::vector<uint8_t> payload;
  payload.reserve(8);
  payload.push_back(mode);
  payload.push_back(red);
  payload.push_back(green);
  payload.push_back(blue);
  append_uint16_le(payload, effect_period_ms);
  payload.push_back(0);
  payload.push_back(0);

  write_frame(FrameType::kWs2812Command, payload);
}

void StmComms::setLedOK() {
  setWs2812Mode(static_cast<uint8_t>(Ws2812Mode::kAllOn), 0, 255, 0, 0);
}

void StmComms::setLedError() {
  setWs2812Mode(static_cast<uint8_t>(Ws2812Mode::kFlow), 255, 0, 0,
                kDefaultErrorLedPeriodMs);
}

bool StmComms::poll() {
  if (!serial_conn_.IsOpen()) {
    return false;
  }

  read_incoming_bytes();
  return process_incoming_frames();
}

std::optional<StmComms::MotorStatus> StmComms::getMotorStatus() const {
  if (!has_motor_status_) {
    return std::nullopt;
  }

  return motor_status_;
}

void StmComms::read_incoming_bytes() {
  while (serial_conn_.IsDataAvailable()) {
    char data_byte = 0;

    try {
      serial_conn_.ReadByte(data_byte, 0);
      rx_buffer_.push_back(static_cast<uint8_t>(data_byte));
    } catch (const std::exception &e) {
      RCLCPP_WARN(rclcpp::get_logger("StmComms"),
                  "Error reading UART status stream: %s", e.what());
      break;
    }
  }
}

bool StmComms::process_incoming_frames() {
  bool did_update_motor_status = false;

  while (rx_buffer_.size() >= kMinimumFrameSize) {
    const auto sof = std::search(rx_buffer_.begin(), rx_buffer_.end(),
                                 kFramePreamble.begin(), kFramePreamble.end());

    if (sof == rx_buffer_.end()) {
      if (!rx_buffer_.empty() && rx_buffer_.back() == kSof0) {
        rx_buffer_.assign(1, kSof0);
      } else {
        rx_buffer_.clear();
      }
      break;
    }

    if (sof != rx_buffer_.begin()) {
      rx_buffer_.erase(rx_buffer_.begin(), sof);
    }

    if (rx_buffer_.size() < kMinimumFrameSize) {
      break;
    }

    const size_t payload_size = rx_buffer_[5];
    const size_t frame_size = kMinimumFrameSize + payload_size;
    if (rx_buffer_.size() < frame_size) {
      break;
    }

    const std::vector<uint8_t> crc_input(rx_buffer_.begin() + 2,
                                         rx_buffer_.begin() + 6 + payload_size);
    const uint16_t expected_crc = compute_crc(crc_input);
    const uint16_t received_crc =
        read_uint16_le(rx_buffer_, 6 + payload_size);

    if (rx_buffer_[2] != kProtocolVersion || expected_crc != received_crc) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const std::vector<uint8_t> payload(rx_buffer_.begin() + 6,
                                       rx_buffer_.begin() + 6 + payload_size);
    did_update_motor_status |=
        handle_frame(rx_buffer_[3], rx_buffer_[4], payload);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_size);
  }

  return did_update_motor_status;
}

bool StmComms::handle_frame(uint8_t type, uint8_t seq,
                            const std::vector<uint8_t> &payload) {
  if (type != static_cast<uint8_t>(FrameType::kMotorStatus)) {
    return false;
  }

  if (payload.size() != kMotorStatusPayloadSize) {
    RCLCPP_WARN(rclcpp::get_logger("StmComms"),
                "Ignoring 0x81 motor status with unexpected payload length %zu",
                payload.size());
    return false;
  }

  motor_status_.seq = seq;
  motor_status_.commanded_left_permille = read_int16_le(payload, 0);
  motor_status_.commanded_right_permille = read_int16_le(payload, 2);
  motor_status_.applied_left_pwm = read_int16_le(payload, 4);
  motor_status_.applied_right_pwm = read_int16_le(payload, 6);
  motor_status_.command_age_ms = read_uint16_le(payload, 8);
  motor_status_.flags = payload[10];
  motor_status_.last_rx_seq = payload[11];
  has_motor_status_ = true;
  return true;
}

bool StmComms::is_connected() const { return serial_conn_.IsOpen(); }
