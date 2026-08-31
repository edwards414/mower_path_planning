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
constexpr uint8_t kCommandValidMask = 0x01;
constexpr uint8_t kCommandTimeoutMask = 0x02;
constexpr uint8_t kDriverAlarmMask = 0x04;

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
  has_motor_status_time_ = false;
  has_ack_progress_time_ = false;
  last_acknowledged_sequence_.reset();
  recent_motor_commands_.clear();
  io_fault_ = false;

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

std::optional<uint8_t>
StmComms::write_frame(FrameType type, const std::vector<uint8_t> &payload) {
  if (!serial_conn_.IsOpen()) {
    RCLCPP_WARN(rclcpp::get_logger("StmComms"),
                "Serial port is not open, dropping frame type 0x%02X",
                static_cast<unsigned int>(type));
    return std::nullopt;
  }

  if (payload.size() > 0xFF) {
    RCLCPP_WARN(rclcpp::get_logger("StmComms"),
                "Payload too large for frame type 0x%02X: %zu bytes",
                static_cast<unsigned int>(type), payload.size());
    return std::nullopt;
  }

  std::vector<uint8_t> frame;
  frame.reserve(8 + payload.size());

  frame.push_back(kSof0);
  frame.push_back(kSof1);
  frame.push_back(kProtocolVersion);
  frame.push_back(static_cast<uint8_t>(type));
  const uint8_t sequence = next_sequence_++;
  frame.push_back(sequence);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  const std::vector<uint8_t> crc_input(frame.begin() + 2, frame.end());
  const uint16_t crc = compute_crc(crc_input);
  frame.push_back(static_cast<uint8_t>(crc & 0x00FF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0x00FF));

  try {
    serial_conn_.Write(std::string(frame.begin(), frame.end()));
    serial_conn_.DrainWriteBuffer();
    return sequence;
  } catch (const std::exception &e) {
    mark_io_fault("writing UART frame", e);
    return std::nullopt;
  }
}

std::optional<uint8_t>
StmComms::setMotorValues(int left_permille, int right_permille,
                         uint16_t command_timeout_ms) {
  const auto clamped_left = static_cast<int16_t>(std::clamp(
      left_permille, -static_cast<int>(kCommandPermilleMax),
      static_cast<int>(kCommandPermilleMax)));
  const auto clamped_right = static_cast<int16_t>(std::clamp(
      right_permille, -static_cast<int>(kCommandPermilleMax),
      static_cast<int>(kCommandPermilleMax)));
  std::vector<uint8_t> payload;
  payload.reserve(8);
  append_int16_le(payload, clamped_left);
  append_int16_le(payload, clamped_right);
  append_uint16_le(payload, command_timeout_ms);
  append_uint16_le(payload, 0);

  const auto sequence = write_frame(FrameType::kMotorOpenLoopCommand, payload);
  if (sequence.has_value()) {
    const auto now = std::chrono::steady_clock::now();
    recent_motor_commands_.push_back(MotorCommandRecord{
        *sequence,
        clamped_left,
        clamped_right,
        command_timeout_ms,
        now,
    });
    while (!recent_motor_commands_.empty() &&
           now - recent_motor_commands_.front().sent_at >
               std::chrono::seconds(2)) {
      recent_motor_commands_.pop_front();
    }
  }
  return sequence;
}

bool StmComms::setMowerBladeValue(int permille,
                                  uint16_t command_timeout_ms) {
  std::vector<uint8_t> payload;
  payload.reserve(8);
  append_int16_le(payload, static_cast<int16_t>(std::clamp(
                              permille, -static_cast<int>(kCommandPermilleMax),
                              static_cast<int>(kCommandPermilleMax))));
  append_uint16_le(payload, command_timeout_ms);
  append_uint16_le(payload, 0);
  append_uint16_le(payload, 0);

  return write_frame(FrameType::kMowerBladeOpenLoopCommand, payload).has_value();
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

  try {
    read_incoming_bytes();
  } catch (const std::exception &error) {
    mark_io_fault("polling UART status", error);
    return false;
  }
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
      mark_io_fault("reading UART status", e);
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

  MotorStatus candidate{};
  candidate.seq = seq;
  candidate.commanded_left_permille = read_int16_le(payload, 0);
  candidate.commanded_right_permille = read_int16_le(payload, 2);
  candidate.applied_left_pwm = read_int16_le(payload, 4);
  candidate.applied_right_pwm = read_int16_le(payload, 6);
  candidate.command_age_ms = read_uint16_le(payload, 8);
  candidate.flags = payload[10];
  candidate.last_rx_seq = payload[11];
  if (seq != candidate.last_rx_seq) {
    RCLCPP_WARN(rclcpp::get_logger("StmComms"),
                "Ignoring inconsistent motor status seq=%u last_rx_seq=%u",
                seq, candidate.last_rx_seq);
    return false;
  }

  const auto command = std::find_if(
      recent_motor_commands_.begin(), recent_motor_commands_.end(),
      [&](const MotorCommandRecord &record) {
        return record.seq == candidate.last_rx_seq;
      });
  if (command == recent_motor_commands_.end()) {
    RCLCPP_WARN(rclcpp::get_logger("StmComms"),
                "Ignoring motor status for unknown command seq=%u",
                candidate.last_rx_seq);
    return false;
  }
  if (candidate.commanded_left_permille != command->left_permille ||
      candidate.commanded_right_permille != command->right_permille) {
    RCLCPP_ERROR(
        rclcpp::get_logger("StmComms"),
        "Ignoring motor status seq=%u with mismatched command echo",
        candidate.last_rx_seq);
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!last_acknowledged_sequence_.has_value() ||
      *last_acknowledged_sequence_ != candidate.last_rx_seq) {
    last_acknowledged_sequence_ = candidate.last_rx_seq;
    last_ack_progress_at_ = now;
    has_ack_progress_time_ = true;
  }
  motor_status_ = candidate;
  has_motor_status_ = true;
  last_motor_status_at_ = now;
  has_motor_status_time_ = true;
  return true;
}

void StmComms::mark_io_fault(const char *operation,
                             const std::exception &error) {
  io_fault_ = true;
  RCLCPP_ERROR(rclcpp::get_logger("StmComms"), "%s failed: %s", operation,
               error.what());
  try {
    if (serial_conn_.IsOpen()) {
      serial_conn_.Close();
    }
  } catch (const std::exception &close_error) {
    RCLCPP_ERROR(rclcpp::get_logger("StmComms"),
                 "Closing faulted UART failed: %s", close_error.what());
  }
}

bool StmComms::is_connected() const {
  return !io_fault_ && serial_conn_.IsOpen();
}

bool StmComms::motor_status_is_fresh_and_acknowledged(
    std::chrono::milliseconds max_age,
    std::optional<uint8_t> expected_sequence) const {
  if (!is_connected() || !has_motor_status_ || !has_motor_status_time_) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now - last_motor_status_at_ > max_age) {
    return false;
  }
  if (expected_sequence.has_value() &&
      motor_status_.last_rx_seq != *expected_sequence) {
    return false;
  }
  if (!has_ack_progress_time_ || now - last_ack_progress_at_ > max_age) {
    return false;
  }
  const auto command = std::find_if(
      recent_motor_commands_.begin(), recent_motor_commands_.end(),
      [&](const MotorCommandRecord &record) {
        return record.seq == motor_status_.last_rx_seq;
      });
  if (command == recent_motor_commands_.end() ||
      now - command->sent_at > max_age ||
      motor_status_.commanded_left_permille != command->left_permille ||
      motor_status_.commanded_right_permille != command->right_permille ||
      motor_status_.command_age_ms > command->timeout_ms ||
      motor_status_.command_age_ms > max_age.count()) {
    return false;
  }
  const bool command_valid =
      (motor_status_.flags & kCommandValidMask) != 0U;
  const bool command_timeout =
      (motor_status_.flags & kCommandTimeoutMask) != 0U;
  const bool driver_alarm =
      (motor_status_.flags & kDriverAlarmMask) != 0U;
  constexpr uint8_t kKnownFlags =
      kCommandValidMask | kCommandTimeoutMask | kDriverAlarmMask;
  const bool unknown_flag = (motor_status_.flags & ~kKnownFlags) != 0U;
  return command_valid && !command_timeout && !driver_alarm && !unknown_flag;
}
