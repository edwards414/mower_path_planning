#include "mower_hardware/mower_protocol.hpp"

#include <cstdio>
#include <cstring>

namespace mower_hardware {

uint16_t crc16_ccitt_false(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<uint8_t> build_frame(uint8_t type, uint8_t seq, const uint8_t * payload, size_t len)
{
  std::vector<uint8_t> f;
  f.reserve(kFrameOverhead + len);
  f.push_back(kSof0);
  f.push_back(kSof1);
  f.push_back(kProtocolVersion);
  f.push_back(type);
  f.push_back(seq);
  f.push_back(static_cast<uint8_t>(len));
  f.insert(f.end(), payload, payload + len);
  uint16_t crc = crc16_ccitt_false(f.data() + 2, 4 + len);  // version..payload
  f.push_back(static_cast<uint8_t>(crc & 0xFF));
  f.push_back(static_cast<uint8_t>(crc >> 8));
  return f;
}

namespace {

inline void put_i16(uint8_t * p, int16_t v)
{
  uint16_t u = static_cast<uint16_t>(v);
  p[0] = static_cast<uint8_t>(u & 0xFF);
  p[1] = static_cast<uint8_t>(u >> 8);
}
inline void put_u16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline int16_t get_i16(const uint8_t * p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}
inline uint16_t get_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
inline uint32_t get_u32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline int32_t get_i32(const uint8_t * p)
{
  uint32_t u = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  return static_cast<int32_t>(u);
}

}  // namespace

std::vector<uint8_t> build_wheel_speed_command(
  uint8_t seq, int16_t left_permille, int16_t right_permille, uint16_t timeout_ms)
{
  uint8_t p[8] = {};
  put_i16(p + 0, left_permille);
  put_i16(p + 2, right_permille);
  put_u16(p + 4, timeout_ms);
  put_u16(p + 6, 0);
  return build_frame(kWheelSpeedCommand, seq, p, sizeof(p));
}

std::vector<uint8_t> build_lawer_motor_command(uint8_t seq, int16_t permille, uint16_t timeout_ms)
{
  uint8_t p[8] = {};
  put_i16(p + 0, permille);
  put_u16(p + 2, timeout_ms);
  put_u16(p + 4, 0);
  put_u16(p + 6, 0);
  return build_frame(kLawerMotorCommand, seq, p, sizeof(p));
}

std::vector<uint8_t> build_power_command(uint8_t seq, uint8_t action)
{
  uint8_t p[4] = {action, 0, 0, 0};
  return build_frame(kPowerCommand, seq, p, sizeof(p));
}

void FrameParser::feed(const uint8_t * data, size_t len, const Callback & on_frame)
{
  buf_.insert(buf_.end(), data, data + len);

  for (;;) {
    // find SOF
    size_t i = 0;
    while (i + 1 < buf_.size() && !(buf_[i] == kSof0 && buf_[i + 1] == kSof1)) {
      ++i;
    }
    if (i + 1 >= buf_.size()) {
      // keep a trailing 0xA5 in case 0x5A arrives next
      if (!buf_.empty() && buf_.back() == kSof0) {
        buf_.erase(buf_.begin(), buf_.end() - 1);
      } else {
        buf_.clear();
      }
      return;
    }
    if (i > 0) {
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(i));
    }
    if (buf_.size() < kFrameOverhead) {
      return;
    }
    size_t plen = buf_[5];
    size_t total = kFrameOverhead + plen;
    if (buf_.size() < total) {
      return;
    }
    uint16_t crc_rx = get_u16(buf_.data() + 6 + plen);
    uint16_t crc_calc = crc16_ccitt_false(buf_.data() + 2, 4 + plen);
    if (crc_rx == crc_calc && buf_[2] == kProtocolVersion) {
      on_frame(buf_[3], buf_[4], buf_.data() + 6, plen);
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(total));
    } else {
      ++crc_errors_;
      buf_.erase(buf_.begin(), buf_.begin() + 2);  // resync past this SOF
    }
  }
}

bool decode_wheel_feedback(uint8_t seq, const uint8_t * p, size_t len, WheelFeedback & out)
{
  if (len != 24) {
    return false;
  }
  out.left_target_rpm = get_i16(p + 0) / 100.0;
  out.left_measured_rpm = get_i16(p + 2) / 100.0;
  out.right_target_rpm = get_i16(p + 4) / 100.0;
  out.right_measured_rpm = get_i16(p + 6) / 100.0;
  out.left_pid_output = get_i16(p + 8);
  out.right_pid_output = get_i16(p + 10);
  out.left_total_counts = get_i32(p + 12);
  out.right_total_counts = get_i32(p + 16);
  out.flags = p[20];
  out.seq = seq;
  return true;
}

bool decode_motor_status(const uint8_t * p, size_t len, MotorStatus & out)
{
  if (len != 12) {
    return false;
  }
  out.commanded_left_permille = get_i16(p + 0);
  out.commanded_right_permille = get_i16(p + 2);
  out.applied_left_pwm = get_i16(p + 4);
  out.applied_right_pwm = get_i16(p + 6);
  out.command_age_ms = get_u16(p + 8);
  out.flags = p[10];
  out.last_rx_seq = p[11];
  return true;
}

std::vector<uint8_t> build_info_request(uint8_t seq)
{
  return build_frame(kInfoRequest, seq, nullptr, 0);
}

std::vector<uint8_t> build_ws2812_command(
  uint8_t seq, uint8_t mode, uint8_t r, uint8_t g, uint8_t b, uint16_t effect_period_ms)
{
  uint8_t p[8] = {mode, r, g, b, 0, 0, 0, 0};
  put_u16(p + 4, effect_period_ms);
  return build_frame(kWs2812Command, seq, p, sizeof(p));
}

bool decode_firmware_info(const uint8_t * p, size_t len, FirmwareInfo & out)
{
  if (len != 16) {
    return false;
  }
  out.major = p[0];
  out.minor = p[1];
  out.patch = p[2];
  out.protocol_version = p[3];
  out.git_sha32 = get_u32(p + 4);
  out.build_unix = get_u32(p + 8);
  out.build_flags = p[12];
  return true;
}

std::string FirmwareInfo::version_string() const
{
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%u.%u.%u+%08x%s%s", major, minor, patch, git_sha32,
    dirty() ? ".dirty" : "", unversioned() ? ".unversioned" : "");
  return buf;
}

std::string FirmwareInfo::to_json() const
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "{\"version\":\"%u.%u.%u\",\"semver\":[%u,%u,%u],\"protocol_version\":%u,"
    "\"git_sha\":\"%08x\",\"git_sha32\":%u,\"build_unix\":%u,\"dirty\":%s,\"unversioned\":%s}",
    major, minor, patch, major, minor, patch, protocol_version, git_sha32, git_sha32, build_unix,
    dirty() ? "true" : "false", unversioned() ? "true" : "false");
  return buf;
}

bool decode_power_status(const uint8_t * p, size_t len, PowerStatus & out)
{
  if (len != 8) {
    return false;
  }
  out.state = p[0];
  out.flags = p[1];
  out.shutdown_reason = p[2];
  out.last_rx_seq = p[3];
  out.press_ms = get_u16(p + 4);
  out.shutdown_elapsed_ms = get_u16(p + 6);
  return true;
}

}  // namespace mower_hardware
