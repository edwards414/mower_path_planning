#include "mower_hardware/mower_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mower_hardware {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace {

constexpr double kTwoPi = 6.283185307179586;

double param_or(const hardware_interface::HardwareInfo & info, const char * key, double def)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? def : std::stod(it->second);
}

std::string param_or(const hardware_interface::HardwareInfo & info, const char * key, const char * def)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? std::string(def) : it->second;
}

}  // namespace

CallbackReturn MowerSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  device_ = param_or(info, "device", "/dev/ttyS0");
  baud_ = static_cast<int>(param_or(info, "baud", 115200.0));
  max_rpm_ = param_or(info, "max_rpm", 58.0);
  counts_per_rev_ = param_or(info, "counts_per_rev", 8896.0);
  command_timeout_ms_ = static_cast<int>(param_or(info, "command_timeout_ms", 300.0));
  feedback_timeout_s_ = param_or(info, "feedback_timeout_s", 0.5);
  shutdown_command_ = param_or(info, "shutdown_command", shutdown_command_.c_str());
  firmware_info_topic_ = param_or(info, "firmware_info_topic", firmware_info_topic_.c_str());
  led_topic_ = param_or(info, "led_topic", led_topic_.c_str());
  pid_topic_ = param_or(info, "pid_topic", pid_topic_.c_str());
  override_topic_ = param_or(info, "override_topic", override_topic_.c_str());
  telemetry_topic_ = param_or(info, "telemetry_topic", telemetry_topic_.c_str());
  telemetry_rate_hz_ = param_or(info, "telemetry_rate_hz", telemetry_rate_hz_);

  if (info.joints.size() != 2) {
    RCLCPP_ERROR(logger(), "expected exactly 2 joints (left, right), got %zu", info.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto & j : info.joints) {
    bool has_vel_cmd = j.command_interfaces.size() == 1 &&
                       j.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY;
    bool has_states = j.state_interfaces.size() == 2;
    if (!has_vel_cmd || !has_states) {
      RCLCPP_ERROR(logger(),
        "joint '%s' must have one velocity command and position+velocity state interfaces",
        j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  left_.joint_name = param_or(info, "left_joint", info.joints[0].name.c_str());
  right_.joint_name = param_or(info, "right_joint", info.joints[1].name.c_str());

  RCLCPP_INFO(logger(), "device=%s baud=%d max_rpm=%.1f counts_per_rev=%.0f left=%s right=%s",
    device_.c_str(), baud_, max_rpm_, counts_per_rev_, left_.joint_name.c_str(), right_.joint_name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MowerSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!port_.open(device_, baud_)) {
    RCLCPP_ERROR(logger(), "serial open failed: %s", port_.last_error().c_str());
    return CallbackReturn::ERROR;
  }
  left_.have_counts = right_.have_counts = false;
  left_.pos = right_.pos = 0.0;
  feedback_valid_ = false;
  warned_timeout_ = false;
  shutdown_acked_ = false;
  firmware_info_valid_ = false;

  // Helper node for the side channels (firmware info out, light requests
  // in). It is spun on its own thread; the control loop only touches
  // atomics. transient_local keeps the last message for late joiners.
  if (!info_node_) {
    try {
      info_node_ = std::make_shared<rclcpp::Node>("mower_hardware_info");
      if (!firmware_info_topic_.empty()) {
        firmware_info_pub_ = info_node_->create_publisher<std_msgs::msg::String>(
          firmware_info_topic_, rclcpp::QoS(1).transient_local().reliable());
      }
      if (!telemetry_topic_.empty() && telemetry_rate_hz_ > 0.0) {
        telemetry_pub_ = info_node_->create_publisher<std_msgs::msg::String>(
          telemetry_topic_, rclcpp::QoS(1).best_effort());
      }
      if (!led_topic_.empty()) {
        led_sub_ = info_node_->create_subscription<std_msgs::msg::String>(
          led_topic_, rclcpp::QoS(1).transient_local().reliable(),
          [this](const std_msgs::msg::String & msg) { on_led_command(msg); });
      }
      if (!pid_topic_.empty()) {
        pid_sub_ = info_node_->create_subscription<std_msgs::msg::String>(
          pid_topic_, rclcpp::QoS(4).reliable(),
          [this](const std_msgs::msg::String & msg) { on_pid_command(msg); });
      }
      if (!override_topic_.empty()) {
        override_sub_ = info_node_->create_subscription<std_msgs::msg::String>(
          override_topic_, rclcpp::QoS(1).best_effort(),
          [this](const std_msgs::msg::String & msg) { on_wheel_override(msg); });
      }
      node_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      node_executor_->add_node(info_node_);
      node_thread_ = std::thread([this]() { node_executor_->spin(); });
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "side-channel node unavailable: %s", e.what());
      stop_node_thread();
      led_sub_.reset();
      pid_sub_.reset();
      override_sub_.reset();
      firmware_info_pub_.reset();
      telemetry_pub_.reset();
      info_node_.reset();
    }
  }

  RCLCPP_INFO(logger(), "opened %s", device_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MowerSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  port_.close();
  stop_node_thread();
  led_sub_.reset();
  pid_sub_.reset();
  override_sub_.reset();
  firmware_info_pub_.reset();
  info_node_.reset();
  return CallbackReturn::SUCCESS;
}

MowerSystem::~MowerSystem()
{
  stop_node_thread();
}

void MowerSystem::stop_node_thread()
{
  if (node_executor_) {
    node_executor_->cancel();
  }
  if (node_thread_.joinable()) {
    node_thread_.join();
  }
  node_executor_.reset();
}

uint64_t MowerSystem::pack_led(
  uint8_t mode, uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms, uint8_t serial)
{
  // serial makes an identical repeated request distinguishable (re-assert)
  return (static_cast<uint64_t>(serial) << 48) | (static_cast<uint64_t>(period_ms) << 32) |
         (static_cast<uint64_t>(mode) << 24) | (static_cast<uint64_t>(r) << 16) |
         (static_cast<uint64_t>(g) << 8) | static_cast<uint64_t>(b);
}

namespace
{
// tiny extractor for the flat JSON the light topic carries; tolerant of
// spacing and missing keys (missing -> def)
long json_int(const std::string & s, const char * key, long def)
{
  std::string k = std::string("\"") + key + "\"";
  auto pos = s.find(k);
  if (pos == std::string::npos) {
    return def;
  }
  pos = s.find(':', pos + k.size());
  if (pos == std::string::npos) {
    return def;
  }
  ++pos;
  while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) {
    ++pos;
  }
  char * end = nullptr;
  long v = std::strtol(s.c_str() + pos, &end, 10);
  if (end == s.c_str() + pos) {
    return def;
  }
  return v;
}

// same, for a float value; `key` may be nested one level ("left" -> "kp")
double json_double(const std::string & s, const char * section, const char * key, double def)
{
  size_t from = 0;
  if (section) {
    std::string sec = std::string("\"") + section + "\"";
    from = s.find(sec);
    if (from == std::string::npos) {
      return def;
    }
    from += sec.size();
  }
  std::string k = std::string("\"") + key + "\"";
  auto pos = s.find(k, from);
  if (pos == std::string::npos) {
    return def;
  }
  pos = s.find(':', pos + k.size());
  if (pos == std::string::npos) {
    return def;
  }
  ++pos;
  while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) {
    ++pos;
  }
  char * end = nullptr;
  double v = std::strtod(s.c_str() + pos, &end);
  if (end == s.c_str() + pos || !std::isfinite(v)) {
    return def;
  }
  return v;
}
}  // namespace

void MowerSystem::on_led_command(const std_msgs::msg::String & msg)
{
  const long mode = json_int(msg.data, "mode", -1);
  if (mode < 0 || mode > 255) {
    RCLCPP_WARN(logger(), "ignoring light request without a valid mode: %s", msg.data.c_str());
    return;
  }
  auto clamp8 = [](long v) { return static_cast<uint8_t>(std::clamp(v, 0L, 255L)); };
  const uint16_t period = static_cast<uint16_t>(std::clamp(json_int(msg.data, "period_ms", 0), 0L, 65535L));
  static uint8_t serial = 0;
  led_request_.store(pack_led(
    static_cast<uint8_t>(mode), clamp8(json_int(msg.data, "r", 0)), clamp8(json_int(msg.data, "g", 0)),
    clamp8(json_int(msg.data, "b", 0)), period, ++serial));
  RCLCPP_INFO(logger(), "light request: mode %ld rgb(%ld,%ld,%ld) period %u ms",
    mode, json_int(msg.data, "r", 0), json_int(msg.data, "g", 0), json_int(msg.data, "b", 0), period);
}

void MowerSystem::send_led_if_needed(const rclcpp::Time & now)
{
  const uint64_t req = led_request_.load();
  if (req == 0) {
    return;
  }
  const bool changed = req != led_sent_;
  const bool stale = led_sent_time_.nanoseconds() == 0 ||
                     (now - led_sent_time_).seconds() >= kLedResendPeriodS;
  if (!changed && !stale) {
    return;
  }
  auto f = build_ws2812_command(
    tx_seq_++, static_cast<uint8_t>(req >> 24), static_cast<uint8_t>(req >> 16),
    static_cast<uint8_t>(req >> 8), static_cast<uint8_t>(req), static_cast<uint16_t>(req >> 32));
  if (port_.write_all(f.data(), f.size())) {
    led_sent_ = req;
    led_sent_time_ = now;
  }
}

void MowerSystem::on_pid_command(const std_msgs::msg::String & msg)
{
  PidConfig cfg;
  const double nan = std::nan("");
  const double lkp = json_double(msg.data, "left", "kp", nan);
  const double rkp = json_double(msg.data, "right", "kp", nan);
  if (std::isnan(lkp) || std::isnan(rkp)) {
    RCLCPP_WARN(logger(), "ignoring pid request without left/right kp: %s", msg.data.c_str());
    return;
  }
  cfg.left_kp = static_cast<float>(lkp);
  cfg.left_ki = static_cast<float>(json_double(msg.data, "left", "ki", 0.0));
  cfg.left_kd = static_cast<float>(json_double(msg.data, "left", "kd", 0.0));
  cfg.right_kp = static_cast<float>(rkp);
  cfg.right_ki = static_cast<float>(json_double(msg.data, "right", "ki", 0.0));
  cfg.right_kd = static_cast<float>(json_double(msg.data, "right", "kd", 0.0));
  cfg.persist_to_flash = json_int(msg.data, "persist", 0) != 0;
  cfg.closed_loop_enabled = json_int(msg.data, "closed_loop", 1) != 0;
  {
    std::lock_guard<std::mutex> lock(pid_mutex_);
    pid_request_ = cfg;
  }
  pid_serial_.fetch_add(1);
  RCLCPP_INFO(logger(), "pid request: L %.3f/%.3f/%.3f R %.3f/%.3f/%.3f persist=%d closed_loop=%d",
    cfg.left_kp, cfg.left_ki, cfg.left_kd, cfg.right_kp, cfg.right_ki, cfg.right_kd,
    cfg.persist_to_flash, cfg.closed_loop_enabled);
}

void MowerSystem::send_pid_if_needed()
{
  const uint32_t serial = pid_serial_.load();
  if (serial == pid_sent_serial_) {
    return;
  }
  PidConfig cfg;
  {
    std::lock_guard<std::mutex> lock(pid_mutex_);
    cfg = pid_request_;
  }
  auto f = build_pid_config_command(tx_seq_++, cfg);
  if (port_.write_all(f.data(), f.size())) {
    pid_sent_serial_ = serial;
  }
}

void MowerSystem::on_wheel_override(const std_msgs::msg::String & msg)
{
  const long ttl = std::clamp(json_int(msg.data, "ttl_ms", 0), 0L, kOverrideMaxTtlMs);
  const long l = std::clamp(json_int(msg.data, "left_permille", 0), -1000L, 1000L);
  const long r = std::clamp(json_int(msg.data, "right_permille", 0), -1000L, 1000L);
  override_request_.store(
    (static_cast<uint64_t>(ttl) << 32) |
    (static_cast<uint64_t>(static_cast<uint16_t>(static_cast<int16_t>(l))) << 16) |
    static_cast<uint64_t>(static_cast<uint16_t>(static_cast<int16_t>(r))));
  override_serial_.fetch_add(1);
}

bool MowerSystem::override_permille(const rclcpp::Time & now, int16_t & left, int16_t & right)
{
  const uint32_t serial = override_serial_.load();
  if (serial != override_seen_serial_) {
    override_seen_serial_ = serial;
    const uint64_t req = override_request_.load();
    override_until_ = now + rclcpp::Duration::from_seconds(static_cast<double>(req >> 32) / 1000.0);
    left = static_cast<int16_t>(static_cast<uint16_t>(req >> 16));
    right = static_cast<int16_t>(static_cast<uint16_t>(req));
    override_left_ = left;
    override_right_ = right;
  }
  const bool active = override_until_.nanoseconds() != 0 && now < override_until_;
  if (active != override_active_) {
    override_active_ = active;
    RCLCPP_INFO(logger(), active ? "wheel override active (controller command bypassed)"
                                 : "wheel override expired, back to controller command");
  }
  if (!active) {
    return false;
  }
  left = override_left_;
  right = override_right_;
  return true;
}

CallbackReturn MowerSystem::on_activate(const rclcpp_lifecycle::State &)
{
  left_.cmd_velocity = right_.cmd_velocity = 0.0;
  send_stop();
  // The firmware also sends 0x87 unsolicited every second; asking makes the
  // version show up in the log right away.
  auto f = build_info_request(tx_seq_++);
  port_.write_all(f.data(), f.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MowerSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  send_stop();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MowerSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> s;
  s.emplace_back(left_.joint_name, hardware_interface::HW_IF_POSITION, &left_.pos);
  s.emplace_back(left_.joint_name, hardware_interface::HW_IF_VELOCITY, &left_.vel);
  s.emplace_back(right_.joint_name, hardware_interface::HW_IF_POSITION, &right_.pos);
  s.emplace_back(right_.joint_name, hardware_interface::HW_IF_VELOCITY, &right_.vel);
  // diagnostics (read with a generic "gpio"/sensor broadcaster or just for debugging)
  s.emplace_back("mower_base", "wheel_flags", &diag_flags_);
  s.emplace_back("mower_base", "motor_flags", &diag_motor_flags_);
  s.emplace_back("mower_base", "command_age_ms", &diag_command_age_ms_);
  s.emplace_back("mower_base", "crc_errors", &diag_crc_errors_);
  s.emplace_back("mower_base", "feedback_age_s", &diag_feedback_age_s_);
  s.emplace_back("mower_base", "power_state", &diag_power_state_);
  s.emplace_back("mower_base", "power_flags", &diag_power_flags_);
  s.emplace_back("mower_base", "firmware_version", &diag_fw_version_);
  s.emplace_back("mower_base", "firmware_git_sha32", &diag_fw_git_sha32_);
  s.emplace_back("mower_base", "firmware_protocol", &diag_fw_protocol_);
  return s;
}

std::vector<hardware_interface::CommandInterface> MowerSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> c;
  c.emplace_back(left_.joint_name, hardware_interface::HW_IF_VELOCITY, &left_.cmd_velocity);
  c.emplace_back(right_.joint_name, hardware_interface::HW_IF_VELOCITY, &right_.cmd_velocity);
  return c;
}

void MowerSystem::handle_frame(uint8_t type, uint8_t seq, const uint8_t * payload, size_t len,
  const rclcpp::Time & now)
{
  if (type == kWheelFeedbackStatus) {
    WheelFeedback fb;
    if (!decode_wheel_feedback(seq, payload, len, fb)) {
      return;
    }
    const double rad_per_count = kTwoPi / counts_per_rev_;
    auto integrate = [&](Wheel & w, int32_t total, double rpm) {
      if (w.have_counts) {
        // signed 32-bit difference handles wrap-around
        int32_t d = static_cast<int32_t>(static_cast<uint32_t>(total) - static_cast<uint32_t>(w.last_total_counts));
        w.pos += d * rad_per_count;
      }
      w.last_total_counts = total;
      w.have_counts = true;
      w.vel = rpm * kTwoPi / 60.0;
    };
    integrate(left_, fb.left_total_counts, fb.left_measured_rpm);
    integrate(right_, fb.right_total_counts, fb.right_measured_rpm);
    diag_flags_ = fb.flags;
    last_wheel_feedback_ = fb;
    last_feedback_time_ = now;
    feedback_valid_ = true;
    telemetry_pending_ = true;
    if (warned_timeout_) {
      RCLCPP_INFO(logger(), "feedback resumed");
      warned_timeout_ = false;
    }
  } else if (type == kMotorStatus) {
    MotorStatus ms;
    if (decode_motor_status(payload, len, ms)) {
      last_motor_status_ = ms;
      have_motor_status_ = true;
      diag_motor_flags_ = ms.flags;
      diag_command_age_ms_ = ms.command_age_ms;
      if (ms.flags & kStatusFlagDriverAlarm) {
        RCLCPP_WARN_THROTTLE(logger(), throttle_clock_, 2000, "driver alarm flag set");
      }
    }
  } else if (type == kPowerStatus) {
    PowerStatus ps;
    if (decode_power_status(payload, len, ps)) {
      last_power_status_ = ps;
      have_power_status_ = true;
      on_power_status(ps);
    }
  } else if (type == kPidConfigStatus) {
    PidConfigStatus st;
    if (decode_pid_config_status(payload, len, st)) {
      last_pid_config_ = st;
      have_pid_config_ = true;
    }
  } else if (type == kWs2812Status) {
    Ws2812Status st;
    if (decode_ws2812_status(payload, len, st)) {
      last_ws2812_status_ = st;
      have_ws2812_status_ = true;
    }
  } else if (type == kFirmwareInfo) {
    FirmwareInfo fi;
    if (decode_firmware_info(payload, len, fi)) {
      on_firmware_info(fi);
    }
  } else if (type == kChargerStatus) {
    ChargerStatus st;
    if (decode_charger_status(payload, len, st)) {
      last_charger_status_ = st;
      have_charger_status_ = true;
    }
  } else if (type == kAnalogStatus) {
    AnalogStatus st;
    if (decode_analog_status(payload, len, st)) {
      last_analog_status_ = st;
      have_analog_status_ = true;
    }
  }
}

void MowerSystem::on_firmware_info(const FirmwareInfo & fi)
{
  if (firmware_info_valid_ && fi == firmware_info_) {
    return;
  }
  firmware_info_ = fi;
  firmware_info_valid_ = true;
  diag_fw_version_ = fi.major * 1000000.0 + fi.minor * 1000.0 + fi.patch;
  diag_fw_git_sha32_ = fi.git_sha32;
  diag_fw_protocol_ = fi.protocol_version;

  RCLCPP_INFO(logger(), "STM32 firmware %s (protocol %u, built %u)",
    fi.version_string().c_str(), fi.protocol_version, fi.build_unix);
  if (fi.protocol_version != kProtocolVersion) {
    RCLCPP_ERROR(logger(), "firmware speaks protocol %u, this driver expects %u",
      fi.protocol_version, kProtocolVersion);
  }
  if (firmware_info_pub_) {
    std_msgs::msg::String msg;
    msg.data = fi.to_json();
    firmware_info_pub_->publish(msg);
  }
}

void MowerSystem::on_power_status(const PowerStatus & ps)
{
  diag_power_state_ = ps.state;
  diag_power_flags_ = ps.flags;

  if (!ps.shutdown_requested()) {
    shutdown_acked_ = false;  // request cleared (cancelled or rail cut)
    return;
  }
  if (shutdown_acked_) {
    return;
  }
  shutdown_acked_ = true;

  RCLCPP_WARN(logger(), "STM32 requests shutdown (reason %u): acking and running '%s'",
    ps.shutdown_reason, shutdown_command_.c_str());
  auto f = build_power_command(tx_seq_++, kPowerActionHostShutdownAck);
  if (!port_.write_all(f.data(), f.size())) {
    RCLCPP_ERROR(logger(), "power ack write failed: %s", port_.last_error().c_str());
  }
  if (!shutdown_command_.empty()) {
    // Detached so the control loop keeps acking status frames while the OS halts.
    std::string cmd = shutdown_command_ + " &";
    int rc = std::system(cmd.c_str());
    if (rc != 0) {
      RCLCPP_ERROR(logger(), "shutdown command returned %d", rc);
    }
  }
}

return_type MowerSystem::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  uint8_t buf[512];
  for (;;) {
    int n = port_.read_some(buf, sizeof(buf));
    if (n < 0) {
      RCLCPP_ERROR_THROTTLE(logger(), throttle_clock_, 2000, "serial read error: %s",
        port_.last_error().c_str());
      return return_type::ERROR;
    }
    if (n == 0) {
      break;
    }
    parser_.feed(buf, static_cast<size_t>(n),
      [&](uint8_t t, uint8_t s, const uint8_t * p, size_t l) { handle_frame(t, s, p, l, time); });
  }
  diag_crc_errors_ = static_cast<double>(parser_.crc_errors());
  publish_telemetry_if_due(time);

  if (feedback_valid_) {
    diag_feedback_age_s_ = (time - last_feedback_time_).seconds();
    if (diag_feedback_age_s_ > feedback_timeout_s_) {
      left_.vel = right_.vel = 0.0;
      if (!warned_timeout_) {
        RCLCPP_WARN(logger(), "no wheel feedback for %.2f s", diag_feedback_age_s_);
        warned_timeout_ = true;
      }
    }
  }
  return return_type::OK;
}

void MowerSystem::publish_telemetry_if_due(const rclcpp::Time & now)
{
  if (!telemetry_pub_ || !feedback_valid_ || !telemetry_pending_) {
    return;
  }
  // One message per new 0x85 (every 50 ms), so the PID auto-tune sees every
  // sample instead of a 50 Hz loop beating against a 20 Hz gate. The rate
  // parameter only throttles when set below the frame rate; the 0.8 keeps
  // frame jitter from dropping every other message at the nominal 20 Hz.
  const double period_s = 0.8 / telemetry_rate_hz_;
  if (telemetry_sent_time_.nanoseconds() != 0 && (now - telemetry_sent_time_).seconds() < period_s) {
    return;
  }
  telemetry_pending_ = false;
  telemetry_sent_time_ = now;
  std_msgs::msg::String msg;
  msg.data = telemetry_json(now);
  telemetry_pub_->publish(msg);
}

std::string MowerSystem::telemetry_json(const rclcpp::Time & now) const
{
  const WheelFeedback & fb = last_wheel_feedback_;
  const MotorStatus & ms = last_motor_status_;
  const PidConfigStatus & pid = last_pid_config_;
  const Ws2812Status & led = last_ws2812_status_;
  const PowerStatus & ps = last_power_status_;
  const ChargerStatus & ch = last_charger_status_;
  const AnalogStatus & an = last_analog_status_;
  char buf[1536];
  std::snprintf(buf, sizeof(buf),
    "{\"t\":%.3f,\"feedback_age_s\":%.3f,\"crc_errors\":%u,"
    "\"wheel\":{\"left\":{\"target_rpm\":%.2f,\"measured_rpm\":%.2f,\"pid_output\":%d,\"total_counts\":%d},"
    "\"right\":{\"target_rpm\":%.2f,\"measured_rpm\":%.2f,\"pid_output\":%d,\"total_counts\":%d},"
    "\"flags\":%u,\"seq\":%u},"
    "\"motor\":{\"valid\":%s,\"cmd_left_permille\":%d,\"cmd_right_permille\":%d,"
    "\"pwm_left\":%d,\"pwm_right\":%d,\"command_age_ms\":%u,\"flags\":%u},"
    "\"pid\":{\"valid\":%s,\"left\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},"
    "\"right\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},\"flags\":%u,\"last_rx_seq\":%u,\"flash_diag\":%u},"
    "\"led\":{\"valid\":%s,\"mode\":%u,\"r\":%u,\"g\":%u,\"b\":%u,\"period_ms\":%u,\"flags\":%u},"
    "\"power\":{\"valid\":%s,\"state\":%u,\"flags\":%u,\"shutdown_reason\":%u,"
    "\"press_ms\":%u,\"shutdown_elapsed_ms\":%u},"
    "\"charger\":{\"valid\":%s,\"online\":%s,\"charging\":%s,\"cv_phase\":%s,\"input_present\":%s,"
    "\"vin_v\":%.2f,\"vout_v\":%.2f,\"iout_a\":%.2f,\"set_cc_a\":%.2f,\"set_cv_v\":%.2f,"
    "\"flags\":%u,\"comm_errors\":%u,\"age_ms\":%u},"
    "\"analog\":{\"valid\":%s,\"main_battery_v\":%.2f,\"main_battery_valid\":%s,"
    "\"aon_battery_v\":%.2f,\"aon_battery_valid\":%s,\"board_temp_c\":%.1f,\"board_temp_valid\":%s,"
    "\"vdda_mv\":%u,\"vdda_calibrated\":%s,\"mg996_current_raw\":%u,\"flags\":%u}}",
    now.seconds(), diag_feedback_age_s_, static_cast<unsigned>(parser_.crc_errors()),
    fb.left_target_rpm, fb.left_measured_rpm, fb.left_pid_output, fb.left_total_counts,
    fb.right_target_rpm, fb.right_measured_rpm, fb.right_pid_output, fb.right_total_counts,
    fb.flags, fb.seq,
    have_motor_status_ ? "true" : "false", ms.commanded_left_permille, ms.commanded_right_permille,
    ms.applied_left_pwm, ms.applied_right_pwm, ms.command_age_ms, ms.flags,
    have_pid_config_ ? "true" : "false", pid.left_kp, pid.left_ki, pid.left_kd,
    pid.right_kp, pid.right_ki, pid.right_kd, pid.flags, pid.last_rx_seq, pid.flash_diag,
    have_ws2812_status_ ? "true" : "false", led.mode, led.r, led.g, led.b, led.effect_period_ms, led.flags,
    have_power_status_ ? "true" : "false", ps.state, ps.flags, ps.shutdown_reason,
    ps.press_ms, ps.shutdown_elapsed_ms,
    have_charger_status_ ? "true" : "false", ch.online() ? "true" : "false",
    ch.charging() ? "true" : "false", ch.cv_phase() ? "true" : "false",
    ch.input_present() ? "true" : "false",
    ch.vin_cv / 100.0, ch.vout_cv / 100.0, ch.iout_ca / 100.0, ch.set_cc_ca / 100.0,
    ch.set_cv_cv / 100.0, ch.flags, ch.comm_error_count, ch.age_ms,
    have_analog_status_ ? "true" : "false",
    an.main_battery_cv / 100.0, an.main_battery_valid() ? "true" : "false",
    an.aon_battery_cv / 100.0, an.aon_battery_valid() ? "true" : "false",
    an.board_temp_valid() ? an.board_temp_dc / 10.0 : 0.0, an.board_temp_valid() ? "true" : "false",
    an.vdda_mv, (an.flags & kAnalogFlagVddaCalibrated) ? "true" : "false",
    an.mg996_current_raw, an.flags);
  return buf;
}

int16_t MowerSystem::rad_s_to_permille(double rad_s) const
{
  double rpm = rad_s * 60.0 / kTwoPi;
  double permille = rpm / max_rpm_ * 1000.0;
  permille = std::clamp(permille, -1000.0, 1000.0);
  return static_cast<int16_t>(std::lround(permille));
}

bool MowerSystem::send_stop()
{
  auto f = build_wheel_speed_command(tx_seq_++, 0, 0, static_cast<uint16_t>(command_timeout_ms_));
  return port_.write_all(f.data(), f.size());
}

return_type MowerSystem::write(const rclcpp::Time & time, const rclcpp::Duration &)
{
  int16_t l = 0;
  int16_t r = 0;
  if (!override_permille(time, l, r)) {
    l = rad_s_to_permille(left_.cmd_velocity);
    r = rad_s_to_permille(right_.cmd_velocity);
  }
  auto f = build_wheel_speed_command(tx_seq_++, l, r, static_cast<uint16_t>(command_timeout_ms_));
  if (!port_.write_all(f.data(), f.size())) {
    RCLCPP_ERROR_THROTTLE(logger(), throttle_clock_, 2000, "serial write error: %s",
      port_.last_error().c_str());
    return return_type::ERROR;
  }
  send_led_if_needed(time);
  send_pid_if_needed();
  return return_type::OK;
}

}  // namespace mower_hardware

PLUGINLIB_EXPORT_CLASS(mower_hardware::MowerSystem, hardware_interface::SystemInterface)
