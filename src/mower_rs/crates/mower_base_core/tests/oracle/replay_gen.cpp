// Generates a synthetic serial recording plus the expectations the Rust
// BaseCycle replay test checks against.
//
// Links the original mower_hardware/mower_protocol.cpp and the original
// ros2_controllers diff_drive_controller odometry.cpp + control_toolbox
// RateLimiter. The cycle glue is transcribed from mower_system.cpp
// (read()/write()/handle_frame) and diff_drive_controller.cpp.
//
// Writes:  <outdir>/replay_synthetic.mowerlog   (record::encode format)
//          <outdir>/replay_synthetic.json       (tick schedule + expectations)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <limits>
#include <queue>
#include <array>
#include <string>
#include <vector>

#include "mower_hardware/mower_protocol.hpp"
#include "diff_drive_controller/odometry.hpp"
#include "diff_drive_controller/speed_limiter.hpp"

using namespace mower_hardware;
using diff_drive_controller::Odometry;
using diff_drive_controller::SpeedLimiter;

static const double NaN = std::numeric_limits<double>::quiet_NaN();
static const double kTwoPi = 6.283185307179586;

static std::string num(double v)
{
  if (std::isnan(v)) return "\"nan\"";
  char b[64];
  std::snprintf(b, sizeof(b), "%.17g", v);
  return b;
}
static std::string hex(const std::vector<uint8_t> & v)
{
  static const char * d = "0123456789abcdef";
  std::string s;
  for (uint8_t b : v) { s.push_back(d[b >> 4]); s.push_back(d[b & 0xF]); }
  return s;
}

// ---------------------------------------------------------------- log ----
struct LogRec { int64_t t_ns; uint8_t dir; std::vector<uint8_t> bytes; };
static std::vector<LogRec> g_log;
static void log_push(int64_t t, uint8_t dir, const std::vector<uint8_t> & b)
{
  g_log.push_back({t, dir, b});
}

// --------------------------------------------------------------- misc ----
static uint32_t rng_state = 0x1234abcdu;
static uint32_t rng()
{
  rng_state = rng_state * 1664525u + 1013904223u;
  return rng_state >> 8;
}

// ------------------------------------------------------ mower_system -----
struct Wheel {
  double cmd_velocity = 0.0;
  double pos = 0.0;
  double vel = 0.0;
  int32_t last_total_counts = 0;
  bool have_counts = false;
};

static const double kMaxRpm = 58.0;
static const double kCountsPerRev = 8896.0;
static const uint16_t kCommandTimeoutMs = 300;
static const double kFeedbackTimeoutS = 0.5;
static const double kLedResendPeriodS = 5.0;

static Wheel g_left, g_right;
static uint8_t g_tx_seq = 0;
static bool g_feedback_valid = false;
static int64_t g_last_feedback_ns = 0;
static double g_feedback_age_s = 0.0;
static bool g_shutdown_acked = false;
static std::vector<uint8_t> g_pending_tx;
static FrameParser g_parser;

static int16_t rad_s_to_permille(double rad_s)
{
  double rpm = rad_s * 60.0 / kTwoPi;
  double permille = rpm / kMaxRpm * 1000.0;
  permille = std::clamp(permille, -1000.0, 1000.0);
  return static_cast<int16_t>(std::lround(permille));
}

static void integrate(Wheel & w, int32_t total, double rpm, double rad_per_count)
{
  if (w.have_counts) {
    int32_t d = static_cast<int32_t>(static_cast<uint32_t>(total) - static_cast<uint32_t>(w.last_total_counts));
    w.pos += d * rad_per_count;
  }
  w.last_total_counts = total;
  w.have_counts = true;
  w.vel = rpm * kTwoPi / 60.0;
}

static void handle_frame(uint8_t type, uint8_t seq, const uint8_t * p, size_t len, int64_t now)
{
  if (type == kWheelFeedbackStatus) {
    WheelFeedback fb;
    if (!decode_wheel_feedback(seq, p, len, fb)) return;
    const double rad_per_count = kTwoPi / kCountsPerRev;
    integrate(g_left, fb.left_total_counts, fb.left_measured_rpm, rad_per_count);
    integrate(g_right, fb.right_total_counts, fb.right_measured_rpm, rad_per_count);
    g_last_feedback_ns = now;
    g_feedback_valid = true;
  } else if (type == kPowerStatus) {
    PowerStatus ps;
    if (!decode_power_status(p, len, ps)) return;
    if (!ps.shutdown_requested()) { g_shutdown_acked = false; return; }
    if (g_shutdown_acked) return;
    g_shutdown_acked = true;
    auto f = build_power_command(g_tx_seq++, kPowerActionHostShutdownAck);
    g_pending_tx.insert(g_pending_tx.end(), f.begin(), f.end());
  }
  // the other status types only latch telemetry, which does not reach the wire
}

// ------------------------------------------------------- STM32 model -----
// Firmware side: closed-loop wheels that reach the commanded rpm next frame.
struct Stm32 {
  double left_target_rpm = 0.0, right_target_rpm = 0.0;
  double left_rpm = 0.0, right_rpm = 0.0;
  double left_counts = 0.0, right_counts = 0.0;
  uint8_t seq = 0;
  uint8_t last_rx_seq = 0;
  int64_t last_status_ns = 0;
};
static Stm32 g_mcu;

int main(int argc, char ** argv)
{
  const std::string outdir = argc > 1 ? argv[1] : ".";
  const int64_t t0 = 1'000'000'000LL;  // a monotonic clock, 1 s in
  const int64_t dt_ns = 40'000'000LL;  // 25 Hz
  const int N = 300;                   // 12 s

  // --- controller state (diff_drive_controller) --------------------------
  const double wheel_separation = 0.40, left_radius = 0.10, right_radius = 0.10;
  const double cmd_vel_timeout = 0.5;
  const int64_t publish_period_ns = static_cast<int64_t>(1.0 / 25.0 * 1e9);
  Odometry odom(10);
  odom.setWheelParams(wheel_separation, left_radius, right_radius);
  odom.setVelocityRollingWindowSize(10);
  SpeedLimiter lim_lin(-0.55, 0.55, NaN, 0.8, NaN, NaN, NaN, NaN);
  SpeedLimiter lim_ang(-2.0, 2.0, NaN, 2.0, NaN, NaN, NaN, NaN);
  std::queue<std::array<double, 2>> prev2;
  prev2.push({{0.0, 0.0}});
  prev2.push({{0.0, 0.0}});
  int64_t previous_publish = t0;
  double ref_lin = NaN, ref_ang = NaN, cmd_lin = NaN, cmd_ang = NaN;
  int64_t cmd_stamp = t0;

  // --- side channel state (mower_system) ---------------------------------
  int64_t blade_until = 0; int16_t blade_permille = 0; bool blade_active = false;
  int64_t override_until = 0; int16_t override_left = 0, override_right = 0;
  bool have_servo_req = false; uint16_t servo_pulse = 0, servo_hold = 0;
  bool have_led_req = false;
  uint8_t led_mode = 0, led_r = 0, led_g = 0, led_b = 0, led_serial = 0;
  uint16_t led_period = 0;
  bool led_sent_valid = false;
  uint8_t s_mode = 0, s_r = 0, s_g = 0, s_b = 0, s_serial = 0; uint16_t s_period = 0;
  int64_t led_sent_ns = 0; bool led_sent_ns_valid = false;
  bool have_pid_req = false; PidConfig pid_req;

  // --- on_activate --------------------------------------------------------
  std::vector<uint8_t> act;
  {
    auto f = build_wheel_speed_command(g_tx_seq++, 0, 0, kCommandTimeoutMs);
    auto b = build_lawer_motor_command(g_tx_seq++, 0, kCommandTimeoutMs);
    auto i = build_info_request(g_tx_seq++);
    act.insert(act.end(), f.begin(), f.end());
    act.insert(act.end(), b.begin(), b.end());
    act.insert(act.end(), i.begin(), i.end());
  }
  log_push(t0 - dt_ns, 1, act);

  // --- STM32 frame schedule ----------------------------------------------
  int64_t next_85 = t0 + 25'000'000LL;   // 20 Hz
  int64_t next_81 = t0 + 30'000'000LL;   // 10 Hz
  int64_t next_86 = t0 + 100'000'000LL;  // 2 Hz
  int64_t next_87 = t0 + 200'000'000LL;  // 1 Hz
  int64_t next_89 = t0 + 150'000'000LL;  // 2 Hz
  int64_t next_88 = t0 + 400'000'000LL;  // 1 Hz
  int64_t next_82 = t0 + 450'000'000LL;  // 1 Hz
  const int64_t silence_from = t0 + 6'000'000'000LL;  // 1 s of nothing: feedback timeout
  const int64_t silence_to = t0 + 7'000'000'000LL;
  bool injected_garbage = false, injected_crc_error = false;

  std::string ticks_json;

  for (int k = 0; k < N; ++k) {
    const int64_t t = t0 + static_cast<int64_t>(k) * dt_ns;
    const double period = static_cast<double>(dt_ns) / 1e9;

    // ---- STM32 -> host bytes for this cycle ------------------------------
    std::vector<uint8_t> rx;
    auto emit = [&](const std::vector<uint8_t> & f) { rx.insert(rx.end(), f.begin(), f.end()); };
    while (next_85 <= t) {
      if (next_85 < silence_from || next_85 >= silence_to) {
        // advance the firmware plant to the moment of the frame
        g_mcu.left_rpm = g_mcu.left_target_rpm;
        g_mcu.right_rpm = g_mcu.right_target_rpm;
        g_mcu.left_counts += g_mcu.left_rpm / 60.0 * 0.05 * kCountsPerRev;
        g_mcu.right_counts += g_mcu.right_rpm / 60.0 * 0.05 * kCountsPerRev;
        uint8_t p[24] = {};
        auto put_i16 = [](uint8_t * q, int v) {
          uint16_t u = static_cast<uint16_t>(static_cast<int16_t>(v));
          q[0] = static_cast<uint8_t>(u & 0xFF); q[1] = static_cast<uint8_t>(u >> 8);
        };
        auto put_i32 = [](uint8_t * q, int64_t v) {
          uint32_t u = static_cast<uint32_t>(static_cast<int32_t>(v));
          q[0] = static_cast<uint8_t>(u); q[1] = static_cast<uint8_t>(u >> 8);
          q[2] = static_cast<uint8_t>(u >> 16); q[3] = static_cast<uint8_t>(u >> 24);
        };
        put_i16(p + 0, static_cast<int>(std::lround(g_mcu.left_target_rpm * 100.0)));
        put_i16(p + 2, static_cast<int>(std::lround(g_mcu.left_rpm * 100.0)));
        put_i16(p + 4, static_cast<int>(std::lround(g_mcu.right_target_rpm * 100.0)));
        put_i16(p + 6, static_cast<int>(std::lround(g_mcu.right_rpm * 100.0)));
        put_i16(p + 8, 120); put_i16(p + 10, -95);
        put_i32(p + 12, static_cast<int64_t>(g_mcu.left_counts));
        put_i32(p + 16, static_cast<int64_t>(g_mcu.right_counts));
        p[20] = kWheelFlagOutputEnabled | kWheelFlagClosedLoop;
        auto f = build_frame(kWheelFeedbackStatus, g_mcu.seq++, p, sizeof(p));
        if (!injected_crc_error && next_85 >= t0 + 4'000'000'000LL) {
          injected_crc_error = true;
          auto bad = f;
          bad[bad.size() - 1] ^= 0xFF;  // corrupt the CRC: must be counted and skipped
          emit(bad);
        }
        emit(f);
      }
      next_85 += 50'000'000LL;
    }
    while (next_81 <= t) {
      if (next_81 < silence_from || next_81 >= silence_to) {
        uint8_t p[12] = {};
        p[8] = 20; p[10] = kStatusFlagCommandValid; p[11] = g_mcu.last_rx_seq;
        emit(build_frame(kMotorStatus, g_mcu.seq++, p, sizeof(p)));
      }
      next_81 += 100'000'000LL;
    }
    while (next_86 <= t) {
      uint8_t p[8] = {};
      p[0] = kPowerStateRunning;
      p[1] = kPowerFlagMainPowerEnabled;
      // 3 s button press near the end: the driver must ack exactly once
      if (next_86 >= t0 + 11'000'000'000LL) {
        p[1] |= kPowerFlagShutdownRequested | kPowerFlagButtonPressed;
        p[2] = 1;
      }
      p[3] = g_mcu.last_rx_seq;
      emit(build_frame(kPowerStatus, g_mcu.seq++, p, sizeof(p)));
      next_86 += 500'000'000LL;
    }
    while (next_87 <= t) {
      uint8_t p[16] = {0, 6, 0, 1, 0x78, 0x56, 0x34, 0x12, 0xDD, 0xCC, 0xBB, 0xAA, 0x00, 0, 0, 0};
      emit(build_frame(kFirmwareInfo, g_mcu.seq++, p, sizeof(p)));
      next_87 += 1'000'000'000LL;
    }
    while (next_89 <= t) {
      uint8_t p[16] = {};
      p[0] = 0x10; p[1] = 0x0A; p[10] = kChargerFlagOnline | kChargerFlagEverSeen;
      emit(build_frame(kChargerStatus, g_mcu.seq++, p, sizeof(p)));
      next_89 += 500'000'000LL;
    }
    while (next_88 <= t) {
      uint8_t p[8] = {0xDC, 0x05, 0, 0, 0, 0, kServoFlagEnabled, g_mcu.last_rx_seq};
      emit(build_frame(kServoStatus, g_mcu.seq++, p, sizeof(p)));
      next_88 += 1'000'000'000LL;
    }
    while (next_82 <= t) {
      uint8_t p[8] = {};
      p[6] = kStatusFlagCommandValid; p[7] = g_mcu.last_rx_seq;
      emit(build_frame(kLawerMotorStatus, g_mcu.seq++, p, sizeof(p)));
      next_82 += 1'000'000'000LL;
    }
    if (!injected_garbage && t >= t0 + 2'000'000'000LL) {
      injected_garbage = true;
      // line noise between frames, including a lone SOF0 and a false SOF pair
      const uint8_t junk[] = {0x00, 0xFF, 0xA5, 0x13, 0xA5, 0x5A, 0x77};
      rx.insert(rx.begin(), std::begin(junk), std::end(junk));
    }

    // split into 1-3 reads at arbitrary boundaries, as VMIN=0 reads do
    std::vector<std::pair<int64_t, std::vector<uint8_t>>> chunks;
    if (!rx.empty()) {
      size_t nchunks = 1 + (rng() % 3);
      size_t off = 0;
      for (size_t c = 0; c < nchunks && off < rx.size(); ++c) {
        size_t remaining = rx.size() - off;
        size_t take = (c + 1 == nchunks) ? remaining : 1 + (rng() % remaining);
        chunks.emplace_back(t - 2'000'000LL + static_cast<int64_t>(c) * 200'000LL,
          std::vector<uint8_t>(rx.begin() + off, rx.begin() + off + take));
        off += take;
      }
    }
    for (auto & c : chunks) {
      log_push(c.first, 0, c.second);
      g_parser.feed(c.second.data(), c.second.size(),
        [&](uint8_t ty, uint8_t sq, const uint8_t * p, size_t l) {
          handle_frame(ty, sq, p, l, c.first);
        });
    }

    // ---- read() tail: stale feedback zeroes the reported velocities ------
    if (g_feedback_valid) {
      g_feedback_age_s = static_cast<double>(t) / 1e9 - static_cast<double>(g_last_feedback_ns) / 1e9;
      if (g_feedback_age_s > kFeedbackTimeoutS) {
        g_left.vel = g_right.vel = 0.0;
      }
    }

    // ---- side channel requests for this cycle ----------------------------
    std::string requests = "[";
    auto add_req = [&](const std::string & s) {
      if (requests.size() > 1) requests += ", ";
      requests += s;
    };
    if (k == 60) {  // blade on, dead-man held for 500 ms
      blade_permille = 300;
      blade_until = t + 500'000'000LL;
      add_req("{\"kind\": \"blade\", \"permille\": 300, \"ttl_ms\": 500}");
    }
    if (k == 75) {  // light request
      have_led_req = true; led_mode = 6; led_r = 255; led_g = 180; led_b = 0;
      led_period = 1600; led_serial = 1;
      add_req("{\"kind\": \"led\", \"mode\": 6, \"r\": 255, \"g\": 180, \"b\": 0, "
              "\"period_ms\": 1600, \"serial\": 1}");
    }
    if (k == 90) {  // lift servo
      have_servo_req = true; servo_pulse = 1500; servo_hold = 0;
      add_req("{\"kind\": \"servo\", \"pulse_us\": 1500, \"hold_ms\": 0}");
    }
    if (k == 150) {  // auto-tune step: raw permille, bypassing the limiter
      override_left = 400; override_right = 400;
      override_until = t + 300'000'000LL;
      add_req("{\"kind\": \"override\", \"left\": 400, \"right\": 400, \"ttl_ms\": 300}");
    }
    if (k == 200) {
      have_pid_req = true;
      pid_req.left_kp = 2.0f; pid_req.left_ki = 0.6f; pid_req.left_kd = 0.0f;
      pid_req.right_kp = 2.0f; pid_req.right_ki = 0.6f; pid_req.right_kd = 0.0f;
      pid_req.persist_to_flash = false; pid_req.closed_loop_enabled = true;
      add_req("{\"kind\": \"pid\", \"left\": [2, 0.6, 0], \"right\": [2, 0.6, 0], "
              "\"persist\": false, \"closed_loop\": true}");
    }
    requests += "]";

    // ---- cmd_vel ----------------------------------------------------------
    bool have_cmd = true;
    double in_lin = 0.0, in_ang = 0.0;
    if (k < 50) { in_lin = 0.35; in_ang = 0.0; }
    else if (k < 100) { in_lin = 0.35; in_ang = 0.7; }
    else if (k < 130) { have_cmd = false; }          // silence -> cmd_vel timeout
    else if (k < 180) { in_lin = -0.25; in_ang = -1.2; }
    else if (k < 230) { in_lin = 0.6; in_ang = 3.0; }  // past the velocity limits
    else { in_lin = 0.0; in_ang = 0.0; }
    if (have_cmd) { cmd_lin = in_lin; cmd_ang = in_ang; cmd_stamp = t; }

    // ---- update_reference_from_subscribers ------------------------------
    const double age = static_cast<double>(t) / 1e9 - static_cast<double>(cmd_stamp) / 1e9;
    if (cmd_vel_timeout != 0.0 && age > cmd_vel_timeout) {
      ref_lin = 0.0; ref_ang = 0.0;
    } else if (std::isfinite(cmd_lin) && std::isfinite(cmd_ang)) {
      ref_lin = cmd_lin; ref_ang = cmd_ang;
    }

    // ---- update_and_write_commands ---------------------------------------
    double linear_command = ref_lin, angular_command = ref_ang;
    bool published = false;
    if (std::isfinite(linear_command) && std::isfinite(angular_command)) {
      double & last_linear = prev2.back()[0];
      double & second_to_last_linear = prev2.front()[0];
      double & last_angular = prev2.back()[1];
      double & second_to_last_angular = prev2.front()[1];
      lim_lin.limit(linear_command, last_linear, second_to_last_linear, period);
      lim_ang.limit(angular_command, last_angular, second_to_last_angular, period);
      prev2.pop();
      prev2.push({{linear_command, angular_command}});

      odom.update(g_left.pos, g_right.pos, rclcpp::Time(t));

      if (previous_publish + publish_period_ns < t) {
        previous_publish += publish_period_ns;
        published = true;
      }
      g_left.cmd_velocity = (linear_command - angular_command * wheel_separation / 2.0) / left_radius;
      g_right.cmd_velocity = (linear_command + angular_command * wheel_separation / 2.0) / right_radius;
    }

    // ---- write() ----------------------------------------------------------
    std::vector<uint8_t> tx;
    tx.swap(g_pending_tx);
    int16_t l, r;
    const bool ovr = override_until != 0 && t < override_until;
    if (ovr) { l = override_left; r = override_right; }
    else { l = rad_s_to_permille(g_left.cmd_velocity); r = rad_s_to_permille(g_right.cmd_velocity); }
    {
      auto f = build_wheel_speed_command(g_tx_seq++, l, r, kCommandTimeoutMs);
      tx.insert(tx.end(), f.begin(), f.end());
      g_mcu.last_rx_seq = static_cast<uint8_t>(g_tx_seq - 1);
      g_mcu.left_target_rpm = l / 1000.0 * kMaxRpm;
      g_mcu.right_target_rpm = r / 1000.0 * kMaxRpm;
    }
    {
      const bool active = blade_until != 0 && t < blade_until;
      const bool was_active = blade_active;
      blade_active = active;
      if (active || was_active) {
        auto f = build_lawer_motor_command(g_tx_seq++, active ? blade_permille : 0, kCommandTimeoutMs);
        tx.insert(tx.end(), f.begin(), f.end());
      }
    }
    if (have_servo_req) {
      have_servo_req = false;
      auto f = build_servo_command(g_tx_seq++, servo_pulse, servo_hold);
      tx.insert(tx.end(), f.begin(), f.end());
    }
    if (have_led_req) {
      const bool changed = !led_sent_valid || led_mode != s_mode || led_r != s_r ||
                           led_g != s_g || led_b != s_b || led_period != s_period ||
                           led_serial != s_serial;
      const bool stale = !led_sent_ns_valid ||
        (static_cast<double>(t) / 1e9 - static_cast<double>(led_sent_ns) / 1e9) >= kLedResendPeriodS;
      if (changed || stale) {
        auto f = build_ws2812_command(g_tx_seq++, led_mode, led_r, led_g, led_b, led_period);
        tx.insert(tx.end(), f.begin(), f.end());
        led_sent_valid = true;
        s_mode = led_mode; s_r = led_r; s_g = led_g; s_b = led_b;
        s_period = led_period; s_serial = led_serial;
        led_sent_ns = t; led_sent_ns_valid = true;
      }
    }
    if (have_pid_req) {
      have_pid_req = false;
      auto f = build_pid_config_command(g_tx_seq++, pid_req);
      tx.insert(tx.end(), f.begin(), f.end());
    }
    log_push(t, 1, tx);

    // ---- expectation row --------------------------------------------------
    std::string odom_json = "null";
    if (published) {
      const double heading = odom.getHeading();
      odom_json = "{\"x\": " + num(odom.getX()) + ", \"y\": " + num(odom.getY()) +
        ", \"yaw\": " + num(heading) + ", \"qz\": " + num(std::sin(heading * 0.5)) +
        ", \"qw\": " + num(std::cos(heading * 0.5)) + ", \"linear\": " + num(odom.getLinear()) +
        ", \"angular\": " + num(odom.getAngular()) + "}";
    }
    ticks_json += std::string(k ? ",\n" : "") +
      "    {\"t_ns\": " + std::to_string(t) +
      ", \"cmd\": " + (have_cmd ? "[" + num(in_lin) + ", " + num(in_ang) + "]" : "null") +
      ", \"requests\": " + requests +
      ", \"odom\": " + odom_json +
      ", \"joints\": {\"pos\": [" + num(g_left.pos) + ", " + num(g_right.pos) +
      "], \"vel\": [" + num(g_left.vel) + ", " + num(g_right.vel) + "]}" +
      ", \"feedback_age_s\": " + num(g_feedback_age_s) +
      ", \"crc_errors\": " + std::to_string(g_parser.crc_errors()) + "}";
  }

  // ---- write the log ------------------------------------------------------
  {
    std::vector<uint8_t> blob(8);
    std::memcpy(blob.data(), "MWRSER01", 8);
    for (const auto & rec : g_log) {
      uint64_t t = static_cast<uint64_t>(rec.t_ns);
      for (int i = 0; i < 8; ++i) blob.push_back(static_cast<uint8_t>(t >> (8 * i)));
      blob.push_back(rec.dir);
      uint32_t len = static_cast<uint32_t>(rec.bytes.size());
      for (int i = 0; i < 4; ++i) blob.push_back(static_cast<uint8_t>(len >> (8 * i)));
      blob.insert(blob.end(), rec.bytes.begin(), rec.bytes.end());
    }
    std::string path = outdir + "/replay_synthetic.mowerlog";
    FILE * fp = std::fopen(path.c_str(), "wb");
    std::fwrite(blob.data(), 1, blob.size(), fp);
    std::fclose(fp);
    std::fprintf(stderr, "wrote %s (%zu bytes, %zu records)\n", path.c_str(), blob.size(), g_log.size());
  }

  {
    std::string j = "{\n  \"t0_ns\": " + std::to_string(t0) +
      ",\n  \"tick_period_ns\": " + std::to_string(dt_ns) +
      ",\n  \"activation_tx\": \"" + hex(act) + "\"" +
      ",\n  \"crc_errors_total\": " + std::to_string(g_parser.crc_errors()) +
      ",\n  \"ticks\": [\n" + ticks_json + "\n  ]\n}\n";
    std::string path = outdir + "/replay_synthetic.json";
    FILE * fp = std::fopen(path.c_str(), "wb");
    std::fwrite(j.data(), 1, j.size(), fp);
    std::fclose(fp);
    std::fprintf(stderr, "wrote %s (%zu bytes)\n", path.c_str(), j.size());
  }
  return 0;
}
