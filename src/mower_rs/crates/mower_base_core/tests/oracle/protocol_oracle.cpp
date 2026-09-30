// Oracle harness: links the original mower_hardware/mower_protocol.cpp and
// emits encode/decode test vectors as JSON for the Rust port to match.
// Self-contained: mower_protocol.cpp has no ROS dependency.
#include "mower_hardware/mower_protocol.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace mower_hardware;

static std::string hex(const std::vector<uint8_t> & v)
{
  static const char * d = "0123456789abcdef";
  std::string s;
  s.reserve(v.size() * 2);
  for (uint8_t b : v) { s.push_back(d[b >> 4]); s.push_back(d[b & 0xF]); }
  return s;
}
static std::string hex(const uint8_t * p, size_t n) { return hex(std::vector<uint8_t>(p, p + n)); }

static std::vector<uint8_t> unhex(const std::string & s)
{
  auto nib = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return c - 'A' + 10;
  };
  std::vector<uint8_t> v;
  for (size_t i = 0; i + 1 < s.size(); i += 2) v.push_back(static_cast<uint8_t>(nib(s[i]) * 16 + nib(s[i + 1])));
  return v;
}

// double -> JSON with full round-trip precision
static std::string num(double v)
{
  char b[64];
  std::snprintf(b, sizeof(b), "%.17g", v);
  return b;
}

int main()
{
  std::string out;
  out += "{\n";

  // ---- crc16_ccitt_false ----------------------------------------------
  const char * crc_inputs[] = {
    "", "00", "ff", "0102030405", "a55a010185", "deadbeef", "010203040506070809",
    "00000000000000000000000000000000",
    "ffffffffffffffffffffffffffffffff",
  };
  out += "  \"crc16\": [\n";
  for (size_t i = 0; i < sizeof(crc_inputs) / sizeof(*crc_inputs); ++i) {
    auto d = unhex(crc_inputs[i]);
    char b[128];
    std::snprintf(b, sizeof(b), "    {\"data\": \"%s\", \"crc\": %u}%s\n", crc_inputs[i],
      crc16_ccitt_false(d.data(), d.size()), i + 1 == sizeof(crc_inputs) / sizeof(*crc_inputs) ? "" : ",");
    out += b;
  }
  out += "  ],\n";

  // ---- encoders --------------------------------------------------------
  out += "  \"encode\": [\n";
  std::vector<std::string> enc;
  auto add = [&](const std::string & name, const std::string & args, const std::vector<uint8_t> & f) {
    enc.push_back("    {\"fn\": \"" + name + "\", \"args\": " + args + ", \"bytes\": \"" + hex(f) + "\"}");
  };

  struct WheelCase { int seq; int l; int r; int t; };
  const WheelCase wheels[] = {
    {0, 0, 0, 300}, {1, 1000, -1000, 300}, {7, -1000, 1000, 300}, {255, 32767, -32768, 65535},
    {42, 123, -456, 500}, {200, -1, 1, 0}, {13, 500, 500, 300}, {14, -500, -500, 300},
  };
  for (const auto & w : wheels) {
    char a[128];
    std::snprintf(a, sizeof(a), "[%d, %d, %d, %d]", w.seq, w.l, w.r, w.t);
    add("wheel_speed_command", a,
      build_wheel_speed_command(static_cast<uint8_t>(w.seq), static_cast<int16_t>(w.l),
        static_cast<int16_t>(w.r), static_cast<uint16_t>(w.t)));
  }
  const int blades[][3] = {{0, 0, 300}, {3, 300, 500}, {9, -300, 300}, {250, 1000, 65535}, {5, 32767, 1}};
  for (const auto & b : blades) {
    char a[128];
    std::snprintf(a, sizeof(a), "[%d, %d, %d]", b[0], b[1], b[2]);
    add("lawer_motor_command", a,
      build_lawer_motor_command(static_cast<uint8_t>(b[0]), static_cast<int16_t>(b[1]), static_cast<uint16_t>(b[2])));
  }
  const int servos[][3] = {{0, 1500, 0}, {2, 500, 1000}, {4, 2500, 65535}, {6, 0, 0}};
  for (const auto & s : servos) {
    char a[128];
    std::snprintf(a, sizeof(a), "[%d, %d, %d]", s[0], s[1], s[2]);
    add("servo_command", a,
      build_servo_command(static_cast<uint8_t>(s[0]), static_cast<uint16_t>(s[1]), static_cast<uint16_t>(s[2])));
  }
  for (int action = 0; action <= 4; ++action) {
    char a[64];
    std::snprintf(a, sizeof(a), "[%d, %d]", action + 10, action);
    add("power_command", a, build_power_command(static_cast<uint8_t>(action + 10), static_cast<uint8_t>(action)));
  }
  for (int seq : {0, 1, 128, 255}) {
    char a[32];
    std::snprintf(a, sizeof(a), "[%d]", seq);
    add("info_request", a, build_info_request(static_cast<uint8_t>(seq)));
  }
  const int leds[][6] = {
    {0, 0, 0, 0, 0, 0}, {1, 1, 255, 180, 0, 1600}, {2, 6, 10, 20, 30, 500}, {3, 5, 255, 255, 255, 65535},
  };
  for (const auto & l : leds) {
    char a[128];
    std::snprintf(a, sizeof(a), "[%d, %d, %d, %d, %d, %d]", l[0], l[1], l[2], l[3], l[4], l[5]);
    add("ws2812_command", a,
      build_ws2812_command(static_cast<uint8_t>(l[0]), static_cast<uint8_t>(l[1]), static_cast<uint8_t>(l[2]),
        static_cast<uint8_t>(l[3]), static_cast<uint8_t>(l[4]), static_cast<uint16_t>(l[5])));
  }
  {
    struct PidCase { int seq; float g[6]; bool persist; bool closed; };
    const PidCase pids[] = {
      {0, {0, 0, 0, 0, 0, 0}, false, true},
      {1, {2.0f, 0.6f, 0.0f, 2.0f, 0.6f, 0.0f}, false, true},
      {2, {1.25f, 0.125f, 0.03125f, 3.5f, 0.75f, 0.0f}, true, false},
      {3, {-1.5f, 1e-6f, 1e6f, 0.1f, 0.2f, 0.3f}, true, true},
    };
    for (const auto & p : pids) {
      PidConfig c;
      c.left_kp = p.g[0]; c.left_ki = p.g[1]; c.left_kd = p.g[2];
      c.right_kp = p.g[3]; c.right_ki = p.g[4]; c.right_kd = p.g[5];
      c.persist_to_flash = p.persist; c.closed_loop_enabled = p.closed;
      char a[256];
      std::snprintf(a, sizeof(a), "[%d, [%s, %s, %s, %s, %s, %s], %s, %s]", p.seq,
        num(p.g[0]).c_str(), num(p.g[1]).c_str(), num(p.g[2]).c_str(), num(p.g[3]).c_str(),
        num(p.g[4]).c_str(), num(p.g[5]).c_str(), p.persist ? "true" : "false",
        p.closed ? "true" : "false");
      add("pid_config_command", a, build_pid_config_command(static_cast<uint8_t>(p.seq), c));
    }
  }
  {
    // raw build_frame with assorted payload lengths, including 0 and 255
    const char * payloads[] = {"", "00", "0102", "deadbeefcafe"};
    for (size_t i = 0; i < 4; ++i) {
      auto p = unhex(payloads[i]);
      char a[128];
      std::snprintf(a, sizeof(a), "[%zu, %zu, \"%s\"]", i + 0x20, i + 3, payloads[i]);
      add("frame", a, build_frame(static_cast<uint8_t>(i + 0x20), static_cast<uint8_t>(i + 3),
        p.empty() ? nullptr : p.data(), p.size()));
    }
    std::vector<uint8_t> big(255);
    for (size_t i = 0; i < big.size(); ++i) big[i] = static_cast<uint8_t>(i * 7 + 3);
    add("frame", "[153, 77, \"" + hex(big) + "\"]", build_frame(0x99, 77, big.data(), big.size()));
  }
  for (size_t i = 0; i < enc.size(); ++i) out += enc[i] + (i + 1 == enc.size() ? "\n" : ",\n");
  out += "  ],\n";

  // ---- FrameParser byte-stream vectors --------------------------------
  // Each case is a list of chunks (as the read() loop would hand them over);
  // the parser must be fed chunk by chunk.
  struct StreamCase { const char * name; std::vector<std::string> chunks; };
  auto f85 = hex(build_frame(kWheelFeedbackStatus, 3, [] {
      static uint8_t p[24];
      for (int i = 0; i < 24; ++i) p[i] = static_cast<uint8_t>(i);
      return p; }(), 24));
  auto f81 = hex(build_frame(kMotorStatus, 4, [] {
      static uint8_t p[12];
      for (int i = 0; i < 12; ++i) p[i] = static_cast<uint8_t>(0x10 + i);
      return p; }(), 12));
  auto f87 = hex(build_frame(kFirmwareInfo, 5, [] {
      static uint8_t p[16] = {0, 6, 0, 1, 0x12, 0x34, 0x56, 0x78, 0xaa, 0xbb, 0xcc, 0xdd, 0x01, 0, 0, 0};
      return p; }(), 16));
  // a frame with a deliberately broken CRC
  std::string bad = f85;
  bad[bad.size() - 1] = (bad[bad.size() - 1] == '0') ? '1' : '0';
  // a frame with a bad protocol version byte (index 2 -> chars 4,5)
  std::string badver = f81;
  badver[4] = '0'; badver[5] = '9';

  std::vector<StreamCase> streams = {
    {"single_frame", {f85}},
    {"three_frames_one_chunk", {f85 + f81 + f87}},
    {"byte_at_a_time", {}},          // filled below
    {"leading_garbage", {"00112233445566" + f85}},
    {"garbage_between", {f85 + "deadbeef" + f81}},
    {"split_mid_header", {f85.substr(0, 6), f85.substr(6)}},
    {"split_mid_payload", {f85.substr(0, 30), f85.substr(30) + f81.substr(0, 10), f81.substr(10)}},
    {"split_between_sof_bytes", {"a5", "5a" + f85.substr(4)}},
    {"trailing_sof0_only", {f85 + "a5"}},
    {"trailing_sof0_then_frame", {f85 + "a5", f81.substr(2)}},
    {"crc_error_then_good", {bad + f81}},
    {"bad_version_then_good", {badver + f85}},
    {"false_sof_inside_garbage", {"a55a" + f85}},
    {"truncated_never_completes", {f85.substr(0, 20)}},
    {"length_255_incomplete", {"a55a01990401"}},
    {"empty_chunk_then_frame", {"", f81}},
    {"two_crc_errors", {bad + bad + f87}},
  };
  for (size_t i = 0; i < f85.size(); i += 2) streams[2].chunks.push_back(f85.substr(i, 2));

  out += "  \"streams\": [\n";
  for (size_t si = 0; si < streams.size(); ++si) {
    FrameParser parser;
    std::string frames;
    size_t nframes = 0;
    std::string chunks_json;
    for (size_t ci = 0; ci < streams[si].chunks.size(); ++ci) {
      const std::string & c = streams[si].chunks[ci];
      chunks_json += "\"" + c + "\"";
      if (ci + 1 != streams[si].chunks.size()) chunks_json += ", ";
      auto bytes = unhex(c);
      parser.feed(bytes.empty() ? nullptr : bytes.data(), bytes.size(),
        [&](uint8_t t, uint8_t s, const uint8_t * p, size_t l) {
          if (nframes) frames += ", ";
          char b[64];
          std::snprintf(b, sizeof(b), "{\"type\": %u, \"seq\": %u, \"payload\": \"", t, s);
          frames += b;
          frames += hex(p, l);
          frames += "\"}";
          ++nframes;
        });
    }
    out += std::string("    {\"name\": \"") + streams[si].name + "\", \"chunks\": [" + chunks_json +
      "], \"frames\": [" + frames + "], \"crc_errors\": " + std::to_string(parser.crc_errors()) +
      "}" + (si + 1 == streams.size() ? "" : ",") + "\n";
  }
  out += "  ],\n";

  // ---- decoders --------------------------------------------------------
  out += "  \"decode\": [\n";
  std::vector<std::string> dec;
  auto mk = [](std::initializer_list<int> bytes) {
    std::vector<uint8_t> v;
    for (int b : bytes) v.push_back(static_cast<uint8_t>(b));
    return v;
  };

  // wheel feedback (0x85, 24 bytes)
  {
    std::vector<std::vector<uint8_t>> cases = {
      mk({0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0,0,0, 0,0,0,0, 0, 0,0,0}),
      mk({0xE8,0x03, 0xD0,0x07, 0x18,0xFC, 0x30,0xF8, 0x64,0x00, 0x9C,0xFF,
          0x10,0x27,0x00,0x00, 0xF0,0xD8,0xFF,0xFF, 0x03, 0,0,0}),
      mk({0xFF,0x7F, 0x00,0x80, 0x01,0x00, 0xFF,0xFF, 0xFF,0x7F, 0x00,0x80,
          0xFF,0xFF,0xFF,0x7F, 0x00,0x00,0x00,0x80, 0x01, 0,0,0}),
    };
    for (size_t i = 0; i < cases.size(); ++i) {
      WheelFeedback o;
      bool ok = decode_wheel_feedback(static_cast<uint8_t>(i + 1), cases[i].data(), cases[i].size(), o);
      char b[512];
      std::snprintf(b, sizeof(b),
        "    {\"fn\": \"wheel_feedback\", \"seq\": %zu, \"payload\": \"%s\", \"ok\": %s, \"fields\": "
        "{\"left_target_rpm\": %s, \"left_measured_rpm\": %s, \"right_target_rpm\": %s, "
        "\"right_measured_rpm\": %s, \"left_pid_output\": %d, \"right_pid_output\": %d, "
        "\"left_total_counts\": %d, \"right_total_counts\": %d, \"flags\": %u, \"seq\": %u}}",
        i + 1, hex(cases[i]).c_str(), ok ? "true" : "false", num(o.left_target_rpm).c_str(),
        num(o.left_measured_rpm).c_str(), num(o.right_target_rpm).c_str(),
        num(o.right_measured_rpm).c_str(), o.left_pid_output, o.right_pid_output,
        o.left_total_counts, o.right_total_counts, o.flags, o.seq);
      dec.push_back(b);
    }
    // wrong length -> false
    auto wrong = mk({1,2,3});
    WheelFeedback o;
    bool ok = decode_wheel_feedback(9, wrong.data(), wrong.size(), o);
    dec.push_back(std::string("    {\"fn\": \"wheel_feedback\", \"seq\": 9, \"payload\": \"") + hex(wrong) +
      "\", \"ok\": " + (ok ? "true" : "false") + ", \"fields\": null}");
  }
  // motor status (0x81, 12)
  {
    std::vector<std::vector<uint8_t>> cases = {
      mk({0x90,0x01, 0x70,0xFE, 0x20,0x03, 0xE0,0xFC, 0x2C,0x01, 0x03, 0x2A}),
      mk({0,0,0,0,0,0,0,0,0,0,0,0}),
    };
    for (auto & c : cases) {
      MotorStatus o;
      bool ok = decode_motor_status(c.data(), c.size(), o);
      char b[512];
      std::snprintf(b, sizeof(b),
        "    {\"fn\": \"motor_status\", \"payload\": \"%s\", \"ok\": %s, \"fields\": "
        "{\"commanded_left_permille\": %d, \"commanded_right_permille\": %d, \"applied_left_pwm\": %d, "
        "\"applied_right_pwm\": %d, \"command_age_ms\": %u, \"flags\": %u, \"last_rx_seq\": %u}}",
        hex(c).c_str(), ok ? "true" : "false", o.commanded_left_permille, o.commanded_right_permille,
        o.applied_left_pwm, o.applied_right_pwm, o.command_age_ms, o.flags, o.last_rx_seq);
      dec.push_back(b);
    }
    auto wrong = mk({1,2,3,4});
    MotorStatus o;
    bool ok = decode_motor_status(wrong.data(), wrong.size(), o);
    dec.push_back(std::string("    {\"fn\": \"motor_status\", \"payload\": \"") + hex(wrong) +
      "\", \"ok\": " + (ok ? "true" : "false") + ", \"fields\": null}");
  }
  // power status (0x86, 8)
  {
    std::vector<std::vector<uint8_t>> cases = {
      mk({3, 0x06, 1, 0x2A, 0xB8,0x0B, 0xD0,0x07}),
      mk({0, 0x02, 0, 0, 0,0, 0,0}),
    };
    for (auto & c : cases) {
      PowerStatus o;
      bool ok = decode_power_status(c.data(), c.size(), o);
      char b[512];
      std::snprintf(b, sizeof(b),
        "    {\"fn\": \"power_status\", \"payload\": \"%s\", \"ok\": %s, \"fields\": "
        "{\"state\": %u, \"flags\": %u, \"shutdown_reason\": %u, \"last_rx_seq\": %u, "
        "\"press_ms\": %u, \"shutdown_elapsed_ms\": %u, \"shutdown_requested\": %s}}",
        hex(c).c_str(), ok ? "true" : "false", o.state, o.flags, o.shutdown_reason, o.last_rx_seq,
        o.press_ms, o.shutdown_elapsed_ms, o.shutdown_requested() ? "true" : "false");
      dec.push_back(b);
    }
    auto wrong = mk({1,2,3,4,5,6,7});
    PowerStatus o;
    bool ok = decode_power_status(wrong.data(), wrong.size(), o);
    dec.push_back(std::string("    {\"fn\": \"power_status\", \"payload\": \"") + hex(wrong) +
      "\", \"ok\": " + (ok ? "true" : "false") + ", \"fields\": null}");
  }
  // ws2812 status (0x83, 8)
  {
    std::vector<std::vector<uint8_t>> cases = {
      mk({6, 255, 180, 0, 0x40,0x06, 0x01, 0x11}),
      mk({0,0,0,0,0,0,0,0}),
    };
    for (auto & c : cases) {
      Ws2812Status o;
      bool ok = decode_ws2812_status(c.data(), c.size(), o);
      char b[512];
      std::snprintf(b, sizeof(b),
        "    {\"fn\": \"ws2812_status\", \"payload\": \"%s\", \"ok\": %s, \"fields\": "
        "{\"mode\": %u, \"r\": %u, \"g\": %u, \"b\": %u, \"effect_period_ms\": %u, \"flags\": %u, "
        "\"last_rx_seq\": %u}}",
        hex(c).c_str(), ok ? "true" : "false", o.mode, o.r, o.g, o.b, o.effect_period_ms, o.flags,
        o.last_rx_seq);
      dec.push_back(b);
    }
    auto wrong = mk({1});
    Ws2812Status o;
    bool ok = decode_ws2812_status(wrong.data(), wrong.size(), o);
    dec.push_back(std::string("    {\"fn\": \"ws2812_status\", \"payload\": \"") + hex(wrong) +
      "\", \"ok\": " + (ok ? "true" : "false") + ", \"fields\": null}");
  }
  // pid config status (0x84, 28)
  {
    std::vector<std::vector<uint8_t>> cases;
    {
      std::vector<uint8_t> c(28, 0);
      const float g[6] = {2.0f, 0.6f, 0.0f, 2.5f, 0.125f, -1.0f};
      std::memcpy(c.data(), g, sizeof(g));
      c[24] = 0x0F; c[25] = 0x2A; c[26] = 0x34; c[27] = 0x12;
      cases.push_back(c);
    }
    cases.push_back(std::vector<uint8_t>(28, 0));
    for (auto & c : cases) {
      PidConfigStatus o;
      bool ok = decode_pid_config_status(c.data(), c.size(), o);
      char b[768];
      std::snprintf(b, sizeof(b),
        "    {\"fn\": \"pid_config_status\", \"payload\": \"%s\", \"ok\": %s, \"fields\": "
        "{\"left_kp\": %s, \"left_ki\": %s, \"left_kd\": %s, \"right_kp\": %s, \"right_ki\": %s, "
        "\"right_kd\": %s, \"flags\": %u, \"last_rx_seq\": %u, \"flash_diag\": %u}}",
        hex(c).c_str(), ok ? "true" : "false", num(o.left_kp).c_str(), num(o.left_ki).c_str(),
        num(o.left_kd).c_str(), num(o.right_kp).c_str(), num(o.right_ki).c_str(),
        num(o.right_kd).c_str(), o.flags, o.last_rx_seq, o.flash_diag);
      dec.push_back(b);
    }
    auto wrong = std::vector<uint8_t>(27, 0);
    PidConfigStatus o;
    bool ok = decode_pid_config_status(wrong.data(), wrong.size(), o);
    dec.push_back(std::string("    {\"fn\": \"pid_config_status\", \"payload\": \"") + hex(wrong) +
      "\", \"ok\": " + (ok ? "true" : "false") + ", \"fields\": null}");
  }
  // firmware info (0x87, 16)
  {
    std::vector<std::vector<uint8_t>> cases = {
      mk({0, 6, 0, 1, 0x78,0x56,0x34,0x12, 0xDD,0xCC,0xBB,0xAA, 0x01, 0,0,0}),
      mk({1, 2, 3, 1, 0,0,0,0, 0,0,0,0, 0x02, 0,0,0}),
      mk({0, 0, 0, 0, 0,0,0,0, 0,0,0,0, 0x03, 0,0,0}),
    };
    for (auto & c : cases) {
      FirmwareInfo o;
      bool ok = decode_firmware_info(c.data(), c.size(), o);
      char b[1024];
      std::string vs = o.version_string();
      std::string js = o.to_json();
      std::string jse;
      for (char ch : js) { if (ch == '"') jse += "\\\""; else jse += ch; }
      std::snprintf(b, sizeof(b),
        "    {\"fn\": \"firmware_info\", \"payload\": \"%s\", \"ok\": %s, \"fields\": "
        "{\"major\": %u, \"minor\": %u, \"patch\": %u, \"protocol_version\": %u, \"git_sha32\": %u, "
        "\"build_unix\": %u, \"build_flags\": %u, \"dirty\": %s, \"unversioned\": %s, "
        "\"version_string\": \"%s\", \"json\": \"%s\"}}",
        hex(c).c_str(), ok ? "true" : "false", o.major, o.minor, o.patch, o.protocol_version,
        o.git_sha32, o.build_unix, o.build_flags, o.dirty() ? "true" : "false",
        o.unversioned() ? "true" : "false", vs.c_str(), jse.c_str());
      dec.push_back(b);
    }
    auto wrong = std::vector<uint8_t>(15, 0);
    FirmwareInfo o;
    bool ok = decode_firmware_info(wrong.data(), wrong.size(), o);
    dec.push_back(std::string("    {\"fn\": \"firmware_info\", \"payload\": \"") + hex(wrong) +
      "\", \"ok\": " + (ok ? "true" : "false") + ", \"fields\": null}");
  }
  // charger status (0x89, 16)
  {
    std::vector<std::vector<uint8_t>> cases = {
      mk({0x10,0x0A, 0x20,0x01, 0x1E,0x00, 0x01,0x00, 0x02,0x00, 0x1B, 0x02, 0xF4,0x01, 0x03, 0}),
      mk({0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0, 0xFF,0xFF, 0, 0}),
    };
    for (auto & c : cases) {
      ChargerStatus o;
      bool ok = decode_charger_status(c.data(), c.size(), o);
      char b[768];
      std::snprintf(b, sizeof(b),
        "    {\"fn\": \"charger_status\", \"payload\": \"%s\", \"ok\": %s, \"fields\": "
        "{\"voltage_cv\": %u, \"current_ca\": %u, \"temp_c\": %u, \"reg3\": %u, \"reg4\": %u, "
        "\"flags\": %u, \"comm_error_count\": %u, \"age_ms\": %u, \"last_exception_code\": %u, "
        "\"online\": %s, \"current_present\": %s, \"input_present\": %s}}",
        hex(c).c_str(), ok ? "true" : "false", o.voltage_cv, o.current_ca, o.temp_c, o.reg3, o.reg4,
        o.flags, o.comm_error_count, o.age_ms, o.last_exception_code,
        o.online() ? "true" : "false", o.current_present() ? "true" : "false",
        o.input_present() ? "true" : "false");
      dec.push_back(b);
    }
    auto wrong = std::vector<uint8_t>(17, 0);
    ChargerStatus o;
    bool ok = decode_charger_status(wrong.data(), wrong.size(), o);
    dec.push_back(std::string("    {\"fn\": \"charger_status\", \"payload\": \"") + hex(wrong) +
      "\", \"ok\": " + (ok ? "true" : "false") + ", \"fields\": null}");
  }
  // servo status (0x88, 8)
  {
    std::vector<std::vector<uint8_t>> cases = {
      mk({0xDC,0x05, 0xE8,0x03, 0x64,0x00, 0x3F, 0x07}),
      mk({0,0, 0,0, 0,0, 0x00, 0}),
    };
    for (auto & c : cases) {
      ServoStatus o;
      bool ok = decode_servo_status(c.data(), c.size(), o);
      char b[768];
      std::snprintf(b, sizeof(b),
        "    {\"fn\": \"servo_status\", \"payload\": \"%s\", \"ok\": %s, \"fields\": "
        "{\"pulse_us\": %u, \"hold_timeout_ms\": %u, \"command_age_ms\": %u, \"flags\": %u, "
        "\"last_rx_seq\": %u, \"enabled\": %s, \"limit_active\": %s, \"output_active\": %s, "
        "\"timed_out\": %s, \"limit_up\": %s, \"limit_down\": %s}}",
        hex(c).c_str(), ok ? "true" : "false", o.pulse_us, o.hold_timeout_ms, o.command_age_ms,
        o.flags, o.last_rx_seq, o.enabled() ? "true" : "false", o.limit_active() ? "true" : "false",
        o.output_active() ? "true" : "false", o.timed_out() ? "true" : "false",
        o.limit_up() ? "true" : "false", o.limit_down() ? "true" : "false");
      dec.push_back(b);
    }
    auto wrong = std::vector<uint8_t>(9, 0);
    ServoStatus o;
    bool ok = decode_servo_status(wrong.data(), wrong.size(), o);
    dec.push_back(std::string("    {\"fn\": \"servo_status\", \"payload\": \"") + hex(wrong) +
      "\", \"ok\": " + (ok ? "true" : "false") + ", \"fields\": null}");
  }
  // lawer (blade) motor status (0x82, 8)
  {
    std::vector<std::vector<uint8_t>> cases = {
      mk({0x2C,0x01, 0x58,0x02, 0x32,0x00, 0x01, 0x0A}),
      mk({0x00,0x00, 0x00,0x00, 0xFF,0xFF, 0x02, 0xFF}),
    };
    for (auto & c : cases) {
      LawerMotorStatus o;
      bool ok = decode_lawer_motor_status(c.data(), c.size(), o);
      char b[512];
      std::snprintf(b, sizeof(b),
        "    {\"fn\": \"lawer_motor_status\", \"payload\": \"%s\", \"ok\": %s, \"fields\": "
        "{\"commanded_permille\": %d, \"applied_pwm\": %d, \"command_age_ms\": %u, \"flags\": %u, "
        "\"last_rx_seq\": %u}}",
        hex(c).c_str(), ok ? "true" : "false", o.commanded_permille, o.applied_pwm, o.command_age_ms,
        o.flags, o.last_rx_seq);
      dec.push_back(b);
    }
    auto wrong = std::vector<uint8_t>(7, 0);
    LawerMotorStatus o;
    bool ok = decode_lawer_motor_status(wrong.data(), wrong.size(), o);
    dec.push_back(std::string("    {\"fn\": \"lawer_motor_status\", \"payload\": \"") + hex(wrong) +
      "\", \"ok\": " + (ok ? "true" : "false") + ", \"fields\": null}");
  }
  for (size_t i = 0; i < dec.size(); ++i) out += dec[i] + (i + 1 == dec.size() ? "\n" : ",\n");
  out += "  ]\n}\n";

  std::fputs(out.c_str(), stdout);
  return 0;
}
