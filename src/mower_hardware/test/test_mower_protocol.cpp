#include <gtest/gtest.h>

#include <cstring>

#include "mower_hardware/mower_protocol.hpp"

using namespace mower_hardware;

TEST(Crc, CheckValue)
{
  // CRC-16/CCITT-FALSE check value for "123456789" is 0x29B1
  const char * s = "123456789";
  EXPECT_EQ(crc16_ccitt_false(reinterpret_cast<const uint8_t *>(s), 9), 0x29B1);
}

TEST(Build, WheelSpeedCommandMatchesReference)
{
  // Reference frame captured from the Python bench tool:
  // seq=7, left=100, right=-100, timeout=300 -> a55a0101070864009cff2c010000b138
  auto f = build_wheel_speed_command(7, 100, -100, 300);
  const uint8_t expect[] = {0xa5, 0x5a, 0x01, 0x01, 0x07, 0x08, 0x64, 0x00,
                            0x9c, 0xff, 0x2c, 0x01, 0x00, 0x00, 0xb1, 0x38};
  ASSERT_EQ(f.size(), sizeof(expect));
  EXPECT_EQ(std::memcmp(f.data(), expect, sizeof(expect)), 0);
}

TEST(Parser, RoundTripAndResync)
{
  auto f = build_wheel_speed_command(7, 100, -100, 300);
  std::vector<uint8_t> stream = {0x00, 0xA5, 0x00};  // junk + a lone SOF0
  stream.insert(stream.end(), f.begin(), f.end());
  stream.insert(stream.end(), f.begin(), f.begin() + 5);  // partial second frame

  FrameParser p;
  int frames = 0;
  p.feed(stream.data(), stream.size(), [&](uint8_t t, uint8_t s, const uint8_t * pl, size_t l) {
    ++frames;
    EXPECT_EQ(t, kWheelSpeedCommand);
    EXPECT_EQ(s, 7);
    EXPECT_EQ(l, 8u);
    EXPECT_EQ(pl[0], 0x64);
  });
  EXPECT_EQ(frames, 1);
  EXPECT_EQ(p.crc_errors(), 0u);

  // complete the partial frame byte by byte
  for (size_t i = 5; i < f.size(); ++i) {
    p.feed(&f[i], 1, [&](uint8_t, uint8_t, const uint8_t *, size_t) { ++frames; });
  }
  EXPECT_EQ(frames, 2);
}

TEST(Parser, CrcErrorIsCountedAndSkipped)
{
  auto good = build_wheel_speed_command(1, 0, 0, 300);
  auto bad = good;
  bad[8] ^= 0xFF;  // corrupt payload
  std::vector<uint8_t> stream(bad);
  stream.insert(stream.end(), good.begin(), good.end());

  FrameParser p;
  int frames = 0;
  p.feed(stream.data(), stream.size(), [&](uint8_t, uint8_t s, const uint8_t *, size_t) {
    ++frames;
    EXPECT_EQ(s, 1);
  });
  EXPECT_EQ(frames, 1);
  EXPECT_EQ(p.crc_errors(), 1u);
}

TEST(Decode, WheelFeedback)
{
  // left tgt 17.40 rpm, meas -4.72, right 17.40 / -4.38, pid 44/44,
  // total L=-14 R=-13, flags OUT_EN|CLOSED_LOOP
  uint8_t p[24] = {};
  auto put16 = [&](int off, int16_t v) { p[off] = v & 0xFF; p[off + 1] = (v >> 8) & 0xFF; };
  auto put32 = [&](int off, int32_t v) {
    for (int i = 0; i < 4; ++i) { p[off + i] = (static_cast<uint32_t>(v) >> (8 * i)) & 0xFF; }
  };
  put16(0, 1740); put16(2, -472); put16(4, 1740); put16(6, -438);
  put16(8, 44); put16(10, 44);
  put32(12, -14); put32(16, -13);
  p[20] = 0x03;

  WheelFeedback fb;
  ASSERT_TRUE(decode_wheel_feedback(9, p, sizeof(p), fb));
  EXPECT_DOUBLE_EQ(fb.left_target_rpm, 17.40);
  EXPECT_DOUBLE_EQ(fb.left_measured_rpm, -4.72);
  EXPECT_DOUBLE_EQ(fb.right_measured_rpm, -4.38);
  EXPECT_EQ(fb.left_total_counts, -14);
  EXPECT_EQ(fb.right_total_counts, -13);
  EXPECT_EQ(fb.flags, 0x03);
  EXPECT_EQ(fb.seq, 9);
  EXPECT_FALSE(decode_wheel_feedback(9, p, 23, fb));
}

TEST(Decode, TotalCountsWrapAround)
{
  // odometry integrates the signed 32-bit difference; check the wrap math
  int32_t prev = 0x7FFFFFF0;
  int32_t now = static_cast<int32_t>(0x8000000F);  // 31 counts later, wrapped
  int32_t d = static_cast<int32_t>(static_cast<uint32_t>(now) - static_cast<uint32_t>(prev));
  EXPECT_EQ(d, 31);
}

TEST(Power, CommandFrameLayout)
{
  auto f = build_power_command(3, kPowerActionHostShutdownAck);
  ASSERT_EQ(f.size(), kFrameOverhead + 4);
  EXPECT_EQ(f[3], kPowerCommand);
  EXPECT_EQ(f[4], 3);
  EXPECT_EQ(f[5], 4);
  EXPECT_EQ(f[6], kPowerActionHostShutdownAck);
  EXPECT_EQ(crc16_ccitt_false(f.data() + 2, 4 + 4), static_cast<uint16_t>(f[10] | (f[11] << 8)));
}

TEST(Power, DecodeStatus)
{
  // state=3 (shutdown pending), flags=button|main|requested, reason=1 (button),
  // seq=9, press_ms=3010, elapsed=250
  const uint8_t p[8] = {0x03, 0x07, 0x01, 0x09, 0xC2, 0x0B, 0xFA, 0x00};
  PowerStatus ps;
  ASSERT_TRUE(decode_power_status(p, sizeof(p), ps));
  EXPECT_EQ(ps.state, kPowerStateShutdownPending);
  EXPECT_TRUE(ps.shutdown_requested());
  EXPECT_EQ(ps.shutdown_reason, 1);
  EXPECT_EQ(ps.last_rx_seq, 9);
  EXPECT_EQ(ps.press_ms, 3010);
  EXPECT_EQ(ps.shutdown_elapsed_ms, 250);
  EXPECT_FALSE(decode_power_status(p, 7, ps));
}

TEST(FirmwareInfo, DecodeAndFormat)
{
  // v0.6.1, protocol 1, sha f22f2465, built 1789571662, clean+versioned
  const uint8_t p[16] = {0x00, 0x06, 0x01, 0x01, 0x65, 0x24, 0x2F, 0xF2,
                         0x4E, 0xB2, 0xAA, 0x6A, 0x00, 0x00, 0x00, 0x00};
  FirmwareInfo fi;
  ASSERT_TRUE(decode_firmware_info(p, sizeof(p), fi));
  EXPECT_EQ(fi.major, 0);
  EXPECT_EQ(fi.minor, 6);
  EXPECT_EQ(fi.patch, 1);
  EXPECT_EQ(fi.protocol_version, kProtocolVersion);
  EXPECT_EQ(fi.git_sha32, 0xF22F2465u);
  EXPECT_EQ(fi.build_unix, 1789571662u);
  EXPECT_FALSE(fi.dirty());
  EXPECT_FALSE(fi.unversioned());
  EXPECT_EQ(fi.version_string(), "0.6.1+f22f2465");
  EXPECT_EQ(fi.to_json(),
    "{\"version\":\"0.6.1\",\"semver\":[0,6,1],\"protocol_version\":1,"
    "\"git_sha\":\"f22f2465\",\"git_sha32\":4063175781,\"build_unix\":1789571662,"
    "\"dirty\":false,\"unversioned\":false}");
  EXPECT_FALSE(decode_firmware_info(p, 15, fi));

  uint8_t q[16];
  std::memcpy(q, p, sizeof(q));
  q[12] = kFwBuildFlagDirty | kFwBuildFlagUnversioned;
  FirmwareInfo fj;
  ASSERT_TRUE(decode_firmware_info(q, sizeof(q), fj));
  EXPECT_TRUE(fj.dirty());
  EXPECT_TRUE(fj.unversioned());
  EXPECT_NE(fi, fj);
  EXPECT_EQ(fj.version_string(), "0.6.1+f22f2465.dirty.unversioned");

  auto f = build_info_request(7);
  ASSERT_EQ(f.size(), kFrameOverhead);
  EXPECT_EQ(f[3], kInfoRequest);
  EXPECT_EQ(f[4], 7);
  EXPECT_EQ(f[5], 0);
}

TEST(Ws2812, CommandFrameLayout)
{
  // orbit, amber, one revolution per 1600 ms
  auto f = build_ws2812_command(9, kLedOrbit, 255, 180, 0, 1600);
  ASSERT_EQ(f.size(), kFrameOverhead + 8);
  EXPECT_EQ(f[3], kWs2812Command);
  EXPECT_EQ(f[4], 9);
  EXPECT_EQ(f[5], 8);
  EXPECT_EQ(f[6], kLedOrbit);
  EXPECT_EQ(f[7], 255);
  EXPECT_EQ(f[8], 180);
  EXPECT_EQ(f[9], 0);
  EXPECT_EQ(f[10], 1600 & 0xFF);
  EXPECT_EQ(f[11], 1600 >> 8);
  EXPECT_EQ(f[12], 0);
  EXPECT_EQ(f[13], 0);
  EXPECT_EQ(crc16_ccitt_false(f.data() + 2, 4 + 8), static_cast<uint16_t>(f[14] | (f[15] << 8)));
}

TEST(Ws2812, DecodeStatus)
{
  // mode=flow, r/g/b=10/20/30, period=1600, flags=1, seq=7
  const uint8_t p[8] = {kLedFlow, 10, 20, 30, 0x40, 0x06, 0x01, 0x07};
  Ws2812Status st;
  ASSERT_TRUE(decode_ws2812_status(p, sizeof(p), st));
  EXPECT_EQ(st.mode, kLedFlow);
  EXPECT_EQ(st.r, 10);
  EXPECT_EQ(st.g, 20);
  EXPECT_EQ(st.b, 30);
  EXPECT_EQ(st.effect_period_ms, 1600);
  EXPECT_EQ(st.flags, 1);
  EXPECT_EQ(st.last_rx_seq, 7);
  EXPECT_FALSE(decode_ws2812_status(p, 7, st));
}

TEST(Pid, DecodeConfigStatus)
{
  // the firmware defaults: 2.0 / 0.6 / 0.0 per wheel, closed loop + flash valid
  uint8_t p[28] = {};
  const float gains[6] = {2.0f, 0.6f, 0.0f, 2.0f, 0.6f, 0.0f};
  std::memcpy(p, gains, sizeof(gains));
  p[24] = kPidFlagClosedLoop | kPidFlagFlashValid;
  p[25] = 3;
  PidConfigStatus st;
  ASSERT_TRUE(decode_pid_config_status(p, sizeof(p), st));
  EXPECT_FLOAT_EQ(st.left_kp, 2.0f);
  EXPECT_FLOAT_EQ(st.left_ki, 0.6f);
  EXPECT_FLOAT_EQ(st.left_kd, 0.0f);
  EXPECT_FLOAT_EQ(st.right_kp, 2.0f);
  EXPECT_FLOAT_EQ(st.right_ki, 0.6f);
  EXPECT_EQ(st.flags, kPidFlagClosedLoop | kPidFlagFlashValid);
  EXPECT_EQ(st.last_rx_seq, 3);
  EXPECT_FALSE(decode_pid_config_status(p, 27, st));
}

TEST(Pid, ConfigCommandMatchesReference)
{
  // Reference frame from the Python bench tool (tools/mower_uart.py build()):
  // seq=9, left 2.5/0.75/0, right 3/1/0.125, persist=1, closed_loop=1
  PidConfig cfg;
  cfg.left_kp = 2.5f;
  cfg.left_ki = 0.75f;
  cfg.left_kd = 0.0f;
  cfg.right_kp = 3.0f;
  cfg.right_ki = 1.0f;
  cfg.right_kd = 0.125f;
  cfg.persist_to_flash = true;
  cfg.closed_loop_enabled = true;
  auto f = build_pid_config_command(9, cfg);
  const uint8_t expect[] = {0xa5, 0x5a, 0x01, 0x04, 0x09, 0x1c,
                            0x00, 0x00, 0x20, 0x40, 0x00, 0x00, 0x40, 0x3f, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x3e,
                            0x01, 0x01, 0x00, 0x00, 0x19, 0x40};
  ASSERT_EQ(f.size(), sizeof(expect));
  EXPECT_EQ(std::memcmp(f.data(), expect, sizeof(expect)), 0);

  // the status decoder must read back what the command encoded
  PidConfigStatus st;
  ASSERT_TRUE(decode_pid_config_status(f.data() + 6, 28, st));
  EXPECT_FLOAT_EQ(st.left_kp, 2.5f);
  EXPECT_FLOAT_EQ(st.right_kd, 0.125f);
  EXPECT_EQ(st.flash_diag, 0u);

  // bytes 26..27 carry the flash diagnostics on newer firmware
  uint8_t p[28] = {};
  std::memcpy(p, f.data() + 6, 26);
  p[26] = 0x20;  // HAL_FLASH_ERROR_OPERATION
  p[27] = 0x80;  // PGSERR was pending
  ASSERT_TRUE(decode_pid_config_status(p, 28, st));
  EXPECT_EQ(st.flash_diag, 0x8020u);
}

TEST(Charger, DecodeStatus)
{
  // voltage=25.10 V, current=1.25 A, temp=35 C, reg3=11, reg4=48961,
  // flags=online|current|input|seen, err=2, age=120 ms, exc=0
  const uint8_t p[16] = {0xCE, 0x09, 0x7D, 0x00, 0x23, 0x00, 0x0B, 0x00,
                         0x41, 0xBF, 0x1B, 0x02, 0x78, 0x00, 0x00, 0x00};
  ChargerStatus ch;
  ASSERT_TRUE(decode_charger_status(p, sizeof(p), ch));
  EXPECT_EQ(ch.voltage_cv, 2510);
  EXPECT_EQ(ch.current_ca, 125);
  EXPECT_EQ(ch.temp_c, 35);
  EXPECT_EQ(ch.reg3, 11);
  EXPECT_EQ(ch.reg4, 48961);
  EXPECT_TRUE(ch.online());
  EXPECT_TRUE(ch.current_present());
  EXPECT_TRUE(ch.input_present());
  EXPECT_EQ(ch.comm_error_count, 2);
  EXPECT_EQ(ch.age_ms, 120);
  EXPECT_EQ(ch.last_exception_code, 0);
  EXPECT_FALSE(decode_charger_status(p, 15, ch));
}

TEST(Analog, DecodeStatus)
{
  // main=24.12 V, aon=3.85 V, temp=31.2 C, vdda=2910 mV, mg996 raw=123,
  // flags=main|aon|temp|curr|vdda_cal
  const uint8_t p[12] = {0x6C, 0x09, 0x81, 0x01, 0x38, 0x01,
                         0x5E, 0x0B, 0x7B, 0x00, 0x1F, 0x00};
  AnalogStatus an;
  ASSERT_TRUE(decode_analog_status(p, sizeof(p), an));
  EXPECT_EQ(an.main_battery_cv, 2412);
  EXPECT_EQ(an.aon_battery_cv, 385);
  EXPECT_EQ(an.board_temp_dc, 312);
  EXPECT_EQ(an.vdda_mv, 2910);
  EXPECT_EQ(an.mg996_current_raw, 123);
  EXPECT_TRUE(an.main_battery_valid());
  EXPECT_TRUE(an.aon_battery_valid());
  EXPECT_TRUE(an.board_temp_valid());
  EXPECT_FALSE(decode_analog_status(p, 11, an));

  // invalid temperature sentinel survives the signed decode
  const uint8_t q[12] = {0, 0, 0, 0, 0x00, 0x80, 0xE4, 0x0C, 0, 0, 0x00, 0};
  ASSERT_TRUE(decode_analog_status(q, sizeof(q), an));
  EXPECT_EQ(an.board_temp_dc, INT16_MIN);
  EXPECT_FALSE(an.board_temp_valid());
  EXPECT_FALSE(an.main_battery_valid());
}
