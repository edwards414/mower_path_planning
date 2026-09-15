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
