// Minimal POSIX serial port (termios), non-blocking reads. No ROS dependencies.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace mower_hardware {

class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Opens 8N1, no flow control, raw mode. Returns false on failure (see last_error()).
  bool open(const std::string & device, int baud);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Non-blocking: returns bytes read (0 if none), -1 on error.
  int read_some(uint8_t * buf, size_t len);
  // Blocking write of the whole buffer. Returns false on error.
  bool write_all(const uint8_t * buf, size_t len);

  const std::string & last_error() const { return last_error_; }

private:
  int fd_ = -1;
  std::string last_error_;
};

}  // namespace mower_hardware
