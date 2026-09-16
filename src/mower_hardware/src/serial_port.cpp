#include "mower_hardware/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace mower_hardware {

namespace {

speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return 0;
  }
}

}  // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string & device, int baud)
{
  close();
  speed_t speed = baud_to_speed(baud);
  if (speed == 0) {
    last_error_ = "unsupported baud rate " + std::to_string(baud);
    return false;
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    last_error_ = std::string("open ") + device + ": " + std::strerror(errno);
    return false;
  }

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    last_error_ = std::string("tcgetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
  tio.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB);
  tio.c_cflag = (tio.c_cflag & ~static_cast<tcflag_t>(CSIZE)) | CS8;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    last_error_ = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  last_error_.clear();
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int SerialPort::read_some(uint8_t * buf, size_t len)
{
  if (fd_ < 0) {
    return -1;
  }
  ssize_t n = ::read(fd_, buf, len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    last_error_ = std::string("read: ") + std::strerror(errno);
    return -1;
  }
  return static_cast<int>(n);
}

bool SerialPort::write_all(const uint8_t * buf, size_t len)
{
  if (fd_ < 0) {
    return false;
  }
  size_t off = 0;
  while (off < len) {
    ssize_t n = ::write(fd_, buf + off, len - off);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      last_error_ = std::string("write: ") + std::strerror(errno);
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace mower_hardware
