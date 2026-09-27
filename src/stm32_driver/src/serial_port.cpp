#include "stm32_driver/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <unistd.h>

namespace stm32_driver
{
namespace
{

speed_t termios_baud(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: throw std::invalid_argument("Unsupported baudrate: " + std::to_string(baudrate));
  }
}

void throw_system_error(const std::string & operation)
{
  throw std::system_error(errno, std::generic_category(), operation);
}

}  // namespace

SerialPort::SerialPort(const std::string & device, int baudrate)
{
  file_descriptor_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (file_descriptor_ < 0) {
    throw_system_error("Failed to open " + device);
  }

  try {
    termios tty{};
    if (tcgetattr(file_descriptor_, &tty) != 0) {
      throw_system_error("tcgetattr failed");
    }

    const speed_t speed = termios_baud(baudrate);
    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
      throw_system_error("Setting baudrate failed");
    }

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(file_descriptor_, TCSANOW, &tty) != 0) {
      throw_system_error("tcsetattr failed");
    }
    if (tcflush(file_descriptor_, TCIOFLUSH) != 0) {
      throw_system_error("tcflush failed");
    }
  } catch (...) {
    close(file_descriptor_);
    file_descriptor_ = -1;
    throw;
  }
}

SerialPort::~SerialPort()
{
  if (file_descriptor_ >= 0) {
    close(file_descriptor_);
  }
}

std::vector<uint8_t> SerialPort::read_available()
{
  std::vector<uint8_t> bytes;
  uint8_t buffer[128]{};
  pollfd descriptor{file_descriptor_, POLLIN, 0};
  int poll_result = 0;

  while ((poll_result = poll(&descriptor, 1, 0)) > 0) {
    if ((descriptor.revents & POLLIN) == 0) {
      break;
    }

    const ssize_t count = read(file_descriptor_, buffer, sizeof(buffer));
    if (count > 0) {
      bytes.insert(bytes.end(), buffer, buffer + count);
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      throw_system_error("Serial read failed");
    } else if (count == 0) {
      break;
    }
  }

  if (poll_result < 0 && errno != EINTR) {
    throw_system_error("Serial poll failed");
  }
  return bytes;
}

void SerialPort::write_all(const uint8_t * data, std::size_t length)
{
  std::size_t offset = 0;
  while (offset < length) {
    const ssize_t count = write(file_descriptor_, data + offset, length - offset);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd descriptor{file_descriptor_, POLLOUT, 0};
      if (poll(&descriptor, 1, 100) > 0) {
        continue;
      }
      throw std::runtime_error("Timed out while writing to the serial port");
    }
    throw_system_error("Serial write failed");
  }
}

void SerialPort::flush_output()
{
  if (tcflush(file_descriptor_, TCOFLUSH) != 0) {
    throw_system_error("Could not flush serial output");
  }
}

}  // namespace stm32_driver
