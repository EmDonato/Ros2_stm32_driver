#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace stm32_driver
{

class SerialPort
{
public:
  SerialPort(const std::string & device, int baudrate);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  std::vector<uint8_t> read_available();
  void write_all(const uint8_t * data, std::size_t length);
  void flush_output();

private:
  int file_descriptor_{-1};
};

}  // namespace stm32_driver
