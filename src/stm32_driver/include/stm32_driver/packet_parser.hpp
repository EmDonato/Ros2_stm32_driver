#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stm32_driver
{

struct Packet
{
  uint8_t type{0};
  std::vector<uint8_t> payload;
};

struct ParseBatch
{
  std::vector<Packet> packets;
  std::size_t checksum_errors{0};
};

class PacketParser
{
public:
  ParseBatch consume(const std::vector<uint8_t> & bytes);

private:
  enum class State
  {
    wait_header_1,
    wait_header_2,
    wait_type,
    wait_length,
    wait_payload,
    wait_checksum
  };

  State state_{State::wait_header_1};
  uint8_t message_type_{0};
  uint8_t message_length_{0};
  uint8_t checksum_{0};
  std::vector<uint8_t> payload_;
};

}  // namespace stm32_driver
