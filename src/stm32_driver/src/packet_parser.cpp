#include "stm32_driver/packet_parser.hpp"

#include "stm32_driver/parsing_types.hpp"

namespace stm32_driver
{

ParseBatch PacketParser::consume(const std::vector<uint8_t> & bytes)
{
  ParseBatch batch;
  for (const uint8_t byte : bytes) {
    switch (state_) {
      case State::wait_header_1:
        if (byte == HDR1) {
          checksum_ = byte;
          state_ = State::wait_header_2;
        }
        break;
      case State::wait_header_2:
        if (byte == HDR2) {
          checksum_ ^= byte;
          state_ = State::wait_type;
        } else {
          state_ = State::wait_header_1;
        }
        break;
      case State::wait_type:
        message_type_ = byte;
        checksum_ ^= byte;
        state_ = State::wait_length;
        break;
      case State::wait_length:
        message_length_ = byte;
        payload_.clear();
        checksum_ ^= byte;
        state_ = message_length_ == 0 ? State::wait_checksum : State::wait_payload;
        break;
      case State::wait_payload:
        payload_.push_back(byte);
        checksum_ ^= byte;
        if (payload_.size() >= message_length_) {
          state_ = State::wait_checksum;
        }
        break;
      case State::wait_checksum:
        if (checksum_ == byte) {
          batch.packets.push_back(Packet{message_type_, payload_});
        } else {
          ++batch.checksum_errors;
        }
        state_ = State::wait_header_1;
        break;
    }
  }
  return batch;
}

}  // namespace stm32_driver
