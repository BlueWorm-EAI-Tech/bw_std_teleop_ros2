#include "bw_std_control/frame_codec.hpp"

#include <algorithm>
#include <cstring>

namespace bw_std_control
{
namespace
{

constexpr std::uint16_t kCrcInitialValue = 0xFFFF;
constexpr std::uint16_t kCrcPolynomial = 0xA001;

std::uint16_t read_u16_le(const std::uint8_t * const data) noexcept
{
  return static_cast<std::uint16_t>(data[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U);
}

void write_u16_le(const std::uint16_t value, std::uint8_t * const data) noexcept
{
  data[0] = static_cast<std::uint8_t>(value & 0x00FFU);
  data[1] = static_cast<std::uint8_t>((value >> 8U) & 0x00FFU);
}

}  // namespace

std::uint16_t compute_modbus_crc16(
  const std::uint8_t * const data, const std::size_t size) noexcept
{
  if (data == nullptr && size != 0U) {
    return kCrcInitialValue;
  }
  std::uint16_t crc = kCrcInitialValue;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x0001U) != 0U ?
        static_cast<std::uint16_t>((crc >> 1U) ^ kCrcPolynomial) :
        static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

CommandFrame encode_command_frame(const CommandPayload & payload) noexcept
{
  CommandFrame frame{};
  frame[0] = kHeader0;
  frame[1] = kHeader1;
  frame[2] = kCommandType;
  write_u16_le(static_cast<std::uint16_t>(kCommandPayloadSize), frame.data() + 3);
  std::memcpy(frame.data() + kPrefixSize, &payload, sizeof(payload));
  const auto crc = compute_modbus_crc16(frame.data() + 2, 3 + kCommandPayloadSize);
  write_u16_le(crc, frame.data() + kPrefixSize + kCommandPayloadSize);
  return frame;
}

ChassisCommandFrame encode_chassis_command_frame(
  const ChassisCommandPayload & payload) noexcept
{
  ChassisCommandFrame frame{};
  frame[0] = kHeader0;
  frame[1] = kHeader1;
  frame[2] = kChassisCommandType;
  write_u16_le(static_cast<std::uint16_t>(kChassisCommandPayloadSize), frame.data() + 3);
  std::memcpy(frame.data() + kPrefixSize, &payload, sizeof(payload));
  const auto crc = compute_modbus_crc16(
    frame.data() + 2, 3 + kChassisCommandPayloadSize);
  write_u16_le(crc, frame.data() + kPrefixSize + kChassisCommandPayloadSize);
  return frame;
}

bool decode_feedback_frame(
  const std::uint8_t * const frame, const std::size_t size,
  FeedbackPayload & feedback) noexcept
{
  if (frame == nullptr || size != kFeedbackFrameSize || frame[0] != kHeader0 ||
    frame[1] != kHeader1 || frame[2] != kFeedbackType ||
    read_u16_le(frame + 3) != kFeedbackPayloadSize)
  {
    return false;
  }
  const auto received_crc = read_u16_le(frame + kPrefixSize + kFeedbackPayloadSize);
  const auto expected_crc = compute_modbus_crc16(frame + 2, 3 + kFeedbackPayloadSize);
  if (received_crc != expected_crc) {
    return false;
  }
  std::memcpy(&feedback, frame + kPrefixSize, sizeof(feedback));
  return true;
}

void StreamParser::append(const std::uint8_t * const data, const std::size_t size) noexcept
{
  if (data == nullptr || size == 0U) {
    return;
  }
  if (size >= buffer_.size()) {
    const auto * const tail = data + size - buffer_.size();
    std::copy(tail, tail + buffer_.size(), buffer_.begin());
    size_ = buffer_.size();
    return;
  }
  const std::size_t required = size_ + size;
  if (required > buffer_.size()) {
    discard_prefix(required - buffer_.size());
  }
  std::copy(data, data + size, buffer_.begin() + static_cast<std::ptrdiff_t>(size_));
  size_ += size;
}

bool StreamParser::pop_feedback(FeedbackPayload & feedback) noexcept
{
  while (size_ >= 2U) {
    std::size_t header_index = 0;
    while (header_index + 1U < size_ &&
      (buffer_[header_index] != kHeader0 || buffer_[header_index + 1U] != kHeader1))
    {
      ++header_index;
    }
    if (header_index + 1U >= size_) {
      const bool preserve_header = buffer_[size_ - 1U] == kHeader0;
      if (preserve_header) {
        buffer_[0] = kHeader0;
      }
      size_ = preserve_header ? 1U : 0U;
      return false;
    }
    if (header_index > 0U) {
      discard_prefix(header_index);
    }
    if (size_ < kPrefixSize) {
      return false;
    }
    if (buffer_[2] != kFeedbackType ||
      read_u16_le(buffer_.data() + 3) != kFeedbackPayloadSize)
    {
      discard_prefix(1U);
      continue;
    }
    if (size_ < kFeedbackFrameSize) {
      return false;
    }
    if (!decode_feedback_frame(buffer_.data(), kFeedbackFrameSize, feedback)) {
      discard_prefix(1U);
      continue;
    }
    discard_prefix(kFeedbackFrameSize);
    return true;
  }
  return false;
}

void StreamParser::clear() noexcept
{
  size_ = 0U;
}

void StreamParser::discard_prefix(const std::size_t count) noexcept
{
  if (count >= size_) {
    size_ = 0U;
    return;
  }
  std::move(
    buffer_.begin() + static_cast<std::ptrdiff_t>(count),
    buffer_.begin() + static_cast<std::ptrdiff_t>(size_), buffer_.begin());
  size_ -= count;
}

}  // namespace bw_std_control
