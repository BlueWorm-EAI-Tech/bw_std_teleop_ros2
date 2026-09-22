#ifndef BW_STD_CONTROL__FRAME_CODEC_HPP_
#define BW_STD_CONTROL__FRAME_CODEC_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "bw_std_control/protocol.hpp"

namespace bw_std_control
{

std::uint16_t compute_modbus_crc16(const std::uint8_t * data, std::size_t size) noexcept;
CommandFrame encode_command_frame(const CommandPayload & payload) noexcept;
ChassisCommandFrame encode_chassis_command_frame(
  const ChassisCommandPayload & payload) noexcept;
bool decode_feedback_frame(
  const std::uint8_t * frame, std::size_t size, FeedbackPayload & feedback) noexcept;

class StreamParser
{
public:
  void append(const std::uint8_t * data, std::size_t size) noexcept;
  bool pop_feedback(FeedbackPayload & feedback) noexcept;
  void clear() noexcept;

private:
  static constexpr std::size_t kBufferCapacity = kFeedbackFrameSize * 8U;

  void discard_prefix(std::size_t count) noexcept;

  std::array<std::uint8_t, kBufferCapacity> buffer_{};
  std::size_t size_{0};
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__FRAME_CODEC_HPP_
