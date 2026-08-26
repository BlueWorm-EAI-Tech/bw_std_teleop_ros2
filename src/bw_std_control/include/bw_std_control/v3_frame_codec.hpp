#ifndef BW_STD_CONTROL__V3_FRAME_CODEC_HPP_
#define BW_STD_CONTROL__V3_FRAME_CODEC_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "bw_std_control/v3_protocol.hpp"

namespace bw_std_control
{

std::uint16_t compute_modbus_crc16(const std::uint8_t * data, std::size_t size) noexcept;
V3CommandFrame encode_command_frame(const V3CommandPayload & payload) noexcept;
bool decode_feedback_frame(
  const std::uint8_t * frame, std::size_t size, V3FeedbackPayload & feedback) noexcept;

class V3StreamParser
{
public:
  void append(const std::uint8_t * data, std::size_t size) noexcept;
  bool pop_feedback(V3FeedbackPayload & feedback) noexcept;
  void clear() noexcept;

private:
  static constexpr std::size_t kBufferCapacity = kV3FeedbackFrameSize * 8U;

  void discard_prefix(std::size_t count) noexcept;

  std::array<std::uint8_t, kBufferCapacity> buffer_{};
  std::size_t size_{0};
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__V3_FRAME_CODEC_HPP_
