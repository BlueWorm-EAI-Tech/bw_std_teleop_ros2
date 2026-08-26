#include <array>
#include <cstdint>
#include <cstring>

#include <gtest/gtest.h>

#include "bw_std_control/v3_frame_codec.hpp"

namespace bw_std_control
{
namespace
{

void write_u16_le(const std::uint16_t value, std::uint8_t * const data)
{
  data[0] = static_cast<std::uint8_t>(value & 0x00FFU);
  data[1] = static_cast<std::uint8_t>((value >> 8U) & 0x00FFU);
}

V3FeedbackFrame make_feedback_frame(const V3FeedbackPayload & payload)
{
  V3FeedbackFrame frame{};
  frame[0] = kV3Header0;
  frame[1] = kV3Header1;
  frame[2] = kV3FeedbackType;
  write_u16_le(static_cast<std::uint16_t>(kV3FeedbackPayloadSize), frame.data() + 3);
  std::memcpy(frame.data() + kV3PrefixSize, &payload, sizeof(payload));
  const auto crc = compute_modbus_crc16(frame.data() + 2, 3 + kV3FeedbackPayloadSize);
  write_u16_le(crc, frame.data() + kV3PrefixSize + kV3FeedbackPayloadSize);
  return frame;
}

TEST(V3FrameCodec, EncodesFixedCommandEnvelopeAndCrc)
{
  V3CommandPayload payload{};
  payload.control_flag = 1U;
  payload.vx = 0.25F;

  const auto frame = encode_command_frame(payload);

  EXPECT_EQ(frame.size(), kV3CommandFrameSize);
  EXPECT_EQ(frame[0], kV3Header0);
  EXPECT_EQ(frame[1], kV3Header1);
  EXPECT_EQ(frame[2], kV3CommandType);
  EXPECT_EQ(frame[kV3PrefixSize], 1U);
  const auto expected_crc = compute_modbus_crc16(frame.data() + 2, 3 + kV3CommandPayloadSize);
  const auto encoded_crc = static_cast<std::uint16_t>(frame[frame.size() - 2]) |
    static_cast<std::uint16_t>(frame.back()) << 8U;
  EXPECT_EQ(encoded_crc, expected_crc);
}

TEST(V3FrameCodec, RejectsCorruptedFeedback)
{
  V3FeedbackPayload payload{};
  payload.pelvis_height = 1520.0F;
  auto frame = make_feedback_frame(payload);
  frame[20] ^= 0x01U;

  V3FeedbackPayload decoded{};
  EXPECT_FALSE(decode_feedback_frame(frame.data(), frame.size(), decoded));
}

TEST(V3StreamParser, RecoversFromNoiseAndFragmentedFrame)
{
  V3FeedbackPayload payload{};
  payload.chassis_vx = 0.35F;
  const auto frame = make_feedback_frame(payload);
  const std::array<std::uint8_t, 4> noise{0x00U, 0x55U, 0x01U, 0xAAU};
  V3StreamParser parser;

  parser.append(noise.data(), noise.size());
  parser.append(frame.data(), 19U);
  V3FeedbackPayload decoded{};
  EXPECT_FALSE(parser.pop_feedback(decoded));
  parser.append(frame.data() + 19U, frame.size() - 19U);

  ASSERT_TRUE(parser.pop_feedback(decoded));
  EXPECT_FLOAT_EQ(decoded.chassis_vx, payload.chassis_vx);
}

}  // namespace
}  // namespace bw_std_control
