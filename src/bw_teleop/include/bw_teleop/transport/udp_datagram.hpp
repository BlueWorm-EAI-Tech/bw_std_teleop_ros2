#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace bw_teleop::transport
{

struct UdpDatagram
{
  std::vector<std::uint8_t> payload;
  std::string source_ip;
};

}  // namespace bw_teleop::transport
