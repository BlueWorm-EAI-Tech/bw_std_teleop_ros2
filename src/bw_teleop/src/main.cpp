#include "bw_teleop/node/main.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "bw_teleop/node/bw_teleop.hpp"

namespace bw_teleop::node
{

int run(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BwTeleop>());
  rclcpp::shutdown();
  return 0;
}

}  // namespace bw_teleop::node

int main(int argc, char ** argv)
{
  return bw_teleop::node::run(argc, argv);
}
