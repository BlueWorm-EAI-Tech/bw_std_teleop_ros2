#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "bw_kinematics/node/kinematics_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bw_kinematics::KinematicsNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
