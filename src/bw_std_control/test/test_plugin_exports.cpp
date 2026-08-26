#include <algorithm>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include "bw_std_control/standard_base_controller.hpp"
#include "bw_std_control/standard_system_hardware.hpp"

namespace bw_std_control
{
namespace
{

TEST(PluginExports, TypesImplementRos2ControlBases)
{
  static_assert(std::is_base_of_v<hardware_interface::SystemInterface, StandardSystemHardware>);
  static_assert(std::is_base_of_v<controller_interface::ControllerInterface, StandardBaseController>);
}

TEST(PluginExports, HardwarePluginCanBeLoadedByPublishedName)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  ASSERT_TRUE(loader.isClassAvailable("bw_std_control/StandardSystemHardware"));
  EXPECT_NE(loader.createSharedInstance("bw_std_control/StandardSystemHardware"), nullptr);
}

TEST(PluginExports, BaseControllerCanBeLoadedByPublishedName)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  ASSERT_TRUE(loader.isClassAvailable("bw_std_control/StandardBaseController"));
  EXPECT_NE(loader.createSharedInstance("bw_std_control/StandardBaseController"), nullptr);
}

}  // namespace
}  // namespace bw_std_control
