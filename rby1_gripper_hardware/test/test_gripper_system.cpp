#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rby1_gripper_hardware/rby1_gripper_system.hpp"

namespace
{

const char * kUrdf = R"(<?xml version="1.0"?>
<robot name="g"><link name="base"/>
<link name="r1"/><link name="r2"/><link name="l1"/><link name="l2"/>
<joint name="gripper_finger_r1" type="prismatic"><parent link="base"/><child link="r1"/><limit lower="-0.05" upper="0" effort="10" velocity="0.05"/></joint>
<joint name="gripper_finger_r2" type="prismatic"><parent link="base"/><child link="r2"/><limit lower="-0.05" upper="0" effort="10" velocity="0.05"/></joint>
<joint name="gripper_finger_l1" type="prismatic"><parent link="base"/><child link="l1"/><limit lower="-0.05" upper="0" effort="10" velocity="0.05"/></joint>
<joint name="gripper_finger_l2" type="prismatic"><parent link="base"/><child link="l2"/><limit lower="-0.05" upper="0" effort="10" velocity="0.05"/></joint>
<ros2_control name="RBY1Gripper" type="system">
  <hardware><plugin>rby1_gripper_hardware/Rby1GripperSystem</plugin><param name="backend">fake</param></hardware>
  <joint name="gripper_finger_r1"><command_interface name="position"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
  <joint name="gripper_finger_r2"><param name="mirror_of">gripper_finger_r1</param><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
  <joint name="gripper_finger_l1"><command_interface name="position"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
  <joint name="gripper_finger_l2"><param name="mirror_of">gripper_finger_l1</param><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
</ros2_control></robot>)";

double value(std::vector<hardware_interface::StateInterface> & states, const std::string & name)
{
  for (auto & s : states) {
    if (s.get_name() == name) {
#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
      return s.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
#else
      return s.get_value();
#endif
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

TEST(GripperSystem, FakeBusFollowsCommandsAndMirrors)
{
  const auto info = hardware_interface::parse_control_resources_from_urdf(kUrdf).at(0);
  rby1_gripper_hardware::Rby1GripperSystem gripper;
#ifdef RBY1_GRIPPER_PARAMS_INIT
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(gripper.on_init(params), hardware_interface::CallbackReturn::SUCCESS);
#else
  ASSERT_EQ(gripper.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
#endif
  auto states = gripper.export_state_interfaces();
  auto commands = gripper.export_command_interfaces();
  ASSERT_EQ(commands.size(), 2u);
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(gripper.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(gripper.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);

  const rclcpp::Time t;
  const rclcpp::Duration d(0, 10000000);
  for (auto & c : commands) {
    (void)c.set_value(-0.025);  // half open
  }
  for (int i = 0; i < 150; ++i) {
    gripper.write(t, d);
    gripper.read(t, d);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(value(states, "gripper_finger_r1/position"), -0.025, 1e-3);
  EXPECT_NEAR(value(states, "gripper_finger_r2/position"), value(states, "gripper_finger_r1/position"), 1e-12);
  EXPECT_NEAR(value(states, "gripper_finger_l1/position"), -0.025, 1e-3);
  gripper.on_deactivate(state);
  gripper.on_cleanup(state);
}

}  // namespace
