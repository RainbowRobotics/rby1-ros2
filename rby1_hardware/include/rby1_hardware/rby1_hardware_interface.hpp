#ifndef RBY1_HARDWARE_INTERFACE_HPP_
#define RBY1_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_command_builder.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rby1_hardware {

class RBY1HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RBY1HardwareInterface() = default;

  // ROS2 lifecycle callbacks
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Interface export
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read & Write
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<rb::Robot<rb::y1_model::A>> robot_;
  std::unique_ptr<rb::RobotCommandStreamHandler<rb::y1_model::A>> stream_;

  std::string address_;

  std::vector<double> joint_positions_;
  std::vector<double> joint_commands_;
  std::vector<std::string> joint_names_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  bool initialized_ = false;

  std::mutex state_mutex_;
  rb::RobotState<rb::y1_model::A> latest_state_;
  bool state_received_ = false;
};

}  // namespace rby1_hardware

#endif  // RBY1_HARDWARE_INTERFACE_HPP_
