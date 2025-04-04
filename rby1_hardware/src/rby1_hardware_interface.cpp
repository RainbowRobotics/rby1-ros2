#include "rby1_hardware/rby1_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rby1_hardware
{

hardware_interface::CallbackReturn RBY1HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  address_ = info.hardware_parameters.at("robot_ip");

  robot_ = rb::Robot<rb::y1_model::A>::Create(address_);
  joint_positions_.resize(info.joints.size(), 0.0);
  joint_commands_.resize(info.joints.size(), 0.0);

  joint_names_.clear();
  for (const auto & joint : info.joints) {
    joint_names_.push_back(joint.name);
  }

  node_ = std::make_shared<rclcpp::Node>("rby1_hw_node");
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 100);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBY1HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (!robot_->Connect()) return hardware_interface::CallbackReturn::ERROR;
  if (!robot_->IsPowerOn(".*") && !robot_->PowerOn(".*")) return hardware_interface::CallbackReturn::ERROR;
  if (!robot_->IsServoOn(".*") && !robot_->ServoOn(".*")) return hardware_interface::CallbackReturn::ERROR;
  robot_->ResetFaultControlManager();
  if (!robot_->EnableControlManager()) return hardware_interface::CallbackReturn::ERROR;
  const auto& control_manager_state = robot_->GetControlManagerState();

  if (control_manager_state.state == rb::ControlManagerState::State::kMajorFault ||
      control_manager_state.state == rb::ControlManagerState::State::kMinorFault)
  {
    std::cerr << "Detected a "
              << (control_manager_state.state == rb::ControlManagerState::State::kMajorFault ? "Major" : "Minor")
              << " fault in the Control Manager." << std::endl;
  
    std::cout << "Attempting to reset the fault..." << std::endl;
    if (!robot_->ResetFaultControlManager()) {
      std::cerr << "Failed to reset the fault in the Control Manager." << std::endl;
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::cout << "Fault reset successfully." << std::endl;
  }
  else {
    std::cout << "Control Manager state is normal. No faults detected." << std::endl;
  }
  
  std::cout << "Enabling Control Manager..." << std::endl;
  if (!robot_->EnableControlManager()) {
    std::cerr << "Failed to enable the Control Manager." << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::cout << "Control Manager enabled successfully." << std::endl;
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBY1HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  auto state = robot_->GetState();
  if (state.position.size() != joint_positions_.size()) {
    RCLCPP_ERROR(node_->get_logger(), "Size mismatch when reading initial robot state.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    joint_positions_[i] = state.position(i);
  }
  joint_commands_ = joint_positions_; 
  
  robot_->StartStateUpdate(
    [this](const auto& state) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = state;
      state_received_ = true;
    },
    100.0
  );
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBY1HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  robot_->Disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RBY1HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RBY1HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type RBY1HardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  rb::RobotState<rb::y1_model::A> state_copy;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_received_) return hardware_interface::return_type::ERROR;
    state_copy = latest_state_;
  }

  if (state_copy.position.size() != joint_positions_.size())
    return hardware_interface::return_type::ERROR;

  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    joint_positions_[i] = state_copy.position(i);
  }

  sensor_msgs::msg::JointState js_msg;
  js_msg.header.stamp = node_->now();
  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    js_msg.name.push_back(info_.joints[i].name);
    js_msg.position.push_back(joint_positions_[i]);
  }
  joint_state_pub_->publish(js_msg);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RBY1HardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) 
{
  if (!initialized_) {
    stream_ = robot_->CreateCommandStream();
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "[WRITE] First time: stream created");
  }

  if (joint_commands_.size() != joint_names_.size()) return hardware_interface::return_type::ERROR;

  Eigen::Vector<double, 1> right_wheel;
  Eigen::Vector<double, 1> left_wheel;
  Eigen::Vector<double, 6> torso;
  Eigen::Vector<double, 7> right_arm;
  Eigen::Vector<double, 7> left_arm;
  Eigen::Vector<double, 2> head;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& name = joint_names_[i];
    if (name == "right_wheel") right_wheel(0, 0) = joint_commands_[i];
    else if (name == "left_wheel") left_wheel(0, 0) = joint_commands_[i];

    else if (name == "torso_0") torso[0] = joint_commands_[i];
    else if (name == "torso_1") torso[1] = joint_commands_[i];
    else if (name == "torso_2") torso[2] = joint_commands_[i];
    else if (name == "torso_3") torso[3] = joint_commands_[i];
    else if (name == "torso_4") torso[4] = joint_commands_[i];
    else if (name == "torso_5") torso[5] = joint_commands_[i];

    else if (name == "right_arm_0") right_arm[0] = joint_commands_[i];
    else if (name == "right_arm_1") right_arm[1] = joint_commands_[i];
    else if (name == "right_arm_2") right_arm[2] = joint_commands_[i];
    else if (name == "right_arm_3") right_arm[3] = joint_commands_[i];
    else if (name == "right_arm_4") right_arm[4] = joint_commands_[i];
    else if (name == "right_arm_5") right_arm[5] = joint_commands_[i];
    else if (name == "right_arm_6") right_arm[6] = joint_commands_[i];

    else if (name == "left_arm_0") left_arm[0] = joint_commands_[i];
    else if (name == "left_arm_1") left_arm[1] = joint_commands_[i];
    else if (name == "left_arm_2") left_arm[2] = joint_commands_[i];
    else if (name == "left_arm_3") left_arm[3] = joint_commands_[i];
    else if (name == "left_arm_4") left_arm[4] = joint_commands_[i];
    else if (name == "left_arm_5") left_arm[5] = joint_commands_[i];
    else if (name == "left_arm_6") left_arm[6] = joint_commands_[i];

    else if (name == "head_0") head[0] = joint_commands_[i];
    else if (name == "head_1") head[1] = joint_commands_[i];
  }

  stream_->SendCommand(
    rb::RobotCommandBuilder().SetCommand(
        rb::ComponentBasedCommandBuilder()
        .SetBodyCommand(
            rb::BodyComponentBasedCommandBuilder()
                .SetRightArmCommand(rb::JointPositionCommandBuilder()
                  .SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(0.3))
                  .SetMinimumTime(0.03).SetPosition(right_arm))
                .SetLeftArmCommand(rb::JointPositionCommandBuilder()
                  .SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(0.3))
                  .SetMinimumTime(0.03).SetPosition(left_arm))
                .SetTorsoCommand(rb::JointPositionCommandBuilder()
                  .SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(0.3))
                  .SetMinimumTime(0.03).SetPosition(torso))
        )
                .SetHeadCommand(rb::JointPositionCommandBuilder()
                  .SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(0.3))
                  .SetMinimumTime(0.03).SetPosition(head))
    )
  );

  return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(rby1_hardware::RBY1HardwareInterface, hardware_interface::SystemInterface)