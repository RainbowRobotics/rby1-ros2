// Power, servo and tool flange services of the RB-Y1 driver.
// Compiled as part of rby1_ros2_driver.cpp, which includes this file: the driver is a
// class template instantiated once per robot model there, not here.
#include "rby1_ros2_driver.hpp"
#include <sstream>

namespace rby1_ros2 {

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::power_control(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {

  // Parse parameters string
  std::vector<std::string> param_list;
  std::stringstream ss(request->parameters);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token.erase(0, token.find_first_not_of(" \t"));
    token.erase(token.find_last_not_of(" \t") + 1);
    if (!token.empty())
      param_list.push_back(token);
  }
  if (param_list.empty())
    param_list.push_back("all");

  // The simulator has one power device, named by ".*"; the robot's are named (48v, 12v, ...)
  std::string power_list_str = "";
  {
    for (size_t i = 0; i < param_list.size(); i++) {
      if (params_.is_simulator() || param_list[i] == "all" || param_list[i] == ".*") {
        power_list_str = ".*";
        break;
      }
      power_list_str += param_list[i];
      if (param_list[i].find('v') == std::string::npos &&
          param_list[i] != ".*") {
        power_list_str += "v";
      }
      if (i != param_list.size() - 1) {
        power_list_str += "|";
      }
    }
  }

  if (request->state) {
    if (robot_->IsPowerOn(power_list_str)) {
      RCLCPP_INFO(this->get_logger(), "Power is already ON [%s], skipping.",
                  power_list_str.c_str());
      response->success = true;
      response->message = "Power already ON";
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Power ON [%s]...", power_list_str.c_str());
    if (!robot_->PowerOn(power_list_str)) {
      response->success = false;
      response->message = "Failed to power on";
      return;
    }
  } else {
    // Check control manager state and disable it if enabled before powering off
    const auto &cm_state = robot_->GetControlManagerState();
    if (cm_state.state == rb::ControlManagerState::State::kEnabled) {
      RCLCPP_INFO(this->get_logger(),
                  "Control Manager is enabled. Disabling Control Manager first "
                  "before powering off...");
      if (!robot_->DisableControlManager()) {
        RCLCPP_WARN(this->get_logger(), "Failed to disable Control Manager, "
                                        "proceeding with power off anyway.");
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Control Manager successfully disabled.");
      }
    }

    RCLCPP_INFO(this->get_logger(), "Power OFF [%s]...",
                power_list_str.c_str());
    {
      std::lock_guard<std::mutex> stream_lock(stream_mutex_);
      close_stream_channels(CHANNEL_ALL, false);
    }
    if (!robot_->PowerOff(power_list_str)) {
      response->success = false;
      response->message = "Failed to power off";
      return;
    }
  }
  response->success = true;
  response->message = "Power control success";
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::servo_control(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
  // Parse parameters string
  std::vector<std::string> param_list;
  std::stringstream ss(request->parameters);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token.erase(0, token.find_first_not_of(" \t"));
    token.erase(token.find_last_not_of(" \t") + 1);
    if (!token.empty())
      param_list.push_back(token);
  }
  if (param_list.empty())
    param_list.push_back("all");

  std::string servo_list_str = "";
  for (size_t i = 0; i < param_list.size(); i++) {
    std::string name = param_list[i];
    if (name == "right")
      servo_list_str += "^right_arm_.*";
    else if (name == "left")
      servo_list_str += "^left_arm_.*";
    else if (name == "head")
      servo_list_str += "^head_.*";
    else if (name == "torso")
      servo_list_str += "^torso_.*";
    else if (name == "all") {
      servo_list_str = ".*";
      break;
    } else {
      servo_list_str += name;
    }
    if (i != param_list.size() - 1) {
      servo_list_str += "|";
    }
  }

  if (request->state) {
    bool already_on = false;
    try {
      if (robot_->IsServoOn(servo_list_str)) {
        already_on = true;
      }
    } catch (...) {
    }

    // Fallback check: If the Control Manager is already Enabled, the servos
    // must be ON.
    auto cm_state_val = robot_->GetControlManagerState().state;
    if (cm_state_val == rb::ControlManagerState::State::kEnabled) {
      already_on = true;
    }

    if (already_on) {
      RCLCPP_INFO(this->get_logger(),
                  "Servo is already ON [%s] (Control Manager: %d), skipping.",
                  servo_list_str.c_str(), static_cast<int>(cm_state_val));
      check_controll_manager();
      response->success = true;
      response->message = "Servo already ON";
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Servo ON [%s]...", servo_list_str.c_str());
    if (!robot_->ServoOn(servo_list_str)) {
      RCLCPP_ERROR(this->get_logger(), "SDK ServoOn failed for [%s]",
                   servo_list_str.c_str());
      response->success = false;
      response->message = "SDK_SERVO_ON_FAILED";
      return;
    }

    // Give SDK a bit of time to settle after servo on
    std::this_thread::sleep_for(kServoSettleTime);

    { // sync info
      std::lock_guard<std::mutex> lock(mutex_);
      info_ = robot_->GetRobotInfo();
    }

    RCLCPP_INFO(this->get_logger(),
                "Checking Control Manager after Servo ON...");
    if (!check_controll_manager()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Control Manager check failed after Servo ON.");
      response->success = false;
      response->message =
          robot_->IsPowerOn(params_.is_simulator() ? ".*" : "48v")
              ? "Control Manager failed to enable (Power is ON)"
              : "Control Manager failed: Robot power is OFF";
      return;
    }
    response->success = true;
    response->message = "Servo control success";
  } else {
    RCLCPP_INFO(this->get_logger(), "Servo OFF [%s]...",
                servo_list_str.c_str());
    stop_control_and_disable();
    if (!robot_->ServoOff(servo_list_str)) {
      RCLCPP_ERROR(this->get_logger(), "SDK ServoOff failed for [%s]",
                   servo_list_str.c_str());
      response->success = false;
      response->message = "SDK ServoOff call failed";
      return;
    }
    response->success = true;
    response->message = "Servo control success";
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::tool_flange_control(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
  // Parse parameters string
  int tool_flange_vol = 0;
  if (request->state) {
    if (request->parameters == "12" || request->parameters == "12v") {
      tool_flange_vol = 12;
    } else if (request->parameters == "24" || request->parameters == "24v") {
      tool_flange_vol = 24;
    } else {
      RCLCPP_INFO(this->get_logger(), "Invalid parameters: %s",
                  request->parameters.c_str());
      response->success = false;
      response->message =
          "Invalid parameters (must be '12', '12v', '24', or '24v')";
      return;
    }
    RCLCPP_INFO(this->get_logger(), "set tool flange output voltage [%d]",
                tool_flange_vol);
    bool right_success =
        robot_->SetToolFlangeOutputVoltage("right", tool_flange_vol);
    bool left_success =
        robot_->SetToolFlangeOutputVoltage("left", tool_flange_vol);
    if (!right_success || !left_success) {
      response->success = false;
      response->message = "Failed to set tool flange output voltage";
      return;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "turn off tool flange");
    bool right_success = robot_->SetToolFlangeOutputVoltage("right", 0);
    bool left_success = robot_->SetToolFlangeOutputVoltage("left", 0);
    if (!right_success || !left_success) {
      response->success = false;
      response->message = "Failed to turn off tool flange";
      return;
    }
  }
  response->success = true;
  response->message = "Tool flange control success";
}

}  // namespace rby1_ros2
