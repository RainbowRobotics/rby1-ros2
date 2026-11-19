// Control manager, control rights and the robot state loop of the RB-Y1 driver.
// Compiled as part of rby1_ros2_driver.cpp, which includes this file: the driver is a
// class template instantiated once per robot model there, not here.
#include "rby1_ros2_driver.hpp"

namespace rby1_ros2 {

// The robot refuses to disable while a control is still running, and a cancel ends on the
// robot asynchronously; so: cancel, drop the streams, wait for idle (measured well under a
// second), then disable.
template <typename ModelType>
bool RBY1_ROS2_DRIVER<ModelType>::stop_control_and_disable() {
  try {
    robot_->CancelControl();
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "CancelControl: %s", e.what());
  }
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    close_stream_channels(CHANNEL_ALL, false);
  }
  for (int i = 0; i < kControlIdleTries; ++i) {
    if (robot_->GetControlManagerState().control_state ==
        rb::ControlManagerState::ControlState::kIdle)
      break;
    std::this_thread::sleep_for(kControlIdleStep);
  }
  return robot_->DisableControlManager();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::control_manager_callback(
    const std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Request>
        request,
    std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Response> response) {

  try {
    if (request->command ==
        rby1_msgs::srv::ControlManagerCommand::Request::CMD_ENABLE) {
      RCLCPP_INFO(this->get_logger(),
                  "Received Control Manager ENABLE request");
      if (robot_->EnableControlManager(params_.control_manager_unlimited)) {
        response->success = true;
        response->message = params_.control_manager_unlimited
                                ? "Control Manager enabled successfully (unlimited mode)."
                                : "Control Manager enabled successfully.";
      } else {
        response->success = false;
        response->message = "Failed to enable Control Manager.";
      }
    } else if (request->command ==
               rby1_msgs::srv::ControlManagerCommand::Request::CMD_DISABLE) {
      RCLCPP_INFO(this->get_logger(),
                  "Received Control Manager DISABLE request");
      if (stop_control_and_disable()) {
        response->success = true;
        response->message = "Control Manager disabled successfully.";
      } else {
        response->success = false;
        response->message = "Failed to disable Control Manager.";
      }
    } else if (request->command ==
               rby1_msgs::srv::ControlManagerCommand::Request::CMD_RESET) {
      RCLCPP_INFO(this->get_logger(), "Received Control Manager RESET request");
      if (robot_->ResetFaultControlManager()) {
        {
          std::lock_guard<std::mutex> stream_lock(stream_mutex_);
          close_stream_channels(CHANNEL_ALL, false);
        }
        response->success = true;
        response->message = "Control Manager reset successfully.";
      } else {
        response->success = false;
        response->message = "Failed to reset Control Manager.";
      }
    } else if (request->command ==
               rby1_msgs::srv::ControlManagerCommand::Request::CMD_UNLIMIT) {
      RCLCPP_INFO(this->get_logger(),
                  "Received Control Manager UNLIMIT request");
      if (robot_->EnableControlManager(true)) {
        response->success = true;
        response->message =
            "Control Manager enabled in unlimited mode successfully.";
      } else {
        response->success = false;
        response->message =
            "Failed to enable Control Manager in unlimited mode.";
      }
    } else {
      response->success = false;
      response->message = "Invalid command type provided.";
      RCLCPP_ERROR(this->get_logger(),
                   "Control Manager command: Invalid command type %d",
                   request->command);
    }
  } catch (const std::exception &e) {
    response->success = false;
    response->message = std::string("Exception occurred: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "Control Manager exception: %s", e.what());
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::hardware_control_callback(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request->state) {
    hardware_control_active_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Hardware control claimed. Rejecting/blocking non-hardware "
                "command sources.");
    response->success = true;
    response->message = "Hardware control activated.";
  } else {
    hardware_control_active_ = false;
    deactivate_all_streams();
    RCLCPP_INFO(this->get_logger(), "Hardware control released.");
    response->success = true;
    response->message = "Hardware control deactivated.";
  }
}

template <typename ModelType>
bool RBY1_ROS2_DRIVER<ModelType>::check_controll_manager() {
  const auto &control_manager_state = robot_->GetControlManagerState();

  if (control_manager_state.state == rb::ControlManagerState::State::kEnabled) {
    return true;
  }

  if (control_manager_state.state ==
          rb::ControlManagerState::State::kMajorFault ||
      control_manager_state.state ==
          rb::ControlManagerState::State::kMinorFault) {
    RCLCPP_WARN(this->get_logger(),
                "Detected a %s fault in the Control Manager. Attempting "
                "automatic reset...",
                (control_manager_state.state ==
                         rb::ControlManagerState::State::kMajorFault
                     ? "Major"
                     : "Minor"));

    if (!robot_->ResetFaultControlManager()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to reset the fault in the Control Manager.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Fault reset successfully.");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Control Manager state is normal. No faults detected.");
  }

  RCLCPP_INFO(this->get_logger(), "Enabling Control Manager%s...",
              params_.control_manager_unlimited ? " (unlimited mode)" : "");
  if (!robot_->EnableControlManager(params_.control_manager_unlimited)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable the Control Manager.");
    return false;
  }

  // Wait for control ready to ensure SendCommand doesn't fail with kUnknown
  // immediately
  if (!robot_->WaitForControlReady(kControlReadyTimeoutMs)) {
    RCLCPP_WARN(this->get_logger(), "Control Manager enabled, but timed out "
                                    "waiting for Control Ready status.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Control Manager enabled and ready.");
  }

  return true;
}

}  // namespace rby1_ros2
