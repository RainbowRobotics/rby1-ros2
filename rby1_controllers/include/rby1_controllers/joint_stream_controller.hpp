#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rby1_controllers/compat.hpp"

namespace rby1_controllers
{

// Streams joint targets received on a topic to the RB-Y1 joint position or joint impedance
// controller of one component.
//
// mode: position  -> claims <joint>/position                    (SDK JointPositionCommand)
// mode: impedance -> claims <joint>/{position,stiffness,damping_ratio[,torque_limit]}
//                                                               (SDK JointImpedanceControlCommand)
// mode: realtime  -> claims <joint>/{position,effort,feedback_gain} (SDK real-time control; the
//                    JointState effort field is the feedforward torque, feedback_gain a parameter)
//
// Inputs (latest message wins):
//   ~/joint_commands  sensor_msgs/JointState  positions by joint name; messages missing one of
//                                             the controller's joints are dropped. This is the
//                                             interface of UR's impedance_controller / Flexiv's
//                                             streaming_position_controller used by Isaac ROS.
//   ~/commands        std_msgs/Float64MultiArray  positions in `joints` order
//
// The target starts at the measured position on activation; stiffness, damping_ratio and
// torque_limit are parameters that can be changed at runtime.
class JointStreamController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Gains
  {
    std::vector<double> stiffness;
    std::vector<double> torque_limit;
    double damping_ratio{1.0};
  };

  bool load_gains(Gains & gains) const;
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & parameters);

  std::vector<std::string> joints_;
  bool impedance_{false};
  bool realtime_{false};
  std::atomic<double> feedback_gain_{10.0};
  realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<double>>> effort_buffer_;
  std::vector<double> effort_;
  std::vector<int> effort_cmd_, gain_cmd_;
  bool use_torque_limit_{false};
  double command_timeout_{0.0};

  realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<double>>> target_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Gains>> gains_buffer_;
  std::atomic<int64_t> last_command_ns_{0};
  std::vector<double> target_;

  // loaned interface indices per joint
  std::vector<int> position_cmd_, stiffness_cmd_, damping_cmd_, torque_cmd_, position_state_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr array_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_handle_;
};

}  // namespace rby1_controllers
