#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <Eigen/Dense>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rby1_msgs/srv/state_on_off.hpp"
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include "rby1-sdk/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_command_builder.h"

// Suppress compiler warnings from optimal_control.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include "rby1-sdk/math/optimal_control.h"
#pragma GCC diagnostic pop

namespace rby1_hardware {

class RBY1RobotWrapper {
public:
  virtual ~RBY1RobotWrapper() = default;
  virtual bool connect(const std::string& address) = 0;
  virtual void disconnect() = 0;
  virtual bool is_connected() const = 0;
  
  virtual bool power_on(const std::string& dev) = 0;
  virtual bool servo_on(const std::string& dev) = 0;
  virtual bool is_servo_on(const std::string& dev) const = 0;
  virtual bool enable_control_manager() = 0;
  virtual bool wait_for_control_ready(long timeout_ms) = 0;
  virtual bool disable_control_manager() = 0;
  virtual bool cancel_control() = 0;
  
  virtual rb::RobotInfo get_robot_info() const = 0;
  virtual void get_joint_states(std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& torques) = 0;
  
  virtual bool init_stream() = 0;
  virtual void send_stream_command(const std::vector<double>& target_positions, double vel_limit = -1.0, double acc_limit = -1.0) = 0;
  virtual void send_mobility_command(double vx, double vy, double wz, double minimum_time) = 0;
  virtual void close_stream() = 0;
  
  virtual std::optional<std::string> get_predicted_collision_reason(const std::vector<double>& target_q, double threshold) = 0;
  virtual int get_dof() const = 0;
};

template <typename ModelType>
class RBY1RobotWrapperImpl : public RBY1RobotWrapper {
private:
  std::shared_ptr<rb::Robot<ModelType>> robot_;
  rb::RobotInfo info_;
  std::unique_ptr<rb::RobotCommandStreamHandler<ModelType>> upper_body_stream_handler_;
  std::unique_ptr<rb::RobotCommandStreamHandler<ModelType>> mobility_stream_handler_;
  
  // Dynamics for predictive collision check
  std::shared_ptr<rb::dyn::Robot<ModelType::kRobotDOF>> dynamics_;
  std::shared_ptr<rb::dyn::State<ModelType::kRobotDOF>> dyn_state_;
  std::vector<std::string> dyn_link_names_;

public:
  RBY1RobotWrapperImpl() = default;
  virtual ~RBY1RobotWrapperImpl() {
    disconnect();
  }

  bool connect(const std::string& address) override {
    try {
      robot_ = rb::Robot<ModelType>::Create(address);
      if (robot_->Connect()) {
        info_ = robot_->GetRobotInfo();
        dynamics_ = robot_->GetDynamics();
        dyn_link_names_ = dynamics_->GetLinkNames();
        dyn_state_ = dynamics_->template MakeState(dyn_link_names_, dynamics_->GetJointNames());
        return true;
      }
    } catch (...) {}
    return false;
  }

  void disconnect() override {
    if (robot_) {
      close_stream();
      robot_->Disconnect();
      robot_.reset();
    }
  }

  bool is_connected() const override {
    return robot_ && robot_->IsConnected();
  }

  bool power_on(const std::string& dev) override {
    return robot_ && robot_->PowerOn(dev);
  }

  bool servo_on(const std::string& dev) override {
    return robot_ && robot_->ServoOn(dev);
  }

  bool is_servo_on(const std::string& dev) const override {
    return robot_ && robot_->IsServoOn(dev);
  }

  bool enable_control_manager() override {
    return robot_ && robot_->EnableControlManager();
  }

  bool wait_for_control_ready(long timeout_ms) override {
    return robot_ && robot_->WaitForControlReady(timeout_ms);
  }

  bool disable_control_manager() override {
    return robot_ && robot_->DisableControlManager();
  }

  bool cancel_control() override {
    return robot_ && robot_->CancelControl();
  }

  rb::RobotInfo get_robot_info() const override {
    return info_;
  }

  int get_dof() const override {
    return ModelType::kRobotDOF;
  }

  void get_joint_states(std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& torques) override {
    if (!robot_) return;
    auto state = robot_->GetState();
    positions.assign(state.position.data(), state.position.data() + state.position.size());
    velocities.assign(state.velocity.data(), state.velocity.data() + state.velocity.size());
    torques.assign(state.torque.data(), state.torque.data() + state.torque.size());
  }

  bool init_stream() override {
    if (!robot_) return false;
    close_stream();
    upper_body_stream_handler_ = robot_->CreateCommandStream(10);
    mobility_stream_handler_ = robot_->CreateCommandStream(10);
    return upper_body_stream_handler_ != nullptr && mobility_stream_handler_ != nullptr;
  }

  void send_stream_command(const std::vector<double>& target_positions, double vel_limit = -1.0, double acc_limit = -1.0) override {
    if (!upper_body_stream_handler_ || target_positions.size() < ModelType::kRobotDOF) return;
    
    rb::TorsoCommandBuilder torso_builder;
    if (!info_.torso_joint_idx.empty()) {
      Eigen::VectorXd torso_q(info_.torso_joint_idx.size());
      for (size_t i = 0; i < info_.torso_joint_idx.size(); ++i) {
        torso_q[i] = target_positions[info_.torso_joint_idx[i]];
      }
      rb::JointPositionCommandBuilder cmd_builder;
      cmd_builder.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
                 .SetPosition(torso_q)
                 .SetMinimumTime(0.01);
      if (vel_limit > 0.0) {
        cmd_builder.SetVelocityLimit(Eigen::VectorXd::Constant(torso_q.size(), vel_limit));
      }
      if (acc_limit > 0.0) {
        cmd_builder.SetAccelerationLimit(Eigen::VectorXd::Constant(torso_q.size(), acc_limit));
      }
      torso_builder.SetCommand(cmd_builder);
    }
    
    rb::ArmCommandBuilder right_arm_builder;
    if (!info_.right_arm_joint_idx.empty()) {
      Eigen::VectorXd right_q(info_.right_arm_joint_idx.size());
      for (size_t i = 0; i < info_.right_arm_joint_idx.size(); ++i) {
        right_q[i] = target_positions[info_.right_arm_joint_idx[i]];
      }
      rb::JointPositionCommandBuilder cmd_builder;
      cmd_builder.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
                 .SetPosition(right_q)
                 .SetMinimumTime(0.01);
      if (vel_limit > 0.0) {
        cmd_builder.SetVelocityLimit(Eigen::VectorXd::Constant(right_q.size(), vel_limit));
      }
      if (acc_limit > 0.0) {
        cmd_builder.SetAccelerationLimit(Eigen::VectorXd::Constant(right_q.size(), acc_limit));
      }
      right_arm_builder.SetCommand(cmd_builder);
    }
    
    rb::ArmCommandBuilder left_arm_builder;
    if (!info_.left_arm_joint_idx.empty()) {
      Eigen::VectorXd left_q(info_.left_arm_joint_idx.size());
      for (size_t i = 0; i < info_.left_arm_joint_idx.size(); ++i) {
        left_q[i] = target_positions[info_.left_arm_joint_idx[i]];
      }
      rb::JointPositionCommandBuilder cmd_builder;
      cmd_builder.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
                 .SetPosition(left_q)
                 .SetMinimumTime(0.01);
      if (vel_limit > 0.0) {
        cmd_builder.SetVelocityLimit(Eigen::VectorXd::Constant(left_q.size(), vel_limit));
      }
      if (acc_limit > 0.0) {
        cmd_builder.SetAccelerationLimit(Eigen::VectorXd::Constant(left_q.size(), acc_limit));
      }
      left_arm_builder.SetCommand(cmd_builder);
    }
    
    rb::HeadCommandBuilder head_builder;
    if (!info_.head_joint_idx.empty()) {
      Eigen::VectorXd head_q(info_.head_joint_idx.size());
      for (size_t i = 0; i < info_.head_joint_idx.size(); ++i) {
        head_q[i] = target_positions[info_.head_joint_idx[i]];
      }
      rb::JointPositionCommandBuilder cmd_builder;
      cmd_builder.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
                 .SetPosition(head_q)
                 .SetMinimumTime(0.01);
      if (vel_limit > 0.0) {
        cmd_builder.SetVelocityLimit(Eigen::VectorXd::Constant(head_q.size(), vel_limit));
      }
      if (acc_limit > 0.0) {
        cmd_builder.SetAccelerationLimit(Eigen::VectorXd::Constant(head_q.size(), acc_limit));
      }
      head_builder.SetCommand(cmd_builder);
    }
    
    rb::BodyComponentBasedCommandBuilder body_comp_builder;
    body_comp_builder.SetTorsoCommand(torso_builder);
    body_comp_builder.SetRightArmCommand(right_arm_builder);
    body_comp_builder.SetLeftArmCommand(left_arm_builder);
    
    rb::ComponentBasedCommandBuilder comp_builder;
    comp_builder.SetBodyCommand(rb::BodyCommandBuilder(std::move(body_comp_builder)));
    comp_builder.SetHeadCommand(head_builder);
    
    upper_body_stream_handler_->SendCommand(rb::RobotCommandBuilder().SetCommand(comp_builder));
  }

  void send_mobility_command(double vx, double vy, double wz, double minimum_time) override {
    if (!mobility_stream_handler_) return;
    
    Eigen::Vector2d linear_vel(vx, vy);
    Eigen::Vector2d acceleration_limit(0.5, 0.5);
    
    rb::SE2VelocityCommandBuilder se2_cmd;
    se2_cmd.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(1.0));
    se2_cmd.SetMinimumTime(minimum_time);
    se2_cmd.SetVelocity(linear_vel, wz);
    se2_cmd.SetAccelerationLimit(acceleration_limit, 0.5);
    
    rb::MobilityCommandBuilder mobility_cmd;
    mobility_cmd.SetCommand(se2_cmd);
    
    rb::ComponentBasedCommandBuilder comp_builder;
    comp_builder.SetMobilityCommand(mobility_cmd);
    
    mobility_stream_handler_->SendCommand(rb::RobotCommandBuilder().SetCommand(comp_builder));
  }

  void close_stream() override {
    if (upper_body_stream_handler_) {
      try { upper_body_stream_handler_->Cancel(); } catch (...) {}
      upper_body_stream_handler_.reset();
    }
    if (mobility_stream_handler_) {
      try { mobility_stream_handler_->Cancel(); } catch (...) {}
      mobility_stream_handler_.reset();
    }
  }

  std::optional<std::string> get_predicted_collision_reason(const std::vector<double>& target_q, double threshold) override {
    if (!dynamics_ || target_q.size() < ModelType::kRobotDOF) return std::nullopt;
    
    Eigen::VectorXd q(ModelType::kRobotDOF);
    for (size_t i = 0; i < ModelType::kRobotDOF; ++i) {
      q[i] = target_q[i];
    }
    
    dyn_state_->q = q;
    dynamics_->ComputeForwardKinematics(dyn_state_);
    
    auto collisions = dynamics_->DetectCollisionsOrNearestLinks(dyn_state_);
    if (!collisions.empty()) {
      double min_dist = 1e9;
      std::string link1, link2;
      for (const auto& col : collisions) {
        if (col.distance < min_dist) {
          min_dist = col.distance;
          link1 = col.link1;
          link2 = col.link2;
        }
      }
      if (min_dist < threshold) {
        std::stringstream ss;
        ss << "Collision predicted between [" << link1 << "] and [" << link2 
           << "] (distance: " << min_dist << " m < threshold: " << threshold << " m)";
        return ss.str();
      }
    }
    return std::nullopt;
  }
};

struct ControlStreamCommand {
  std::vector<double> joint_positions;
  double vel_limit{-1.0};
  double acc_limit{-1.0};

  bool has_twist{false};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};

  bool cancel_requested{false};
};

class RBY1SystemHardware : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
 
private:
  std::unique_ptr<RBY1RobotWrapper> robot_;
  std::string robot_ip_;
  std::string model_type_;
  std::string driver_ns_prefix_;
  double collision_threshold_{0.01};
  bool collision_check_enable_{false};
  
  // Joint states and commands
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  
  // Mapping from URDF joint index to SDK index
  std::vector<unsigned int> joint_name_to_sdk_index_;

  // ROS 2 node, subscribers, and service client
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Client<rby1_msgs::srv::StateOnOff>::SharedPtr hardware_control_client_;
  rclcpp::Client<rby1_msgs::srv::StateOnOff>::SharedPtr power_control_client_;
  rclcpp::Client<rby1_msgs::srv::StateOnOff>::SharedPtr servo_control_client_;

  double velocity_limit_{4.712388};
  double acceleration_limit_{1.0};

  // Joint index classification by prefix
  std::vector<unsigned int> torso_joint_indices_;
  std::vector<unsigned int> right_arm_joint_indices_;
  std::vector<unsigned int> left_arm_joint_indices_;
  std::vector<unsigned int> head_joint_indices_;

  // Cached states for read/write thread-safe exchange
  std::mutex state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  geometry_msgs::msg::Twist::SharedPtr latest_twist_;
  rclcpp::Time cmd_vel_recv_time_;

  // Single IO Worker Thread and non-blocking command exchange buffer
  std::thread io_worker_thread_;
  std::atomic<bool> worker_running_{false};
  std::atomic<bool> is_stream_alive_{true};

  std::mutex cmd_exchange_mutex_;
  std::condition_variable cmd_exchange_cv_;
  ControlStreamCommand pending_command_;
  bool has_pending_command_{false};

  void io_worker_loop();
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

} // namespace rby1_hardware
