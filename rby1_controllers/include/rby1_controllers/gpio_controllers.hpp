#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rby1_controllers/compat.hpp"

namespace rby1_controllers
{

// Turns the SDK gravity compensation controller of one component on (or off via ~/set_enabled).
// Claims <gpio>/enable, e.g. gpio: right_arm_gravity_compensation.
class GravityCompensationController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string gpio_;
  std::atomic<bool> enabled_{true};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

// Mobile base velocity through the SDK SE2 velocity controller (claims base_se2/*), with
// odometry from the rby1_status gpio.
//   ~/cmd_vel (geometry_msgs/Twist), ~/cmd_vel_stamped (geometry_msgs/TwistStamped)
//   ~/odom (nav_msgs/Odometry) and /tf odom -> base (optional)
// Velocities fall back to zero after cmd_vel_timeout.
class BaseVelocityController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Velocity
  {
    double vx{0.0}, vy{0.0}, wz{0.0};
    int64_t stamp_ns{0};
  };

  std::string gpio_;
  std::string status_gpio_;
  bool publish_odom_{true};
  bool publish_tf_{true};
  std::string odom_frame_;
  std::string base_frame_;
  double timeout_{0.5};
  realtime_tools::RealtimeBuffer<Velocity> velocity_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_rt_;
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tf_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_rt_;
  std::vector<int> cmd_index_, state_index_;
  double last_x_{0.0}, last_y_{0.0}, last_yaw_{0.0};
  bool have_last_{false};
};

// Task-space targets for the SDK Cartesian, Cartesian impedance, impedance and optimal
// control modes. Claims the pose slots of one gpio, e.g. right_arm_cartesian.
//   ~/target_pose   geometry_msgs/PoseStamped  target 0
//   ~/target_poses  geometry_msgs/PoseArray    all targets in order
//   ~/joint_targets std_msgs/Float64MultiArray joint target positions (if the gpio has them)
//   ~/com_target    geometry_msgs/PointStamped center of mass target (optimal control)
// Poses in another frame than the target's reference link are transformed with tf.
// Nothing is commanded until the first target arrives (the component is held).
class TaskSpaceController : public controller_interface::ControllerInterface
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
  struct Targets
  {
    std::vector<double> values;  // in claimed interface order
  };

  bool transform(const geometry_msgs::msg::PoseStamped & in, size_t target, geometry_msgs::msg::Pose & out);
  void store_pose(size_t target, const geometry_msgs::msg::Pose & pose);

  std::string gpio_;
  std::vector<std::string> ref_links_;
  size_t joint_target_count_{0};
  bool com_{false};
  std::vector<std::string> names_;               // claimed interface names
  std::vector<std::pair<std::string, double>> constants_;  // optional fields written every cycle
  std::vector<int> index_;                       // loaned command interface per names_/constants_ entry
  std::mutex pending_mutex_;
  std::vector<double> pending_;                  // non-RT staging, same order as names_
  realtime_tools::RealtimeBuffer<std::shared_ptr<Targets>> targets_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr com_sub_;
};

}  // namespace rby1_controllers
