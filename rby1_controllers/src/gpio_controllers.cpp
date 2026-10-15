#include "rby1_controllers/gpio_controllers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rby1_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}

// ============================================================================================
// GravityCompensationController

CallbackReturn GravityCompensationController::on_init()
{
  auto_declare<std::string>("gpio", "");
  auto_declare<bool>("enabled", true);
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration GravityCompensationController::command_interface_configuration() const
{
  return {interface_configuration_type::INDIVIDUAL, {gpio_ + "/enable"}};
}

InterfaceConfiguration GravityCompensationController::state_interface_configuration() const
{
  return {interface_configuration_type::NONE, {}};
}

CallbackReturn GravityCompensationController::on_configure(const rclcpp_lifecycle::State &)
{
  gpio_ = get_node()->get_parameter("gpio").as_string();
  if (gpio_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'gpio' is empty (e.g. right_arm_gravity_compensation)");
    return CallbackReturn::ERROR;
  }
  service_ = get_node()->create_service<std_srvs::srv::SetBool>(
    "~/set_enabled",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      enabled_ = request->data;
      response->success = true;
      response->message = request->data ? "gravity compensation on" : "gravity compensation off";
    });
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationController::on_activate(const rclcpp_lifecycle::State &)
{
  enabled_ = get_node()->get_parameter("enabled").as_bool();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  write_value(command_interfaces_[0], enabled_ ? 1.0 : 0.0);
  return controller_interface::return_type::OK;
}

// ============================================================================================
// BaseVelocityController

CallbackReturn BaseVelocityController::on_init()
{
  auto_declare<std::string>("gpio", "base_se2");
  auto_declare<std::string>("status_gpio", "rby1_status");
  auto_declare<double>("cmd_vel_timeout", 0.5);
  auto_declare<bool>("publish_odom", true);
  auto_declare<bool>("publish_tf", true);
  auto_declare<std::string>("odom_frame", "odom");
  auto_declare<std::string>("base_frame", "base");
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration BaseVelocityController::command_interface_configuration() const
{
  return {interface_configuration_type::INDIVIDUAL,
    {gpio_ + "/linear_x", gpio_ + "/linear_y", gpio_ + "/angular_z"}};
}

InterfaceConfiguration BaseVelocityController::state_interface_configuration() const
{
  if (!publish_odom_ || status_gpio_.empty()) {
    return {interface_configuration_type::NONE, {}};
  }
  return {interface_configuration_type::INDIVIDUAL,
    {status_gpio_ + "/odometry_x", status_gpio_ + "/odometry_y", status_gpio_ + "/odometry_yaw"}};
}

CallbackReturn BaseVelocityController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  gpio_ = node->get_parameter("gpio").as_string();
  status_gpio_ = node->get_parameter("status_gpio").as_string();
  timeout_ = node->get_parameter("cmd_vel_timeout").as_double();
  publish_odom_ = node->get_parameter("publish_odom").as_bool();
  publish_tf_ = node->get_parameter("publish_tf").as_bool();
  odom_frame_ = node->get_parameter("odom_frame").as_string();
  base_frame_ = node->get_parameter("base_frame").as_string();

  auto store = [this](double vx, double vy, double wz) {
      Velocity v;
      v.vx = vx;
      v.vy = vy;
      v.wz = wz;
      v.stamp_ns = get_node()->get_clock()->now().nanoseconds();
      velocity_.writeFromNonRT(v);
    };
  twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [store](const geometry_msgs::msg::Twist::SharedPtr msg) {store(msg->linear.x, msg->linear.y, msg->angular.z);});
  twist_stamped_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/cmd_vel_stamped", rclcpp::SystemDefaultsQoS(),
    [store](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      store(msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);
    });
  if (publish_odom_) {
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS());
    odom_rt_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);
    if (publish_tf_) {
      tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
      tf_rt_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_pub_);
      tf_rt_->msg_.transforms.resize(1);
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn BaseVelocityController::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_index_.clear();
  for (const auto * n : {"/linear_x", "/linear_y", "/angular_z"}) {
    cmd_index_.push_back(find_interface(command_interfaces_, gpio_ + n));
  }
  state_index_.clear();
  if (publish_odom_ && !status_gpio_.empty()) {
    for (const auto * n : {"/odometry_x", "/odometry_y", "/odometry_yaw"}) {
      state_index_.push_back(find_interface(state_interfaces_, status_gpio_ + n));
    }
  }
  if (std::find(cmd_index_.begin(), cmd_index_.end(), -1) != cmd_index_.end() ||
    std::find(state_index_.begin(), state_index_.end(), -1) != state_index_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing base_se2 or status interfaces");
    return CallbackReturn::ERROR;
  }
  velocity_.writeFromNonRT(Velocity{});
  have_last_ = false;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BaseVelocityController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  Velocity v = *velocity_.readFromRT();
  if (v.stamp_ns == 0 || (timeout_ > 0.0 && (time.nanoseconds() - v.stamp_ns) * 1e-9 > timeout_)) {
    v = Velocity{};
  }
  write_value(command_interfaces_[cmd_index_[0]], v.vx);
  write_value(command_interfaces_[cmd_index_[1]], v.vy);
  write_value(command_interfaces_[cmd_index_[2]], v.wz);

  if (state_index_.size() != 3) {
    return controller_interface::return_type::OK;
  }
  const double x = read_value(state_interfaces_[state_index_[0]]);
  const double y = read_value(state_interfaces_[state_index_[1]]);
  const double yaw = read_value(state_interfaces_[state_index_[2]]);
  const double dt = period.seconds();
  double vx = 0.0, vy = 0.0, wz = 0.0;
  if (have_last_ && dt > 0.0) {
    const double dx = x - last_x_;
    const double dy = y - last_y_;
    vx = (std::cos(yaw) * dx + std::sin(yaw) * dy) / dt;
    vy = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / dt;
    wz = std::remainder(yaw - last_yaw_, 2.0 * M_PI) / dt;
  }
  last_x_ = x;
  last_y_ = y;
  last_yaw_ = yaw;
  have_last_ = true;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  if (odom_rt_ && odom_rt_->trylock()) {
    auto & m = odom_rt_->msg_;
    m.header.stamp = time;
    m.header.frame_id = odom_frame_;
    m.child_frame_id = base_frame_;
    m.pose.pose.position.x = x;
    m.pose.pose.position.y = y;
    m.pose.pose.orientation = tf2::toMsg(q);
    m.twist.twist.linear.x = vx;
    m.twist.twist.linear.y = vy;
    m.twist.twist.angular.z = wz;
    odom_rt_->unlockAndPublish();
  }
  if (tf_rt_ && tf_rt_->trylock()) {
    auto & t = tf_rt_->msg_.transforms[0];
    t.header.stamp = time;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation = tf2::toMsg(q);
    tf_rt_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

// ============================================================================================
// TaskSpaceController

CallbackReturn TaskSpaceController::on_init()
{
  auto_declare<std::string>("gpio", "");
  auto_declare<std::vector<std::string>>("ref_links", {});  // one per target
  auto_declare<int>("joint_target_count", 0);
  auto_declare<bool>("center_of_mass", false);
  // Optional per-target fields written every cycle (claimed only if > 0).
  for (const auto * f : {"linear_velocity_limit", "angular_velocity_limit", "acceleration_limit_scaling",
      "linear_acceleration_limit", "angular_acceleration_limit", "translation_weight", "rotation_weight"})
  {
    auto_declare<double>(f, 0.0);
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration TaskSpaceController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & n : names_) {
    config.names.push_back(gpio_ + "/" + n);
  }
  for (const auto & c : constants_) {
    config.names.push_back(gpio_ + "/" + c.first);
  }
  return config;
}

InterfaceConfiguration TaskSpaceController::state_interface_configuration() const
{
  return {interface_configuration_type::NONE, {}};
}

CallbackReturn TaskSpaceController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  gpio_ = node->get_parameter("gpio").as_string();
  ref_links_ = node->get_parameter("ref_links").as_string_array();
  joint_target_count_ = static_cast<size_t>(std::max<int64_t>(0, node->get_parameter("joint_target_count").as_int()));
  com_ = node->get_parameter("center_of_mass").as_bool();
  if (gpio_.empty() || ref_links_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "'gpio' and 'ref_links' (one per target) are required");
    return CallbackReturn::ERROR;
  }

  names_.clear();
  constants_.clear();
  for (size_t i = 0; i < ref_links_.size(); ++i) {
    for (const auto * f : {"x", "y", "z", "qx", "qy", "qz", "qw"}) {
      names_.push_back("t" + std::to_string(i) + "." + f);
    }
    for (const auto * f : {"linear_velocity_limit", "angular_velocity_limit", "acceleration_limit_scaling",
        "linear_acceleration_limit", "angular_acceleration_limit", "translation_weight", "rotation_weight"})
    {
      const double v = node->get_parameter(f).as_double();
      if (v > 0.0) {
        constants_.emplace_back("t" + std::to_string(i) + "." + f, v);
      }
    }
  }
  for (size_t k = 0; k < joint_target_count_; ++k) {
    names_.push_back("j" + std::to_string(k) + ".position");
  }
  if (com_) {
    names_.insert(names_.end(), {"com.x", "com.y", "com.z"});
  }
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_.assign(names_.size(), kNaN);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto publish_pending = [this]() {
      auto t = std::make_shared<Targets>();
      std::lock_guard<std::mutex> lock(pending_mutex_);
      t->values = pending_;
      targets_.writeFromNonRT(t);
    };

  pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", rclcpp::SystemDefaultsQoS(),
    [this, publish_pending](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      geometry_msgs::msg::Pose pose;
      if (transform(*msg, 0, pose)) {
        store_pose(0, pose);
        publish_pending();
      }
    });
  poses_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "~/target_poses", rclcpp::SystemDefaultsQoS(),
    [this, publish_pending](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      if (msg->poses.size() != ref_links_.size()) {
        RCLCPP_WARN(get_node()->get_logger(), "target_poses: expected %zu poses", ref_links_.size());
        return;
      }
      for (size_t i = 0; i < msg->poses.size(); ++i) {
        geometry_msgs::msg::PoseStamped in;
        in.header = msg->header;
        in.pose = msg->poses[i];
        geometry_msgs::msg::Pose pose;
        if (!transform(in, i, pose)) {
          return;
        }
        store_pose(i, pose);
      }
      publish_pending();
    });
  if (joint_target_count_ > 0) {
    joints_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/joint_targets", rclcpp::SystemDefaultsQoS(),
      [this, publish_pending](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != joint_target_count_) {
          RCLCPP_WARN(get_node()->get_logger(), "joint_targets: expected %zu values", joint_target_count_);
          return;
        }
        {
          std::lock_guard<std::mutex> lock(pending_mutex_);
          const size_t offset = ref_links_.size() * 7;
          for (size_t k = 0; k < joint_target_count_; ++k) {
            pending_[offset + k] = msg->data[k];
          }
        }
        publish_pending();
      });
  }
  if (com_) {
    com_sub_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
      "~/com_target", rclcpp::SystemDefaultsQoS(),
      [this, publish_pending](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(pending_mutex_);
          const size_t offset = ref_links_.size() * 7 + joint_target_count_;
          pending_[offset] = msg->point.x;
          pending_[offset + 1] = msg->point.y;
          pending_[offset + 2] = msg->point.z;
        }
        publish_pending();
      });
  }
  return CallbackReturn::SUCCESS;
}

bool TaskSpaceController::transform(
  const geometry_msgs::msg::PoseStamped & in, size_t target, geometry_msgs::msg::Pose & out)
{
  const auto & ref = ref_links_[target];
  if (in.header.frame_id.empty() || in.header.frame_id == ref) {
    out = in.pose;
    return true;
  }
  try {
    geometry_msgs::msg::PoseStamped stamped = in;
    stamped.header.stamp = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
    out = tf_buffer_->transform(stamped, ref, tf2::durationFromSec(0.1)).pose;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Cannot transform target from '%s' to '%s': %s", in.header.frame_id.c_str(), ref.c_str(), e.what());
    return false;
  }
}

void TaskSpaceController::store_pose(size_t target, const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  double * p = &pending_[target * 7];
  p[0] = pose.position.x;
  p[1] = pose.position.y;
  p[2] = pose.position.z;
  p[3] = pose.orientation.x;
  p[4] = pose.orientation.y;
  p[5] = pose.orientation.z;
  p[6] = pose.orientation.w;
}

CallbackReturn TaskSpaceController::on_activate(const rclcpp_lifecycle::State &)
{
  index_.clear();
  for (const auto & n : names_) {
    index_.push_back(find_interface(command_interfaces_, gpio_ + "/" + n));
  }
  for (const auto & c : constants_) {
    index_.push_back(find_interface(command_interfaces_, gpio_ + "/" + c.first));
  }
  if (std::find(index_.begin(), index_.end(), -1) != index_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing command interfaces on gpio '%s'", gpio_.c_str());
    return CallbackReturn::ERROR;
  }
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    std::fill(pending_.begin(), pending_.end(), kNaN);
  }
  targets_.writeFromNonRT(nullptr);
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  targets_.writeFromNonRT(nullptr);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto targets = *targets_.readFromRT();
  for (size_t i = 0; i < names_.size(); ++i) {
    // NaN until the first target arrives: the hardware holds the component meanwhile.
    write_value(command_interfaces_[index_[i]], targets ? targets->values[i] : kNaN);
  }
  for (size_t k = 0; k < constants_.size(); ++k) {
    write_value(command_interfaces_[index_[names_.size() + k]], constants_[k].second);
  }
  return controller_interface::return_type::OK;
}

}  // namespace rby1_controllers

PLUGINLIB_EXPORT_CLASS(rby1_controllers::GravityCompensationController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(rby1_controllers::BaseVelocityController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(rby1_controllers::TaskSpaceController, controller_interface::ControllerInterface)
