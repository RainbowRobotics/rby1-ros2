#include "rby1_ros2_driver.hpp"
#include <iomanip>
#include <sstream>
// The driver is a class template, instantiated for each robot model at the end of this
// file; its other source files are compiled here, as part of this one.
#include "command_builder.cpp"
#include "control.cpp"
#include "control_stream.cpp"
#include "kinematics.cpp"
#include "power.cpp"
#include "state_manager.cpp"
namespace rby1_ros2 {
// using namespace rb;
template <typename ModelType>
RBY1_ROS2_DRIVER<ModelType>::RBY1_ROS2_DRIVER() : Node("rby1_ros2_driver") {
  // declare parameter from yaml
  init_parameter();
  try {
    robot_ = rb::Robot<ModelType>::Create(params_.robot_ip);
    if (robot_->Connect()) {
      robot_->SetParameter("joint_position_command.cutoff_frequency",
                           std::to_string(params_.robot.joint_position_command_cutoff_frequency));
      robot_->SetParameter("cartesian_command.cutoff_frequency",
                           std::to_string(params_.robot.cartesian_command_cutoff_frequency));
      robot_->SetParameter("default.acceleration_limit_scaling",
                           std::to_string(params_.robot.default_acceleration_limit_scaling));
      robot_->SetParameter("default.linear_acceleration_limit",
                           std::to_string(params_.robot.default_linear_acceleration_limit));
      robot_->SetParameter("default.velocity_limit_scaling",
                           std::to_string(params_.robot.default_velocity_limit_scaling));
      robot_->SetParameter("default.angular_acceleration_limit",
                           std::to_string(params_.robot.default_angular_acceleration_limit));
      // Fetch robot info once and cache it
      info_ = robot_->GetRobotInfo();

      RCLCPP_INFO(this->get_logger(),
                  "Robot Info: Model=%s, Version=%s, Compile-time DOF=%zu",
                  info_.robot_model_name.c_str(),
                  info_.robot_model_version.c_str(), ModelType::kRobotDOF);

      // Model check in driver constructor
      {
        std::string exp_model = params_.model;
        std::string conn_model = info_.robot_model_name;
        std::transform(exp_model.begin(), exp_model.end(), exp_model.begin(), ::tolower);
        std::transform(conn_model.begin(), conn_model.end(), conn_model.begin(), ::tolower);

        bool model_match = false;
        if (exp_model == "a" && (conn_model.find("a") != std::string::npos || conn_model.find("rby1a") != std::string::npos)) {
          model_match = true;
        } else if (exp_model == "m" && (conn_model.find("m") != std::string::npos || conn_model.find("rby1m") != std::string::npos)) {
          model_match = true;
        }

        if (!model_match) {
          RCLCPP_ERROR(this->get_logger(),
                       "\033[1;31m[MODEL MISMATCH] Configured model '%s' does not match connected robot '%s'!\033[0m",
                       params_.model.c_str(), info_.robot_model_name.c_str());
          throw std::runtime_error("Robot model mismatch: configured " + params_.model + ", connected " + info_.robot_model_name);
        }
      }
      RCLCPP_INFO(this->get_logger(),
                  "Joint counts: torso=%zu, right_arm=%zu, left_arm=%zu, "
                  "head=%zu, mobility=%zu",
                  info_.torso_joint_idx.size(),
                  info_.right_arm_joint_idx.size(),
                  info_.left_arm_joint_idx.size(), info_.head_joint_idx.size(),
                  info_.mobility_joint_idx.size());
      try {
        RCLCPP_INFO(this->get_logger(), "Loading Dynamics model...");
        dynamics_ = robot_->GetDynamics();
        RCLCPP_INFO(this->get_logger(),
                    "Dynamics model loaded. Creating DynState...");

        // Register ALL links from dynamics model
        dyn_link_names_ = dynamics_->GetLinkNames();

        dyn_state_ = dynamics_->template MakeState(dyn_link_names_,
                                                   dynamics_->GetJointNames());
        RCLCPP_INFO(this->get_logger(), "DynState created.");

        q_lower_ = dynamics_->GetLimitQLower(dyn_state_);
        q_upper_ = dynamics_->GetLimitQUpper(dyn_state_);
        qdot_upper_ = dynamics_->GetLimitQdotUpper(dyn_state_);
        qddot_upper_ = dynamics_->GetLimitQddotUpper(dyn_state_);
        auto joint_names = dyn_state_->GetJointNames();
        dyn_joint_names_.clear();
        for (size_t i = 0; i < joint_names.size(); ++i) {
          dyn_joint_names_.push_back(std::string(joint_names[i]));
        }

      } catch (const std::exception &e) {
        RCLCPP_WARN(
            this->get_logger(),
            "\033[1;33m[DYNAMICS WARNING] Failed to load dynamics model: %s. "
            "Cartesian poses and impedance features will be disabled.\033[0m",
            e.what());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31m================================================="
                   "===================\033[0m");
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31m[CONNECTION ERROR] Failed to connect to robot at "
                   "address: %s\033[0m",
                   params_.robot_ip.c_str());
      RCLCPP_ERROR(this->get_logger(), "\033[1;31mPlease check:\033[0m");
      RCLCPP_ERROR(this->get_logger(), "\033[1;31m  1. Is the robot's physical "
                                       "power/servo switched on?\033[0m");
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31m  2. Is the network ethernet/Wi-Fi connection "
                   "active?\033[0m");
      RCLCPP_ERROR(
          this->get_logger(),
          "\033[1;31m  3. Is the robot_ip parameter '%s' correct?\033[0m",
          params_.robot_ip.c_str());
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31m================================================="
                   "===================\033[0m");
      throw std::runtime_error("Failed to connect to RBY1 robot at address: " +
                               params_.robot_ip);
    }

    joint_overall_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        state_topic_name, 10);
    joint_state_pub_ = this->create_publisher<rby1_msgs::msg::PartJointStates>(
        "joint_states", 10);

    // consolidated robot state publisher
    robot_state_pub_ =
        this->create_publisher<rby1_msgs::msg::RobotState>("robot_state", 10);

    if (params_.publish_battery_state) {
      battery_state_pub_ =
          this->create_publisher<sensor_msgs::msg::BatteryState>(
              "battery_state", 10);
    }
    if (params_.publish_tool_flange_state) {
      // IMU publishers
      tool_flange_left_imu_pub_ =
          this->create_publisher<sensor_msgs::msg::Imu>(
              "tool_flange/left/imu", 10);
      tool_flange_right_imu_pub_ =
          this->create_publisher<sensor_msgs::msg::Imu>(
              "tool_flange/right/imu", 10);

      // Force/Torque (WrenchStamped) publishers
      tool_flange_left_wrench_pub_ =
          this->create_publisher<geometry_msgs::msg::WrenchStamped>(
              "tool_flange/left/wrench", 10);
      tool_flange_right_wrench_pub_ =
          this->create_publisher<geometry_msgs::msg::WrenchStamped>(
              "tool_flange/right/wrench", 10);
    }

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_base_footprint_tf();
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::cmd_vel_callback, this, _1));

    // loop for reading joint state
    // loop for reading joint state
    joint_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(kStatePeriod * 1000.0)),
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::read_joint_state, this));

    // service for important robot action
    power_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
        "robot_power",
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::power_control, this, _1, _2));

    servo_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
        "robot_servo",
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::servo_control, this, _1, _2));

    tool_flange_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
        "tool_flange_power",
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::tool_flange_control, this, _1,
                  _2));

    gravity_compensation_service_ =
        this->create_service<rby1_msgs::srv::GravityCompensation>(
            "gravity_compensation",
            std::bind(
                &RBY1_ROS2_DRIVER<ModelType>::gravity_compensation_callback,
                this, _1, _2));

    cancel_control_service_ = this->create_service<std_srvs::srv::Trigger>(
        "cancel_control",
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::cancel_control_callback, this,
                  _1, _2));

    get_cartesian_pose_service_ =
        this->create_service<rby1_msgs::srv::GetCartesianPose>(
            "get_cartesian_pose",
            std::bind(&RBY1_ROS2_DRIVER<ModelType>::get_cartesian_pose_callback,
                      this, _1, _2));
                      
    control_manager_service_ =
        this->create_service<rby1_msgs::srv::ControlManagerCommand>(
            "control_manager_command",
            std::bind(&RBY1_ROS2_DRIVER<ModelType>::control_manager_callback,
                      this, _1, _2));

    stream_control_service_ = this->create_service<rby1_msgs::srv::StateOnOff>(
        "stream_control",
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::stream_control_callback, this,
                  _1, _2));

    set_trajectory_impedance_service_ =
        this->create_service<rby1_msgs::srv::SetTrajectoryImpedance>(
            "set_trajectory_impedance",
            std::bind(
                &RBY1_ROS2_DRIVER<ModelType>::set_trajectory_impedance_callback,
                this, _1, _2));

    hardware_control_service_ =
        this->create_service<rby1_msgs::srv::StateOnOff>(
            "hardware_control",
            std::bind(&RBY1_ROS2_DRIVER<ModelType>::hardware_control_callback,
                      this, _1, _2));

    stream_safety_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::check_stream_safety, this));

    /* we need to add tool flange on/off service*/

    follow_joint_trajectory_action_server_ = rclcpp_action::create_server<
        FollowJointTrajectory>(
        this, "follow_joint_trajectory",
        std::bind(
            &RBY1_ROS2_DRIVER<ModelType>::handle_follow_joint_trajectory_goal,
            this, _1, _2),
        std::bind(
            &RBY1_ROS2_DRIVER<ModelType>::handle_follow_joint_trajectory_cancel,
            this, _1),
        std::bind(&RBY1_ROS2_DRIVER<
                      ModelType>::handle_follow_joint_trajectory_accepted,
                  this, _1));

    if (!joint_position_topic_name.empty()) {
      rby1_joint_command_action_server_ =
          rclcpp_action::create_server<Rby1JointCommand>(
              this, joint_position_topic_name,
              std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_goal,
                        this, _1, _2),
              std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_cancel,
                        this, _1),
              std::bind(
                  &RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_accepted,
                  this, _1));
    }

    if (!cartesian_position_topic_name.empty()) {
      rby1_cartesian_command_action_server_ =
          rclcpp_action::create_server<Rby1CartesianCommand>(
              this, cartesian_position_topic_name,
              std::bind(
                  &RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_goal,
                  this, _1, _2),
              std::bind(
                  &RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_cancel,
                  this, _1),
              std::bind(
                  &RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_accepted,
                  this, _1));
    }

    // Stream goals arrive tens of times a second. The action server keeps every
    // finished goal for result_timeout (15 min by default) and publishes their
    // statuses as one array each time, which grows into thousands of entries: a
    // Python client streaming at 50 Hz then spent 30 ms per spin on it. Streamed
    // results matter only for a moment.
    rcl_action_server_options_t stream_options = rcl_action_server_get_default_options();
    stream_options.result_timeout.nanoseconds = RCUTILS_S_TO_NS(kStreamResultTimeoutS);
    stream_joint_action_server_ = rclcpp_action::create_server<StreamJoint>(
        this, "stream_joint",
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_goal, this,
                  _1, _2),
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_cancel,
                  this, _1),
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_accepted,
                  this, _1),
        stream_options);

    stream_cartesian_action_server_ =
        rclcpp_action::create_server<StreamCartesian>(
            this, "stream_cartesian",
            std::bind(
                &RBY1_ROS2_DRIVER<ModelType>::handle_stream_cartesian_goal,
                this, _1, _2),
            std::bind(
                &RBY1_ROS2_DRIVER<ModelType>::handle_stream_cartesian_cancel,
                this, _1),
            std::bind(
                &RBY1_ROS2_DRIVER<ModelType>::handle_stream_cartesian_accepted,
                this, _1),
            stream_options);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "\033[1;31m[INITIALIZATION ERROR] Exception caught during "
                 "driver setup: %s\033[0m",
                 e.what());
    throw;
  }
}

// Leaving: stop whatever the robot is doing for this driver (single commands, streams,
// gravity compensation), drop the stream handles, and disable the control manager, so the
// robot is not left following a command nobody watches.
template <typename ModelType> RBY1_ROS2_DRIVER<ModelType>::~RBY1_ROS2_DRIVER() {
  if (!robot_)
    return;
  try {
    if (!stop_control_and_disable())
      RCLCPP_WARN(this->get_logger(), "Could not disable the Control Manager at shutdown");
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Stopping control at shutdown: %s", e.what());
  }
  gravity_compensation_torso_handler_.reset();
  gravity_compensation_right_arm_handler_.reset();
  gravity_compensation_left_arm_handler_.reset();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::init_parameter() {
  RCLCPP_INFO(this->get_logger(), "Declaring parameters...");
  this->declare_parameter<std::string>("robot_ip", "127.0.0.1:50051");
  this->declare_parameter<std::string>("model", "m");

  // The defaults are those of config/driver_parameters.yaml (2026-10-26).
  this->declare_parameter<double>("se2_minimum_time", 1.0);
  this->declare_parameter<double>("se2_linear_acceleration_limit", 0.5);
  this->declare_parameter<double>("se2_angular_acceleration_limit", 0.5);
  this->declare_parameter<double>("stream_command_hz", 30.0);
  // The robot's own defaults (read from the simulator, 2026-10-26).
  this->declare_parameter<double>("joint_position_command_cutoff_frequency", 15.0);
  this->declare_parameter<double>("cartesian_command_cutoff_frequency", 15.0);
  this->declare_parameter<double>("default_acceleration_limit_scaling", 1.0);
  this->declare_parameter<double>("default_velocity_limit_scaling", 1.0);
  this->declare_parameter<double>("default_linear_acceleration_limit", 20.0);
  this->declare_parameter<double>("default_angular_acceleration_limit", 10.0);

  this->declare_parameter<double>("collision_threshold", 0.02);
  this->declare_parameter<bool>("control_manager_unlimited", false);
  this->declare_parameter<bool>("publish_battery_state", true);
  this->declare_parameter<bool>("publish_tool_flange_state", true);

  this->get_parameter("robot_ip", params_.robot_ip);
  this->get_parameter("model", params_.model);

  state_topic_name = "joint_overall_states";
  joint_position_topic_name = "robot_joint";
  cartesian_position_topic_name = "robot_cartesian";

  this->get_parameter("se2_minimum_time", params_.se2_minimum_time);
  this->get_parameter("se2_linear_acceleration_limit",
                      params_.se2_linear_acceleration_limit);
  this->get_parameter("se2_angular_acceleration_limit",
                      params_.se2_angular_acceleration_limit);
  this->get_parameter("collision_threshold", params_.collision_threshold);
  this->get_parameter("control_manager_unlimited", params_.control_manager_unlimited);
  this->get_parameter("publish_battery_state", params_.publish_battery_state);
  this->get_parameter("publish_tool_flange_state",
                      params_.publish_tool_flange_state);
  this->get_parameter("stream_command_hz", params_.stream_command_hz);
  this->get_parameter("joint_position_command_cutoff_frequency",
                      params_.robot.joint_position_command_cutoff_frequency);
  this->get_parameter("cartesian_command_cutoff_frequency",
                      params_.robot.cartesian_command_cutoff_frequency);
  this->get_parameter("default_acceleration_limit_scaling",
                      params_.robot.default_acceleration_limit_scaling);
  this->get_parameter("default_linear_acceleration_limit",
                      params_.robot.default_linear_acceleration_limit);
  this->get_parameter("default_velocity_limit_scaling",
                      params_.robot.default_velocity_limit_scaling);
  this->get_parameter("default_angular_acceleration_limit",
                      params_.robot.default_angular_acceleration_limit);
  collision_enable_ = true;

  if (params_.robot_ip == "" || params_.model == "") {
    RCLCPP_ERROR(this->get_logger(), "address or model isn't declared");
    rclcpp::shutdown();
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::get_cartesian_pose_callback(
    const std::shared_ptr<rby1_msgs::srv::GetCartesianPose::Request> request,
    std::shared_ptr<rby1_msgs::srv::GetCartesianPose::Response> response) {

  if (!dynamics_ || !dyn_state_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Dynamics model not loaded. Cannot get cartesian pose.");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  try {
    auto state = get_state_with_retry();
    Eigen::Vector<double, ModelType::kRobotDOF> q =
        Eigen::Vector<double, ModelType::kRobotDOF>::Zero();
    auto dyn_joint_names = dyn_state_->GetJointNames();
    for (int i = 0; i < (int)dyn_joint_names.size(); ++i) {
      std::string name(dyn_joint_names[i]);
      for (size_t j = 0; j < info_.joint_infos.size(); ++j) {
        if (info_.joint_infos[j].name == name) {
          q[i] = state.position[j];
          break;
        }
      }
    }

    dyn_state_->SetQ(q);
    dynamics_->ComputeForwardKinematics(dyn_state_);

    std::string ref = request->ref_link;
    std::string target = request->target_link;

    auto it_ref =
        std::find(dyn_link_names_.begin(), dyn_link_names_.end(), ref);
    auto it_target =
        std::find(dyn_link_names_.begin(), dyn_link_names_.end(), target);

    if (it_ref != dyn_link_names_.end() && it_target != dyn_link_names_.end()) {
      int ref_idx = std::distance(dyn_link_names_.begin(), it_ref);
      int target_idx = std::distance(dyn_link_names_.begin(), it_target);

      auto T =
          dynamics_->ComputeTransformation(dyn_state_, ref_idx, target_idx);

      geometry_msgs::msg::Transform transform;
      transform.translation.x = T(0, 3);
      transform.translation.y = T(1, 3);
      transform.translation.z = T(2, 3);
      Eigen::Matrix3d R = T.block(0, 0, 3, 3);
      Eigen::Quaterniond q_rot(R);
      transform.rotation.x = q_rot.x();
      transform.rotation.y = q_rot.y();
      transform.rotation.z = q_rot.z();
      transform.rotation.w = q_rot.w();

      response->transform = transform;
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Link name not found in dynamics model. Ref: %s, Target: %s",
                   ref.c_str(), target.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error in get_cartesian_pose: %s",
                 e.what());
  }
}

template <typename ModelType>
int RBY1_ROS2_DRIVER<ModelType>::get_link_index(const std::string &name) {
  auto it = std::find(dyn_link_names_.begin(), dyn_link_names_.end(), name);
  if (it != dyn_link_names_.end()) {
    return std::distance(dyn_link_names_.begin(), it);
  }
  return -1;
}

template <typename ModelType>
std::optional<std::string>
RBY1_ROS2_DRIVER<ModelType>::get_predicted_collision_reason(
    const Eigen::VectorXd &target_q) {
  if (!dynamics_ || !dyn_state_) {
    return std::nullopt;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto temp_state =
      std::make_shared<rb::dyn::State<ModelType::kRobotDOF>>(*dyn_state_);
  temp_state->SetQ(target_q);

  dynamics_->ComputeForwardKinematics(temp_state);
  auto cols = dynamics_->DetectCollisionsOrNearestLinks(temp_state, kNearestLinkPairs);

  for (const auto &col : cols) {
    if (col.distance < params_.collision_threshold) {
      std::stringstream ss;
      ss << "Collision predicted between [" << col.link1 << "] and ["
         << col.link2 << "] (distance: " << std::fixed << std::setprecision(4)
         << col.distance << " m < threshold: " << params_.collision_threshold << " m)";
      return ss.str();
    }
  }
  return std::nullopt;
}


// The odometry moves base_footprint in odom; the robot model's root is base, on the floor
// (the SDK's wheel radius equals the axles' height: M 0.076 m, A 0.1 m). This joins the two
// TF trees. M (mecanum): base_footprint is base. A (differential drive): base_footprint is
// the middle of the wheel axle, which the SDK model puts 0.228 m in front of base
// (<mobile xyz="0.228 0 0">), so base is 0.228 m behind it. Published once, static.
template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::publish_base_footprint_tf() {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = "base_footprint";
  tf.child_frame_id = "base";
  tf.transform.translation.x =
      std::is_same_v<ModelType, rb::y1_model::A> ? -0.228 : 0.0;
  tf.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(tf);
}

// The simulator occasionally serializes a Collision whose link name is not
// valid UTF-8; protobuf then rejects the whole GetRobotState reply and gRPC
// reports UNIMPLEMENTED "No message returned for unary request" (seen once in
// ~790k calls). One bad frame is not a lost robot, so read again briefly.
template <typename ModelType>
rb::RobotState<ModelType> RBY1_ROS2_DRIVER<ModelType>::get_state_with_retry(int attempts) {
  for (int attempt = 1;; ++attempt) {
    try {
      return robot_->GetState();
    } catch (const std::exception &e) {
      if (attempt >= attempts)
        throw;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "GetState failed (attempt %d/%d), retrying: %s", attempt,
                           attempts, e.what());
      std::this_thread::sleep_for(kStateRetryDelay);
    }
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::read_joint_state() {
  if (info_.joint_infos.empty())
    return; // info가 아직 오지 않았으면 리턴
  try {
    auto state = get_state_with_retry();
    auto cm_state = robot_->GetControlManagerState();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto now = this->now();

      if (cm_state.state == rb::ControlManagerState::State::kMinorFault) {
        robot_state_.state = MINOR_FAULT;
      } else if (cm_state.state ==
                 rb::ControlManagerState::State::kMajorFault) {
        robot_state_.state = MAJOR_FAULT;
      } else if (cm_state.state == rb::ControlManagerState::State::kIdle) {
        robot_state_.state = IDLE;
      } else if (cm_state.state == rb::ControlManagerState::State::kEnabled) {
        if (cm_state.control_state ==
            rb::ControlManagerState::ControlState::kExecuting) {
          robot_state_.state = EXECUTING;
        } else {
          robot_state_.state = ENABLE;
        }
      } else {
        robot_state_.state = NONE;
      }

      // Collision check (always enabled)
      bool is_collision = false;
      double min_dist = 1e9;
      std::string link1, link2;
      if (!state.collisions.empty()) {
        for (const auto &col : state.collisions) {
          if (col.distance < min_dist) {
            min_dist = col.distance;
            link1 = col.link1;
            link2 = col.link2;
          }
        }
        if (min_dist < params_.collision_threshold) {
          is_collision = true;
        }
      }
      robot_state_.collision = is_collision;

      if (is_collision) {
        if (!has_printed_collision_log_.exchange(true)) {
          RCLCPP_WARN(this->get_logger(),
                      "Collision detected! Distance: %.4f (Links: %s <-> %s)",
                      min_dist, link1.c_str(), link2.c_str());
        }
      }

      if (dynamics_ && dyn_state_) {
        Eigen::Vector<double, ModelType::kRobotDOF> q =
            Eigen::Vector<double, ModelType::kRobotDOF>::Zero();
        for (int i = 0; i < (int)dyn_joint_names_.size(); ++i) {
          std::string name = dyn_joint_names_[i];
          for (size_t j = 0; j < info_.joint_infos.size(); ++j) {
            if (info_.joint_infos[j].name == name) {
              q[i] = state.position[j];
              break;
            }
          }
        }
        // The measured posture, for the IK's start point and the collision check (both
        // copy this state and compute their own kinematics on the copy).
        dyn_state_->SetQ(q);
      }

      // Joint states: the joints of `idx_vec` appended to `js`. Each part gets its own
      // message (joint_states); every joint goes into one (joint_overall_states, what
      // TF, MoveIt and RViz read).
      auto append = [&](sensor_msgs::msg::JointState &js,
                        const std::vector<unsigned int> &idx_vec) {
        for (unsigned int idx : idx_vec) {
          js.name.push_back(info_.joint_infos[idx].name);
          js.position.push_back(state.position[idx]);
          js.velocity.push_back(state.velocity[idx]);
          js.effort.push_back(state.torque[idx]);
        }
      };
      auto part = [&](sensor_msgs::msg::JointState &js,
                      const std::vector<unsigned int> &idx_vec) {
        js = sensor_msgs::msg::JointState();
        js.header.stamp = now;
        append(js, idx_vec);
      };
      part(part_states_.torso, info_.torso_joint_idx);
      part(part_states_.right_arm, info_.right_arm_joint_idx);
      part(part_states_.left_arm, info_.left_arm_joint_idx);
      part(part_states_.head, info_.head_joint_idx);
      part_states_.header.stamp = now;
      this->joint_state_pub_->publish(part_states_);

      sensor_msgs::msg::JointState all_joints;
      all_joints.header.stamp = now;
      for (const auto *idx_vec : {&info_.torso_joint_idx, &info_.right_arm_joint_idx,
                                  &info_.left_arm_joint_idx, &info_.head_joint_idx,
                                  &info_.mobility_joint_idx})
        append(all_joints, *idx_vec);
      this->joint_overall_state_pub_->publish(all_joints);

      // Base Mobility Odometry publishing
      {
        static double last_x = 0.0;
        static double last_y = 0.0;
        static double last_theta = 0.0;
        static bool first_odom = true;

        double x = state.odometry(0, 2);
        double y = state.odometry(1, 2);
        double theta = std::atan2(state.odometry(1, 0), state.odometry(0, 0));

        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;

        // The loop is timed at kStatePeriod but runs at 97 to 100 Hz (measured): the
        // velocity uses the time the two readings are apart, not the nominal period.
        static int64_t last_odom_ns = 0;
        const int64_t odom_ns = this->now().nanoseconds();
        const double dt = first_odom ? 0.0 : (odom_ns - last_odom_ns) * 1e-9;
        last_odom_ns = odom_ns;
        if (dt > 0.0 && !first_odom) {
          vx = (x - last_x) / dt;
          vy = (y - last_y) / dt;
          wz = (theta - last_theta) / dt;
          if (wz > M_PI / dt)
            wz -= 2.0 * M_PI / dt;
          if (wz < -M_PI / dt)
            wz += 2.0 * M_PI / dt;
        }
        first_odom = false;
        last_x = x;
        last_y = y;
        last_theta = theta;

        double cy = std::cos(theta * 0.5);
        double sy = std::sin(theta * 0.5);

        geometry_msgs::msg::Quaternion odom_quat;
        odom_quat.x = 0.0;
        odom_quat.y = 0.0;
        odom_quat.z = sy;
        odom_quat.w = cy;

        // TF frame prefixing is namespace-safe because ROS 2 handles base
        // frames relative to node
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = wz;

        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = now;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_footprint";

        odom_tf.transform.translation.x = x;
        odom_tf.transform.translation.y = y;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom_quat;

        tf_broadcaster_->sendTransform(odom_tf);
      }
      // Publish RobotState
      rby1_msgs::msg::RobotState robot_state_msg;
      robot_state_msg.control_manager_state =
          static_cast<int32_t>(robot_state_.state);

      auto fill_brakes = [&](std::vector<bool> &brake_vec,
                             const std::vector<unsigned int> &idx_vec) {
        brake_vec.resize(idx_vec.size());
        for (size_t i = 0; i < idx_vec.size(); ++i) {
          unsigned int idx = idx_vec[i];
          bool has_brake = info_.joint_infos[idx].has_brake;
          brake_vec[i] = has_brake && !state.joint_states[idx].is_ready;
        }
      };

      fill_brakes(robot_state_msg.brake_state.torso, info_.torso_joint_idx);
      fill_brakes(robot_state_msg.brake_state.right_arm,
                  info_.right_arm_joint_idx);
      fill_brakes(robot_state_msg.brake_state.left_arm,
                  info_.left_arm_joint_idx);
      fill_brakes(robot_state_msg.brake_state.head, info_.head_joint_idx);

      bool left_connected =
          (state.tool_flange_left.time_since_last_update.tv_sec != 0 ||
           state.tool_flange_left.time_since_last_update.tv_nsec != 0);
      bool right_connected =
          (state.tool_flange_right.time_since_last_update.tv_sec != 0 ||
           state.tool_flange_right.time_since_last_update.tv_nsec != 0);
      robot_state_msg.tool_flange_left.connected = left_connected;
      robot_state_msg.tool_flange_left.output_voltage =
          left_connected ? state.tool_flange_left.output_voltage : 0;
      robot_state_msg.tool_flange_left.switch_a =
          left_connected && state.tool_flange_left.switch_A;
      robot_state_msg.tool_flange_right.connected = right_connected;
      robot_state_msg.tool_flange_right.output_voltage =
          right_connected ? state.tool_flange_right.output_voltage : 0;
      robot_state_msg.tool_flange_right.switch_a =
          right_connected && state.tool_flange_right.switch_A;

      robot_state_msg.emo_state = false;
      if (!state.emo_states.empty()) {
        robot_state_msg.emo_state =
            (state.emo_states[0].state == rb::EMOState::State::kPressed);
      }

      robot_state_msg.center_of_mass[0] = state.center_of_mass[0];
      robot_state_msg.center_of_mass[1] = state.center_of_mass[1];
      robot_state_msg.center_of_mass[2] = state.center_of_mass[2];

      robot_state_msg.robot_stream_state = stream_active_;
      robot_state_msg.collision = robot_state_.collision;

      double version_val = 0.0;
      try {
        if (!info_.robot_model_version.empty()) {
          std::string version_str = info_.robot_model_version;
          // The SDK reports versions like "v1.2"; std::stod rejects the
          // leading 'v', so every robot used to publish 0.0.
          version_str.erase(0, version_str.find_first_of("0123456789"));
          std::replace(version_str.begin(), version_str.end(), '_', '.');
          version_val = std::stod(version_str);
        }
      } catch (...) {
        version_val = 0.0;
      }
      robot_state_msg.robot_version = version_val;

      robot_state_pub_->publish(robot_state_msg);

      // Publish Battery State
      if (params_.publish_battery_state) {
        sensor_msgs::msg::BatteryState bat_msg;
        bat_msg.header.stamp = now;
        bat_msg.voltage = state.battery_state.voltage;
        bat_msg.current = state.battery_state.current;
        bat_msg.percentage = state.battery_state.level_percent / 100.0;
        battery_state_pub_->publish(bat_msg);
      }

      // Publish Classified Tool Flange States (IMU, Wrench, Status)
      if (params_.publish_tool_flange_state) {
        // --- Left Flange ---
        bool lf_valid =
            (state.tool_flange_left.time_since_last_update.tv_sec != 0 ||
             state.tool_flange_left.time_since_last_update.tv_nsec != 0);
        bool lf_ft_valid =
            (state.ft_sensor_left.time_since_last_update.tv_sec != 0 ||
             state.ft_sensor_left.time_since_last_update.tv_nsec != 0);

        // 1. Left IMU
        sensor_msgs::msg::Imu imu_left;
        imu_left.header.stamp = now;
        imu_left.header.frame_id = "ee_left";
        if (lf_valid) {
          imu_left.angular_velocity.x = state.tool_flange_left.gyro[0];
          imu_left.angular_velocity.y = state.tool_flange_left.gyro[1];
          imu_left.angular_velocity.z = state.tool_flange_left.gyro[2];
          imu_left.linear_acceleration.x =
              state.tool_flange_left.acceleration[0];
          imu_left.linear_acceleration.y =
              state.tool_flange_left.acceleration[1];
          imu_left.linear_acceleration.z =
              state.tool_flange_left.acceleration[2];
        }
        imu_left.orientation_covariance[0] = -1.0; // Orientation not available
        tool_flange_left_imu_pub_->publish(imu_left);

        // 2. Left F/T (Wrench)
        geometry_msgs::msg::WrenchStamped wrench_left;
        wrench_left.header.stamp = now;
        wrench_left.header.frame_id = "ee_left";
        if (lf_ft_valid) {
          wrench_left.wrench.force.x = state.ft_sensor_left.force[0];
          wrench_left.wrench.force.y = state.ft_sensor_left.force[1];
          wrench_left.wrench.force.z = state.ft_sensor_left.force[2];
          wrench_left.wrench.torque.x = state.ft_sensor_left.torque[0];
          wrench_left.wrench.torque.y = state.ft_sensor_left.torque[1];
          wrench_left.wrench.torque.z = state.ft_sensor_left.torque[2];
        }
        tool_flange_left_wrench_pub_->publish(wrench_left);

        // --- Right Flange ---
        bool rf_valid =
            (state.tool_flange_right.time_since_last_update.tv_sec != 0 ||
             state.tool_flange_right.time_since_last_update.tv_nsec != 0);
        bool rf_ft_valid =
            (state.ft_sensor_right.time_since_last_update.tv_sec != 0 ||
             state.ft_sensor_right.time_since_last_update.tv_nsec != 0);

        // 1. Right IMU
        sensor_msgs::msg::Imu imu_right;
        imu_right.header.stamp = now;
        imu_right.header.frame_id = "ee_right";
        if (rf_valid) {
          imu_right.angular_velocity.x = state.tool_flange_right.gyro[0];
          imu_right.angular_velocity.y = state.tool_flange_right.gyro[1];
          imu_right.angular_velocity.z = state.tool_flange_right.gyro[2];
          imu_right.linear_acceleration.x =
              state.tool_flange_right.acceleration[0];
          imu_right.linear_acceleration.y =
              state.tool_flange_right.acceleration[1];
          imu_right.linear_acceleration.z =
              state.tool_flange_right.acceleration[2];
        }
        imu_right.orientation_covariance[0] = -1.0; // Orientation not available
        tool_flange_right_imu_pub_->publish(imu_right);

        // 2. Right F/T (Wrench)
        geometry_msgs::msg::WrenchStamped wrench_right;
        wrench_right.header.stamp = now;
        wrench_right.header.frame_id = "ee_right";
        if (rf_ft_valid) {
          wrench_right.wrench.force.x = state.ft_sensor_right.force[0];
          wrench_right.wrench.force.y = state.ft_sensor_right.force[1];
          wrench_right.wrench.force.z = state.ft_sensor_right.force[2];
          wrench_right.wrench.torque.x = state.ft_sensor_right.torque[0];
          wrench_right.wrench.torque.y = state.ft_sensor_right.torque[1];
          wrench_right.wrench.torque.z = state.ft_sensor_right.torque[2];
        }
        tool_flange_right_wrench_pub_->publish(wrench_right);
      }
    }
    last_state_ok_ns_ = this->now().nanoseconds();
    failed_state_reads_ = 0;
  } catch (const std::exception &e) {
    // Only a sustained failure means the connection is gone; a stray bad
    // frame from the robot must not take the whole driver down.
    const int64_t now_ns = this->now().nanoseconds();
    if (last_state_ok_ns_ == 0)
      last_state_ok_ns_ = now_ns;
    ++failed_state_reads_;
    const double failing_for = (now_ns - last_state_ok_ns_) * 1e-9;
    if (failing_for < kStateLossTimeout) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 250,
                           "State read failed (%d in a row, %.2f s): %s -- a blip is harmless; "
                           "the driver stops after %.1f s of this. Check the robot or "
                           "simulator and the network if it keeps coming",
                           failed_state_reads_, failing_for, e.what(), kStateLossTimeout);
      return;
    }
    RCLCPP_ERROR(this->get_logger(), "Error in read_joint_state loop: %s", e.what());
    RCLCPP_FATAL(this->get_logger(),
                 "Connection to robot lost: no state for %.2f s (%d failed reads). "
                 "Shutting down driver node.", failing_for, failed_state_reads_);
    rclcpp::shutdown();
  }
}

// Explicit template instantiations
template class RBY1_ROS2_DRIVER<rb::y1_model::A>;
template class RBY1_ROS2_DRIVER<rb::y1_model::M>;
} // namespace rby1_ros2
