#include "rby1_ros2_driver.hpp"
#include <iomanip>
#include <sstream>
namespace rby1_ros2 {
// using namespace rb;
template <typename ModelType>
RBY1_ROS2_DRIVER<ModelType>::RBY1_ROS2_DRIVER() : Node("rby1_ros2_driver") {
  // declare parameter from yaml
  init_parameter();
  try {
    robot_ = rb::Robot<ModelType>::Create(address);
    if (robot_->Connect()) {
      robot_->SetParameter("default.acceleration_limit_scaling",
                           std::to_string(robot_parameter_.acceleration_limit));
      robot_->SetParameter(
          "joint_position_command.cutoff_frequency",
          std::to_string(robot_parameter_.angular_velocity_limit));
      robot_->SetParameter(
          "cartesian_command.cutoff_frequency",
          std::to_string(robot_parameter_.linear_velocity_limit));
      robot_->SetParameter("default.linear_acceleration_limit",
                           std::to_string(robot_parameter_.acceleration_limit));
      // Fetch robot info once and cache it
      info_ = robot_->GetRobotInfo();

      // Auto-reset MAJOR/MINOR faults on startup if fault_reset_trigger is true
      if (fault_reset_trigger) {
        const auto &control_manager_state = robot_->GetControlManagerState();
        if (control_manager_state.state ==
                rb::ControlManagerState::State::kMajorFault ||
            control_manager_state.state ==
                rb::ControlManagerState::State::kMinorFault) {
          RCLCPP_INFO(this->get_logger(),
                      "Auto-resetting active fault on startup as requested by "
                      "fault_reset_trigger...");
          if (robot_->ResetFaultControlManager()) {
            RCLCPP_INFO(this->get_logger(),
                        "Fault auto-reset completed successfully.");
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to auto-reset fault on startup.");
          }
        }
      }

      RCLCPP_INFO(this->get_logger(),
                  "Robot Info: Model=%s, Version=%s, Compile-time DOF=%zu",
                  info_.robot_model_name.c_str(),
                  info_.robot_model_version.c_str(), ModelType::kRobotDOF);

      // Model check in driver constructor
      {
        std::string exp_model = model;
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
                       model.c_str(), info_.robot_model_name.c_str());
          throw std::runtime_error("Robot model mismatch: configured " + model + ", connected " + info_.robot_model_name);
        }
      }
      RCLCPP_INFO(this->get_logger(),
                  "Joint counts: torso=%zu, right_arm=%zu, left_arm=%zu, "
                  "head=%zu, mobility=%zu",
                  info_.torso_joint_idx.size(),
                  info_.right_arm_joint_idx.size(),
                  info_.left_arm_joint_idx.size(), info_.head_joint_idx.size(),
                  info_.mobility_joint_idx.size());
      resize_joint_states();
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
                   address.c_str());
      RCLCPP_ERROR(this->get_logger(), "\033[1;31mPlease check:\033[0m");
      RCLCPP_ERROR(this->get_logger(), "\033[1;31m  1. Is the robot's physical "
                                       "power/servo switched on?\033[0m");
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31m  2. Is the network ethernet/Wi-Fi connection "
                   "active?\033[0m");
      RCLCPP_ERROR(
          this->get_logger(),
          "\033[1;31m  3. Is the robot_ip parameter '%s' correct?\033[0m",
          address.c_str());
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31m================================================="
                   "===================\033[0m");
      throw std::runtime_error("Failed to connect to RBY1 robot at address: " +
                               address);
    }

    // robot state publisher
    torso_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        state_topic_name + "/torso", 10);
    right_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        state_topic_name + "/right_arm", 10);
    left_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        state_topic_name + "/left_arm", 10);
    head_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        state_topic_name + "/head", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        state_topic_name, 10);

    // consolidated robot state publisher
    robot_state_pub_ =
        this->create_publisher<rby1_msgs::msg::RobotState>("robot_state", 10);

    if (publish_battery_state_) {
      battery_state_pub_ =
          this->create_publisher<sensor_msgs::msg::BatteryState>(
              "battery_state", 10);
    }
    if (publish_tool_flange_state_) {
      tool_flange_left_pub_ =
          this->create_publisher<rby1_msgs::msg::ToolFlangeState>(
              "tool_flange/left", 10);
      tool_flange_right_pub_ =
          this->create_publisher<rby1_msgs::msg::ToolFlangeState>(
              "tool_flange/right", 10);
    }

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::cmd_vel_callback, this, _1));

    // loop for reading joint state
    // loop for reading joint state
    joint_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(robot_parameter_.get_state_period * 1000.0)),
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

    last_stream_command_time_ns_ = this->now().nanoseconds();
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

    stream_joint_action_server_ = rclcpp_action::create_server<StreamJoint>(
        this, "stream_joint",
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_goal, this,
                  _1, _2),
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_cancel,
                  this, _1),
        std::bind(&RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_accepted,
                  this, _1));

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
                this, _1));

    if (robot_initialize_flag) {
      RCLCPP_INFO(this->get_logger(),
                  "[INITIALIZE] initialize_robot is true. Starting automatic "
                  "power/servo initialization...");

      // --- Power ON (mirrors power_control service logic) ---
      {
        // Build power_list_str from robot_parameter_.power_on_list
        std::string power_list_str;
        if (address == "127.0.0.1:50051" || address == "localhost:50051") {
          power_list_str = ".*";
        } else {
          const auto &plist = robot_parameter_.power_on_list;
          for (size_t i = 0; i < plist.size(); i++) {
            if (plist[i] == "all" || plist[i] == ".*") {
              power_list_str = ".*";
              break;
            }
            power_list_str += plist[i];
            if (plist[i].find('v') == std::string::npos && plist[i] != ".*") {
              power_list_str += "v";
            }
            if (i != plist.size() - 1) {
              power_list_str += "|";
            }
          }
        }

        if (robot_->IsPowerOn(power_list_str)) {
          RCLCPP_INFO(this->get_logger(),
                      "[INITIALIZE] Power is already ON [%s], skipping.",
                      power_list_str.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "[INITIALIZE] Power ON [%s]...",
                      power_list_str.c_str());
          if (!robot_->PowerOn(power_list_str)) {
            RCLCPP_ERROR(this->get_logger(),
                         "[INITIALIZE] Failed to power on [%s]. Aborting "
                         "initialization.",
                         power_list_str.c_str());
            throw std::runtime_error(
                "[INITIALIZE] PowerOn failed during robot initialization.");
          }
          RCLCPP_INFO(this->get_logger(), "[INITIALIZE] Power ON succeeded.");
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
      }

      // --- Servo ON (mirrors servo_control service logic) ---
      {
        // Build servo_list_str from robot_parameter_.servo_on_list
        std::string servo_list_str;
        const auto &slist = robot_parameter_.servo_on_list;
        for (size_t i = 0; i < slist.size(); i++) {
          const std::string &name = slist[i];
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
          if (i != slist.size() - 1) {
            servo_list_str += "|";
          }
        }

        bool already_on = false;
        try {
          if (robot_->IsServoOn(servo_list_str))
            already_on = true;
        } catch (...) {
        }
        if (robot_->GetControlManagerState().state ==
            rb::ControlManagerState::State::kEnabled) {
          already_on = true;
        }

        if (already_on) {
          RCLCPP_INFO(this->get_logger(),
                      "[INITIALIZE] Servo is already ON [%s], skipping.",
                      servo_list_str.c_str());
          check_controll_manager();
        } else {
          RCLCPP_INFO(this->get_logger(), "[INITIALIZE] Servo ON [%s]...",
                      servo_list_str.c_str());
          if (!robot_->ServoOn(servo_list_str)) {
            RCLCPP_ERROR(this->get_logger(),
                         "[INITIALIZE] SDK ServoOn failed for [%s]. Aborting "
                         "initialization.",
                         servo_list_str.c_str());
            throw std::runtime_error(
                "[INITIALIZE] ServoOn failed during robot initialization.");
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(500));

          { // sync info
            std::lock_guard<std::mutex> lock(mutex_);
            info_ = robot_->GetRobotInfo();
            resize_joint_states();
          }

          RCLCPP_INFO(
              this->get_logger(),
              "[INITIALIZE] Checking Control Manager after Servo ON...");
          if (!check_controll_manager()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "[INITIALIZE] Control Manager check failed after Servo ON.");
            throw std::runtime_error(
                "[INITIALIZE] Control Manager failed after ServoOn during "
                "robot initialization.");
          }
          RCLCPP_INFO(this->get_logger(),
                      "[INITIALIZE] Servo ON and Control Manager ready.");
        }
      }

      RCLCPP_INFO(this->get_logger(),
                  "[INITIALIZE] Robot initialization complete.");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "\033[1;31m[INITIALIZATION ERROR] Exception caught during "
                 "driver setup: %s\033[0m",
                 e.what());
    throw;
  }
}

template <typename ModelType> RBY1_ROS2_DRIVER<ModelType>::~RBY1_ROS2_DRIVER() {
  stream_active_ = false;
  if (upper_body_stream_handler_) {
    upper_body_stream_handler_.reset();
  }
  if (mobility_stream_handler_) {
    mobility_stream_handler_.reset();
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::init_parameter() {
  RCLCPP_INFO(this->get_logger(), "Declaring parameters...");
  this->declare_parameter<std::string>("robot_ip", "127.0.0.1:50051");
  this->declare_parameter<std::string>("model", "a");
  this->declare_parameter<bool>("initialize_robot", false);
  this->declare_parameter<std::vector<std::string>>("power_on", {"all"});
  this->declare_parameter<std::vector<std::string>>("servo_on", {"all"});

  this->declare_parameter<double>("get_state_period", 0.01);
  this->declare_parameter<double>("minimum_time", 2.0);
  this->declare_parameter<double>("angular_velocity_limit", 4.712388);
  this->declare_parameter<double>("linear_velocity_limit", 1.5);
  this->declare_parameter<double>("acceleration_limit", 1.0);
  this->declare_parameter<double>("stop_orientation_tracking_error", 1e-5);
  this->declare_parameter<double>("stop_position_tracking_error", 1e-5);
  this->declare_parameter<double>("se2_minimum_time", 1.0);
  this->declare_parameter<double>("se2_linear_acceleration_limit", 0.5);
  this->declare_parameter<double>("se2_angular_acceleration_limit", 0.5);
  this->declare_parameter<double>("stream_hz", 15.0);

  this->declare_parameter<bool>("fault_reset_trigger", true);
  this->declare_parameter<double>("collision_threshold", 0.01);
  this->declare_parameter<bool>("publish_battery_state", true);
  this->declare_parameter<bool>("publish_tool_flange_state", true);

  this->get_parameter("robot_ip", address);
  this->get_parameter("model", model);

  state_topic_name = "joint_states";
  joint_position_topic_name = "robot_joint";
  cartesian_position_topic_name = "robot_cartesian";

  this->get_parameter("initialize_robot", robot_initialize_flag);
  this->get_parameter("power_on", robot_parameter_.power_on_list);
  this->get_parameter("servo_on", robot_parameter_.servo_on_list);
  this->get_parameter("get_state_period", robot_parameter_.get_state_period);
  this->get_parameter("minimum_time", robot_parameter_.minimum_time);
  this->get_parameter("angular_velocity_limit",
                      robot_parameter_.angular_velocity_limit);
  this->get_parameter("linear_velocity_limit",
                      robot_parameter_.linear_velocity_limit);
  this->get_parameter("acceleration_limit",
                      robot_parameter_.acceleration_limit);
  this->get_parameter("stop_orientation_tracking_error",
                      robot_parameter_.stop_orientation_tracking_error);
  this->get_parameter("stop_position_tracking_error",
                      robot_parameter_.stop_position_tracking_error);
  this->get_parameter("se2_minimum_time", robot_parameter_.se2_minimum_time);
  this->get_parameter("se2_linear_acceleration_limit",
                      robot_parameter_.se2_linear_acceleration_limit);
  this->get_parameter("se2_angular_acceleration_limit",
                      robot_parameter_.se2_angular_acceleration_limit);
  this->get_parameter("fault_reset_trigger", fault_reset_trigger);
  this->get_parameter("collision_threshold", collision_threshold_);
  this->get_parameter("publish_battery_state", publish_battery_state_);
  this->get_parameter("publish_tool_flange_state", publish_tool_flange_state_);
  this->get_parameter("stream_hz", stream_hz_);
  collision_enable_ = true;

  if (address == "" || model == "") {
    RCLCPP_ERROR(this->get_logger(), "address or model isn't declared");
    rclcpp::shutdown();
  }
}

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

  std::string power_list_str = "";
  if (address == "127.0.0.1:50051") {
    power_list_str = ".*";
  } else {
    for (size_t i = 0; i < param_list.size(); i++) {
      if (param_list[i] == "all" || param_list[i] == ".*") {
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
      if (upper_body_stream_handler_) {
        upper_body_stream_handler_.reset();
      }
      if (mobility_stream_handler_) {
        mobility_stream_handler_.reset();
      }
      stream_active_ = false;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    { // sync info
      std::lock_guard<std::mutex> lock(mutex_);
      info_ = robot_->GetRobotInfo();
      resize_joint_states();
    }

    RCLCPP_INFO(this->get_logger(),
                "Checking Control Manager after Servo ON...");
    if (!check_controll_manager()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Control Manager check failed after Servo ON.");
      response->success = false;
      if (robot_->IsPowerOn("48v")) {
        response->message = "Control Manager failed to enable (Power is ON)";
      } else {
        response->message = "Control Manager failed: Robot power is OFF";
      }
      return;
    }
    response->success = true;
    response->message = "Servo control success";
  } else {
    RCLCPP_INFO(this->get_logger(), "Servo OFF [%s]...",
                servo_list_str.c_str());
    robot_->DisableControlManager();
    {
      std::lock_guard<std::mutex> stream_lock(stream_mutex_);
      if (upper_body_stream_handler_) {
        upper_body_stream_handler_.reset();
      }
      if (mobility_stream_handler_) {
        mobility_stream_handler_.reset();
      }
      stream_active_ = false;
    }
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
template <typename ModelType>
geometry_msgs::msg::Pose
RBY1_ROS2_DRIVER<ModelType>::matrix_to_pose(const Eigen::Matrix4d &matrix) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::gravity_compensation_callback(
    const std::shared_ptr<rby1_msgs::srv::GravityCompensation::Request> request,
    std::shared_ptr<rby1_msgs::srv::GravityCompensation::Response> response) {
  if (request->part_name != "torso" && request->part_name != "right_arm" &&
      request->part_name != "left_arm") {
    response->response = false;
    RCLCPP_ERROR(this->get_logger(),
                 "GravityCompensation: Invalid part_name: %s",
                 request->part_name.c_str());
    return;
  }

  size_t part_idx = PART_COUNT;
  if (request->part_name == "torso")
    part_idx = PART_TORSO;
  else if (request->part_name == "right_arm")
    part_idx = PART_RIGHT_ARM;
  else if (request->part_name == "left_arm")
    part_idx = PART_LEFT_ARM;

  // 1. Collision check: Robot must NOT be moving (executing/switching)
  const auto &cm_state = robot_->GetControlManagerState();

  bool current_gc_state = false;
  if (part_idx == PART_TORSO)
    current_gc_state = gravity_compensation_torso_;
  else if (part_idx == PART_RIGHT_ARM)
    current_gc_state = gravity_compensation_right_arm_;
  else if (part_idx == PART_LEFT_ARM)
    current_gc_state = gravity_compensation_left_arm_;

  // We only enforce the active execution check if:
  // - We are enabling gravity compensation (request->state == true)
  // - AND gravity compensation is not already active on this body part.
  if (request->state) {
    {
      std::lock_guard<std::mutex> stream_lock(stream_mutex_);
      if (stream_active_) {
        response->response = false;
        RCLCPP_WARN(this->get_logger(),
                    "GravityCompensation: Stream is active. Gravity "
                    "compensation cannot be enabled while stream is active.");
        return;
      }
    }
    if (is_controlling_[part_idx].load(std::memory_order_acquire) &&
        !current_gc_state) {
      response->response = false;
      RCLCPP_WARN(this->get_logger(),
                  "GravityCompensation: Part %s is already being controlled.",
                  request->part_name.c_str());
      return;
    }

    if (!current_gc_state) {
      if (cm_state.control_state ==
              rb::ControlManagerState::ControlState::kExecuting ||
          cm_state.control_state ==
              rb::ControlManagerState::ControlState::kSwitching) {
        response->response = false;
        RCLCPP_WARN(this->get_logger(),
                    "GravityCompensation: Robot is active in "
                    "execution/switching state. Rejecting request.");
        return;
      }
    }
  }

  // 2. Build and Send Gravity Compensation command
  auto b = rb::GravityCompensationCommandBuilder();
  b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(
      10.0)); // standard control hold time
  b.SetOn(request->state);

  auto body_comp = rb::BodyComponentBasedCommandBuilder();
  if (request->part_name == "torso") {
    body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
  } else if (request->part_name == "right_arm") {
    body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
  } else if (request->part_name == "left_arm") {
    body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
  }

  auto component_cmd_builder = rb::ComponentBasedCommandBuilder();
  component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));

  try {
    if (request->state) {
      is_controlling_[part_idx].store(true, std::memory_order_release);
      auto cmd_handler = robot_->SendCommand(
          rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
      if (request->part_name == "torso") {
        gravity_compensation_torso_ = true;
        gravity_compensation_torso_handler_ = std::move(cmd_handler);
      } else if (request->part_name == "right_arm") {
        gravity_compensation_right_arm_ = true;
        gravity_compensation_right_arm_handler_ = std::move(cmd_handler);
      } else if (request->part_name == "left_arm") {
        gravity_compensation_left_arm_ = true;
        gravity_compensation_left_arm_handler_ = std::move(cmd_handler);
      }
    } else {
      // Reset/cancel the original ON command handler first
      if (request->part_name == "torso") {
        gravity_compensation_torso_handler_.reset();
        gravity_compensation_torso_ = false;
      } else if (request->part_name == "right_arm") {
        gravity_compensation_right_arm_handler_.reset();
        gravity_compensation_right_arm_ = false;
      } else if (request->part_name == "left_arm") {
        gravity_compensation_left_arm_handler_.reset();
        gravity_compensation_left_arm_ = false;
      }
      is_controlling_[part_idx].store(false, std::memory_order_release);

      // Send explicit command to turn gravity compensation OFF and wait for
      // completion
      auto cmd_handler = robot_->SendCommand(
          rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
      if (cmd_handler) {
        cmd_handler->WaitFor(
            1000); // Wait up to 1 second for transition to complete
      }
    }
    response->response = true;
    RCLCPP_INFO(
        this->get_logger(),
        "GravityCompensation: Successfully set %s gravity compensation to %s",
        request->part_name.c_str(), request->state ? "ON" : "OFF");
  } catch (const std::exception &e) {
    if (request->state) {
      is_controlling_[part_idx].store(false, std::memory_order_release);
      if (request->part_name == "torso")
        gravity_compensation_torso_ = false;
      else if (request->part_name == "right_arm")
        gravity_compensation_right_arm_ = false;
      else if (request->part_name == "left_arm")
        gravity_compensation_left_arm_ = false;
    }
    response->response = false;
    RCLCPP_ERROR(this->get_logger(),
                 "GravityCompensation: Failed to send command: %s", e.what());
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

  RCLCPP_INFO(this->get_logger(), "Enabling Control Manager...");
  if (!robot_->EnableControlManager()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable the Control Manager.");
    return false;
  }

  // Wait for control ready to ensure SendCommand doesn't fail with kUnknown
  // immediately
  if (!robot_->WaitForControlReady(2000)) {
    RCLCPP_WARN(this->get_logger(), "Control Manager enabled, but timed out "
                                    "waiting for Control Ready status.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Control Manager enabled and ready.");
  }

  return true;
}

template <typename ModelType>
std::string RBY1_ROS2_DRIVER<ModelType>::finish_code_to_string(
    rb::RobotCommandFeedback::FinishCode code) {
  switch (code) {
  case rb::RobotCommandFeedback::FinishCode::kUnknown:
    return "kUnknown";
  case rb::RobotCommandFeedback::FinishCode::kOk:
    return "kOk";
  case rb::RobotCommandFeedback::FinishCode::kCanceled:
    return "kCanceled";
  case rb::RobotCommandFeedback::FinishCode::kPreempted:
    return "kPreempted";
  case rb::RobotCommandFeedback::FinishCode::kInitializationFailed:
    return "kInitializationFailed";
  case rb::RobotCommandFeedback::FinishCode::kControlManagerIdle:
    return "kControlManagerIdle";
  case rb::RobotCommandFeedback::FinishCode::kControlManagerFault:
    return "kControlManagerFault";
  case rb::RobotCommandFeedback::FinishCode::kUnexpectedState:
    return "kUnexpectedState";
  default:
    return "Unknown";
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::read_joint_state() {
  if (info_.joint_infos.empty())
    return; // info가 아직 오지 않았으면 리턴
  try {
    auto state = robot_->GetState();
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
        if (min_dist < collision_threshold_) {
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

      auto fill = [&](JointState &js,
                      const std::vector<unsigned int> &idx_vec) {
        js.header.stamp = now;
        for (size_t i = 0; i < idx_vec.size(); ++i) {
          unsigned int idx = idx_vec[i];
          js.name[i] = info_.joint_infos[idx].name;
          js.position[i] = state.position[idx];
          js.velocity[i] = state.velocity[idx];
          js.effort[i] = state.torque[idx];
        }
      };

      fill(this->robot_joint_.joint_torso, this->info_.torso_joint_idx);
      fill(this->robot_joint_.joint_right_arm, this->info_.right_arm_joint_idx);
      fill(this->robot_joint_.joint_left_arm, this->info_.left_arm_joint_idx);
      fill(this->robot_joint_.joint_head, this->info_.head_joint_idx);
      fill(this->robot_joint_.joint_wheel, this->info_.mobility_joint_idx);

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
        dyn_state_->SetQ(q);
        dynamics_->ComputeForwardKinematics(dyn_state_);
      }

      this->torso_pub_->publish(this->robot_joint_.joint_torso);
      this->right_arm_pub_->publish(this->robot_joint_.joint_right_arm);
      this->left_arm_pub_->publish(this->robot_joint_.joint_left_arm);
      this->head_pub_->publish(this->robot_joint_.joint_head);

      // Publish unified joint states for MoveIt integration
      {
        sensor_msgs::msg::JointState unified_js;
        unified_js.header.stamp = now;

        auto append_joints = [&](const std::vector<unsigned int> &idx_vec) {
          for (size_t i = 0; i < idx_vec.size(); ++i) {
            unsigned int idx = idx_vec[i];
            unified_js.name.push_back(info_.joint_infos[idx].name);
            unified_js.position.push_back(state.position[idx]);
            unified_js.velocity.push_back(state.velocity[idx]);
            unified_js.effort.push_back(state.torque[idx]);
          }
        };

        append_joints(this->info_.torso_joint_idx);
        append_joints(this->info_.right_arm_joint_idx);
        append_joints(this->info_.left_arm_joint_idx);
        append_joints(this->info_.head_joint_idx);
        append_joints(this->info_.mobility_joint_idx);

        this->joint_state_pub_->publish(unified_js);
      }

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

        double dt = robot_parameter_.get_state_period;
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
      robot_state_msg.tool_flange_state = {left_connected, right_connected};

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
          std::replace(version_str.begin(), version_str.end(), '_', '.');
          version_val = std::stod(version_str);
        }
      } catch (...) {
        version_val = 0.0;
      }
      robot_state_msg.robot_version = version_val;

      robot_state_pub_->publish(robot_state_msg);

      // Publish Battery State
      if (publish_battery_state_) {
        sensor_msgs::msg::BatteryState bat_msg;
        bat_msg.header.stamp = now;
        bat_msg.voltage = state.battery_state.voltage;
        bat_msg.current = state.battery_state.current;
        bat_msg.percentage = state.battery_state.level_percent / 100.0;
        battery_state_pub_->publish(bat_msg);
      }

      // Publish Tool Flange State split
      if (publish_tool_flange_state_) {
        auto fill_vec3 = [](std::array<double, 3> &dest,
                            const Eigen::Vector<double, 3> &src, bool valid) {
          if (valid) {
            dest[0] = src[0];
            dest[1] = src[1];
            dest[2] = src[2];
          } else {
            dest[0] = 0.0;
            dest[1] = 0.0;
            dest[2] = 0.0;
          }
        };

        // left flange
        bool lf_valid =
            (state.tool_flange_left.time_since_last_update.tv_sec != 0 ||
             state.tool_flange_left.time_since_last_update.tv_nsec != 0);
        bool lf_ft_valid =
            (state.ft_sensor_left.time_since_last_update.tv_sec != 0 ||
             state.ft_sensor_left.time_since_last_update.tv_nsec != 0);
        rby1_msgs::msg::ToolFlangeState tf_left;
        fill_vec3(tf_left.ft_force, state.ft_sensor_left.force, lf_ft_valid);
        fill_vec3(tf_left.ft_torque, state.ft_sensor_left.torque, lf_ft_valid);
        fill_vec3(tf_left.gyro, state.tool_flange_left.gyro, lf_valid);
        fill_vec3(tf_left.acceleration, state.tool_flange_left.acceleration,
                  lf_valid);
        tf_left.switch_a = lf_valid ? state.tool_flange_left.switch_A : false;
        tf_left.output_voltage =
            lf_valid ? state.tool_flange_left.output_voltage : 0;
        tf_left.digital_input_a =
            lf_valid ? state.tool_flange_left.digital_input_A : false;
        tf_left.digital_input_b =
            lf_valid ? state.tool_flange_left.digital_input_B : false;
        tf_left.digital_output_a =
            lf_valid ? state.tool_flange_left.digital_output_A : false;
        tf_left.digital_output_b =
            lf_valid ? state.tool_flange_left.digital_output_B : false;
        tool_flange_left_pub_->publish(tf_left);

        // right flange
        bool rf_valid =
            (state.tool_flange_right.time_since_last_update.tv_sec != 0 ||
             state.tool_flange_right.time_since_last_update.tv_nsec != 0);
        bool rf_ft_valid =
            (state.ft_sensor_right.time_since_last_update.tv_sec != 0 ||
             state.ft_sensor_right.time_since_last_update.tv_nsec != 0);
        rby1_msgs::msg::ToolFlangeState tf_right;
        fill_vec3(tf_right.ft_force, state.ft_sensor_right.force, rf_ft_valid);
        fill_vec3(tf_right.ft_torque, state.ft_sensor_right.torque,
                  rf_ft_valid);
        fill_vec3(tf_right.gyro, state.tool_flange_right.gyro, rf_valid);
        fill_vec3(tf_right.acceleration, state.tool_flange_right.acceleration,
                  rf_valid);
        tf_right.switch_a = rf_valid ? state.tool_flange_right.switch_A : false;
        tf_right.output_voltage =
            rf_valid ? state.tool_flange_right.output_voltage : 0;
        tf_right.digital_input_a =
            rf_valid ? state.tool_flange_right.digital_input_A : false;
        tf_right.digital_input_b =
            rf_valid ? state.tool_flange_right.digital_input_B : false;
        tf_right.digital_output_a =
            rf_valid ? state.tool_flange_right.digital_output_A : false;
        tf_right.digital_output_b =
            rf_valid ? state.tool_flange_right.digital_output_B : false;
        tool_flange_right_pub_->publish(tf_right);
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error in read_joint_state loop: %s", e.what());
    RCLCPP_FATAL(this->get_logger(), "Connection to robot lost. Shutting down driver node.");
    rclcpp::shutdown();
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::cancel_control_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Cancel Control service called");
  is_control_canceled_ = true;
  // Clear all per-part control flags
  for (size_t i = 0; i < PART_COUNT; ++i)
    is_controlling_[i].store(false, std::memory_order_release);
  // Increment preemption sequence to unblock any waiting actions
  ++command_seq_;
  robot_->CancelControl();
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (upper_body_stream_handler_) {
      upper_body_stream_handler_->Cancel();
      upper_body_stream_handler_.reset();
    }
    if (mobility_stream_handler_) {
      mobility_stream_handler_->Cancel();
      mobility_stream_handler_.reset();
    }
    stream_active_ = false;
  }
  response->success = true;
  response->message = "Control cancelled";
}

// --- FollowJointTrajectory Handlers ---
template <typename ModelType>
rclcpp_action::GoalResponse
RBY1_ROS2_DRIVER<ModelType>::handle_follow_joint_trajectory_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received FollowJointTrajectory request");
  (void)uuid;
  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting FollowJointTrajectory: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory has no points.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_follow_joint_trajectory_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "Received request to cancel FollowJointTrajectory goal");
  (void)goal_handle;
  robot_->CancelControl();
  // if (stream_handler_) {
  //     stream_handler_->Cancel();
  // }
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_follow_joint_trajectory_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>
        goal_handle) {
  using namespace std::placeholders;
  RCLCPP_INFO(
      this->get_logger(),
      "FollowJointTrajectory Goal accepted. Detaching execution thread...");

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_follow_joint_trajectory_goal_ &&
        active_follow_joint_trajectory_goal_->is_active()) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      result->error_string = "Preempted by new goal";
      try {
        active_follow_joint_trajectory_goal_->abort(result);
      } catch (...) {
      }
    }
    active_follow_joint_trajectory_goal_ = goal_handle;
  }

  std::thread{
      std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_follow_joint_trajectory,
                this, _1),
      goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_follow_joint_trajectory(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>
        goal_handle) {
  has_printed_collision_log_ = false;

  auto result = std::make_shared<FollowJointTrajectory::Result>();

  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (!stream_active_ || !upper_body_stream_handler_) {
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      result->error_string =
          "Stream control is not active. Please activate stream control first.";
      RCLCPP_WARN(this->get_logger(), "\033[1;33m[CONTROL REJECTED] FJT "
                                      "rejected: Stream not active.\033[0m");
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  is_control_canceled_ = false;

  // Mark relevant body parts as controlled
  // (FJT controls torso + arms + head based on joint names)
  std::vector<size_t> fjt_parts = {PART_TORSO, PART_RIGHT_ARM, PART_LEFT_ARM,
                                   PART_HEAD};
  // In stream mode, set is_controlling_ flags
  for (auto p : fjt_parts)
    is_controlling_[p].store(true, std::memory_order_release);

  // RAII cleanup
  struct FjtGuard {
    std::atomic<bool> *flags;
    std::vector<size_t> parts;
    FjtGuard(std::atomic<bool> *f, std::vector<size_t> p)
        : flags(f), parts(std::move(p)) {}
    ~FjtGuard() {
      for (auto i : parts)
        flags[i].store(false, std::memory_order_release);
    }
  } fguard(is_controlling_, fjt_parts);

  RCLCPP_INFO(this->get_logger(), "Starting FollowJointTrajectory (stream=%s)",
              stream_active_ ? "ON" : "OFF");
  const auto goal = goal_handle->get_goal();
  const auto &trajectory = goal->trajectory;
  std::vector<int> joint_mapping(trajectory.joint_names.size(), -1);

  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    for (size_t j = 0; j < info_.joint_infos.size(); ++j) {
      if (trajectory.joint_names[i] == info_.joint_infos[j].name) {
        joint_mapping[i] = j;
        break;
      }
    }
  }

  // Joint limit checks across all waypoints
  if (dynamics_ && dyn_state_ && q_lower_.size() > 0) {
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &pt = trajectory.points[i];
      for (size_t j = 0; j < joint_mapping.size(); ++j) {
        if (joint_mapping[j] < 0 || j >= pt.positions.size())
          continue;
        double pos = pt.positions[j];
        std::string jname = info_.joint_infos[joint_mapping[j]].name;
        int dyn_idx = -1;
        for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
          if (dyn_joint_names_[k] == jname) {
            dyn_idx = k;
            break;
          }
        }
        if (dyn_idx < 0)
          continue;
        if (pos < q_lower_[dyn_idx] || pos > q_upper_[dyn_idx]) {
          result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
          result->error_string = "Limit exceeded: joint '" + jname +
                                 "' = " + std::to_string(pos) + " out of [" +
                                 std::to_string(q_lower_[dyn_idx]) + ", " +
                                 std::to_string(q_upper_[dyn_idx]) +
                                 "] rad at waypoint " + std::to_string(i);
          RCLCPP_ERROR(this->get_logger(), "\033[1;31m[SAFETY] %s\033[0m",
                       result->error_string.c_str());
          try {
            goal_handle->abort(result);
          } catch (...) {
          }
          return;
        }
      }
    }
  }

  // Pre-collision check per waypoint
  if (dynamics_ && dyn_state_) {
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &pt = trajectory.points[i];
      Eigen::Vector<double, ModelType::kRobotDOF> tq =
          Eigen::Vector<double, ModelType::kRobotDOF>::Zero();
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto cs = robot_->GetState();
        for (int k = 0; k < (int)dyn_joint_names_.size(); ++k) {
          for (size_t jj = 0; jj < info_.joint_infos.size(); ++jj) {
            if (info_.joint_infos[jj].name == dyn_joint_names_[k]) {
              tq[k] = cs.position[jj];
              break;
            }
          }
        }
      }
      for (size_t j = 0; j < joint_mapping.size(); ++j) {
        if (joint_mapping[j] >= 0 && j < pt.positions.size()) {
          std::string jn = info_.joint_infos[joint_mapping[j]].name;
          for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
            if (dyn_joint_names_[k] == jn) {
              tq[k] = pt.positions[j];
              break;
            }
          }
        }
      }
      auto reason = get_predicted_collision_reason(tq);
      if (reason.has_value()) {
        RCLCPP_WARN(this->get_logger(),
                    "\033[1;33m[PREDICTIVE COLLISION WARNING] %s at waypoint "
                    "%zu\033[0m",
                    reason.value().c_str(), i);
      }
    }
  }

  this->check_controll_manager();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto cur = robot_->GetState();
  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(cur.position.data(),
                                                        cur.position.size());

  try {
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &pt = trajectory.points[i];

      // Preemption / cancel check
      {
        std::lock_guard<std::mutex> lk(mutex_);
        if (goal_handle != active_follow_joint_trajectory_goal_) {
          return;
        }
      }
      if (goal_handle->is_canceling() || is_control_canceled_ ||
          !rclcpp::ok()) {
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = "Canceled";
        try {
          goal_handle->canceled(result);
        } catch (...) {
        }
        {
          std::lock_guard<std::mutex> lk(mutex_);
          if (active_follow_joint_trajectory_goal_ == goal_handle)
            active_follow_joint_trajectory_goal_ = nullptr;
        }
        return;
      }

      // Update q with this waypoint
      for (size_t j = 0; j < joint_mapping.size(); ++j) {
        if (joint_mapping[j] >= 0 && j < pt.positions.size())
          q[joint_mapping[j]] = pt.positions[j];
      }

      // Publish feedback
      auto fb = std::make_shared<FollowJointTrajectory::Feedback>();
      fb->joint_names = trajectory.joint_names;
      fb->desired = pt;
      try {
        goal_handle->publish_feedback(fb);
      } catch (...) {
      }

      double pt_time =
          (i > 0)
              ? (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 -
                 trajectory.points[i - 1].time_from_start.sec -
                 trajectory.points[i - 1].time_from_start.nanosec * 1e-9)
              : (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9);
      if (pt_time <= 0.0)
        pt_time = 0.01;

      // Build and send command directly via upper_body_stream_handler
      {
        rb::ComponentBasedCommandBuilder comp;
        rb::BodyComponentBasedCommandBuilder body;
        bool has_body = false;
        for (size_t jj = 0; jj < info_.torso_joint_idx.size(); ++jj) {
          (void)jj;
        }

        auto build_part = [&](const std::vector<unsigned int> &idx_vec,
                              size_t tgt_offset, bool is_torso, bool is_r,
                              bool is_l) {
          Eigen::VectorXd pq(idx_vec.size());
          for (size_t k = 0; k < idx_vec.size(); ++k)
            pq[k] = q[idx_vec[k]];
          rb::JointPositionCommandBuilder b;
          make_joint_pos_builder(b, pq, pt_time, 1e6);
          if (is_torso)
            body.SetTorsoCommand(rb::TorsoCommandBuilder(b));
          else if (is_r)
            body.SetRightArmCommand(rb::ArmCommandBuilder(b));
          else if (is_l)
            body.SetLeftArmCommand(rb::ArmCommandBuilder(b));
          else {
            comp.SetHeadCommand(rb::HeadCommandBuilder(b));
          }
          has_body = has_body || is_torso || is_r || is_l;
          (void)tgt_offset;
        };
        build_part(info_.torso_joint_idx, 3, true, false, false);
        build_part(info_.right_arm_joint_idx, 9, false, true, false);
        build_part(info_.left_arm_joint_idx, 16, false, false, true);
        build_part(info_.head_joint_idx, 23, false, false, false);
        if (has_body)
          comp.SetBodyCommand(rb::BodyCommandBuilder(body));

        try {
          std::lock_guard<std::mutex> sl(stream_mutex_);
          if (upper_body_stream_handler_) {
            upper_body_stream_handler_->SendCommand(
                rb::RobotCommandBuilder().SetCommand(comp));
            last_stream_command_time_ns_ = this->now().nanoseconds();
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "[FJT-Normal] SendCommand error: %s",
                       e.what());
        }
      }

      std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<long long>(pt_time * 1e6)));
    }

    // Final completion
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (active_follow_joint_trajectory_goal_ == goal_handle)
        active_follow_joint_trajectory_goal_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "FollowJointTrajectory completed.");
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    try {
      goal_handle->succeed(result);
    } catch (...) {
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[FJT] Exception: %s", e.what());
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = e.what();
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
  }
}

// --- Rby1 Joint Command Handlers ---
template <typename ModelType>
rclcpp_action::GoalResponse RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Rby1JointCommand::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received Rby1JointCommand request");
  (void)uuid;
  (void)goal; // We will do validation in execute

  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting Rby1JointCommand: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_follow_joint_trajectory_goal_ &&
        active_follow_joint_trajectory_goal_->is_active()) {
      RCLCPP_WARN(this->get_logger(),
                  "Rejecting Rby1JointCommand: FollowJointTrajectory is "
                  "currently executing.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "Received request to cancel Rby1JointCommand goal");
  (void)goal_handle;
  robot_->CancelControl();
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>>
        goal_handle) {
  using namespace std::placeholders;
  std::thread{
      std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_rby1_joint_command, this,
                _1),
      goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_rby1_joint_command(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>>
        goal_handle) {
  has_printed_collision_log_ = false;
  is_control_canceled_ = false;
  auto result = std::make_shared<Rby1JointCommand::Result>();
  const auto goal = goal_handle->get_goal();

  // Determine commanded parts
  std::vector<size_t> commanded_parts;
  if (!goal->torso.position.empty())
    commanded_parts.push_back(PART_TORSO);
  if (!goal->right_arm.position.empty())
    commanded_parts.push_back(PART_RIGHT_ARM);
  if (!goal->left_arm.position.empty())
    commanded_parts.push_back(PART_LEFT_ARM);
  if (!goal->head.position.empty())
    commanded_parts.push_back(PART_HEAD);

  if (commanded_parts.empty()) {
    result->success = false;
    result->finish_code = "Empty arrays";
    RCLCPP_WARN(this->get_logger(),
                "\033[1;33m[CONTROL REJECTED] Empty Rby1JointCommand.\033[0m");
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // ── NORMAL / NO-STREAM MODE ───────────────────────────────────────────
  // Conflict check: reject if any commanded part is already being controlled
  for (auto part : commanded_parts) {
    if (is_controlling_[part].load(std::memory_order_acquire)) {
      result->success = false;
      result->finish_code =
          "Part " + std::to_string(part) + " already being controlled";
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[CONTROL REJECTED] Part %zu already being "
                  "controlled.\033[0m",
                  part);
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // RAII: set is_controlling_ for commanded parts, clear on exit
  struct PartGuard {
    std::atomic<bool> *f;
    std::vector<size_t> p;
    PartGuard(std::atomic<bool> *flags, std::vector<size_t> parts)
        : f(flags), p(std::move(parts)) {
      for (auto i : p)
        f[i].store(true, std::memory_order_release);
    }
    ~PartGuard() {
      for (auto i : p)
        f[i].store(false, std::memory_order_release);
    }
  } guard(is_controlling_, commanded_parts);

  try {
    // Power/servo check
    auto cm_state_val = robot_->GetControlManagerState().state;
    bool is_ready = (cm_state_val == rb::ControlManagerState::State::kEnabled);
    if (!is_ready) {
      try {
        is_ready = robot_->IsPowerOn(".*") && robot_->IsServoOn(".*");
      } catch (...) {
        is_ready = false;
      }
    }
    if (!is_ready) {
      result->success = false;
      result->finish_code = "Robot is disabled (power/servo is off)";
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[CONTROL REJECTED] Robot disabled.\033[0m");
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    if (!robot_->HasEstablishedTimeSync())
      robot_->SyncTime();
    if (!this->check_controll_manager()) {
      result->success = false;
      result->finish_code = "Control Manager Error";
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31m[CONTROL MANAGER ERROR] Not ready.\033[0m");
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    // Pre-collision detection
    if (dynamics_ && dyn_state_) {
      Eigen::Vector<double, ModelType::kRobotDOF> target_q =
          Eigen::Vector<double, ModelType::kRobotDOF>::Zero();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto cs = robot_->GetState();
        for (int i = 0; i < (int)dyn_joint_names_.size(); ++i)
          for (size_t j = 0; j < info_.joint_infos.size(); ++j)
            if (info_.joint_infos[j].name == dyn_joint_names_[i]) {
              target_q[i] = cs.position[j];
              break;
            }
      }
      auto update_tq = [&](const rby1_msgs::msg::JointCommand &cmd,
                           const std::string &pn) {
        if (cmd.position.empty())
          return;
        if (cmd.use_group_joint) {
          for (size_t i = 0;
               i < cmd.joint_names.size() && i < cmd.position.size(); ++i)
            for (size_t k = 0; k < dyn_joint_names_.size(); ++k)
              if (dyn_joint_names_[k] == cmd.joint_names[i]) {
                target_q[k] = cmd.position[i];
                break;
              }
        } else {
          const std::vector<unsigned int> *pi = nullptr;
          if (pn == "torso")
            pi = &info_.torso_joint_idx;
          else if (pn == "right_arm")
            pi = &info_.right_arm_joint_idx;
          else if (pn == "left_arm")
            pi = &info_.left_arm_joint_idx;
          else if (pn == "head")
            pi = &info_.head_joint_idx;
          if (pi && cmd.position.size() == pi->size())
            for (size_t i = 0; i < cmd.position.size(); ++i) {
              std::string jn = info_.joint_infos[(*pi)[i]].name;
              for (size_t k = 0; k < dyn_joint_names_.size(); ++k)
                if (dyn_joint_names_[k] == jn) {
                  target_q[k] = cmd.position[i];
                  break;
                }
            }
        }
      };
      update_tq(goal->torso, "torso");
      update_tq(goal->right_arm, "right_arm");
      update_tq(goal->left_arm, "left_arm");
      update_tq(goal->head, "head");
      auto reason = get_predicted_collision_reason(target_q);
      if (reason.has_value()) {
        RCLCPP_WARN(this->get_logger(),
                    "\033[1;33m[PREDICTIVE COLLISION WARNING] %s\033[0m",
                    reason.value().c_str());
      }
    }

    // Build command
    rb::ComponentBasedCommandBuilder component_cmd_builder;
    rb::BodyComponentBasedCommandBuilder body_comp;
    bool use_body = false;
    bool use_head = false;
    std::string err_msg;

    if (!process_joint_part(goal->torso, "torso", info_.torso_joint_idx.size(),
                            body_comp, component_cmd_builder, use_body,
                            use_head, err_msg) ||
        !process_joint_part(
            goal->right_arm, "right_arm", info_.right_arm_joint_idx.size(),
            body_comp, component_cmd_builder, use_body, use_head, err_msg) ||
        !process_joint_part(
            goal->left_arm, "left_arm", info_.left_arm_joint_idx.size(),
            body_comp, component_cmd_builder, use_body, use_head, err_msg) ||
        !process_joint_part(goal->head, "head", info_.head_joint_idx.size(),
                            body_comp, component_cmd_builder, use_body,
                            use_head, err_msg)) {
      result->success = false;
      result->finish_code = err_msg;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
    if (!use_body && !use_head) {
      result->success = false;
      result->finish_code = "Empty arrays";
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
    if (use_body)
      component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));

    // ── STREAM MODE: send via upper_body_stream_handler, wait minimum_time ──
    if (stream_active_) {
      {
        std::lock_guard<std::mutex> sl(stream_mutex_);
        if (!upper_body_stream_handler_) {
          result->success = false;
          result->finish_code = "Stream handler lost";
          try {
            goal_handle->abort(result);
          } catch (...) {
          }
          return;
        }
        upper_body_stream_handler_->SendCommand(
            rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
        last_stream_command_time_ns_ = this->now().nanoseconds();
      }
      double max_mt =
          std::max({goal->torso.minimum_time, goal->right_arm.minimum_time,
                    goal->left_arm.minimum_time, goal->head.minimum_time});
      if (max_mt <= 0.0)
        max_mt = robot_parameter_.minimum_time;
      uint64_t my_seq = ++command_seq_;
      auto deadline = std::chrono::steady_clock::now() +
                      std::chrono::duration<double>(max_mt);
      rclcpp::Rate rate(100);
      while (rclcpp::ok()) {
        if (command_seq_.load() != my_seq) {
          result->success = false;
          result->finish_code = "kPreempted";
          try {
            goal_handle->canceled(result);
          } catch (...) {
          }
          return;
        }
        if (goal_handle->is_canceling() || is_control_canceled_) {
          result->success = false;
          result->finish_code = "kCanceled";
          try {
            goal_handle->canceled(result);
          } catch (...) {
          }
          return;
        }
        if (std::chrono::steady_clock::now() >= deadline)
          break;
        rate.sleep();
      }
      result->success = true;
      result->finish_code = "kOk";
      try {
        goal_handle->succeed(result);
      } catch (...) {
      }
      return;
    }

    // ── NO STREAM: classic blocking SendCommand ──
    auto cmd_handler = robot_->SendCommand(
        rb::RobotCommandBuilder().SetCommand(component_cmd_builder),
        goal->priority);

    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !cmd_handler->IsDone()) {
      if (goal_handle->is_canceling()) {
        cmd_handler->Cancel();
        result->success = false;
        result->finish_code = "kCanceled";
        try {
          goal_handle->canceled(result);
        } catch (...) {
        }
        return;
      }
      auto cm_state = robot_->GetControlManagerState();
      if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
          cm_state.state == rb::ControlManagerState::State::kMinorFault) {
        cmd_handler->Cancel();
        result->success = false;
        result->finish_code = "Fault Detected";
        try {
          goal_handle->abort(result);
        } catch (...) {
        }
        return;
      }
      auto feedback = std::make_shared<Rby1JointCommand::Feedback>();
      feedback->current_state = "executing";
      try {
        goal_handle->publish_feedback(feedback);
      } catch (...) {
      }
      rate.sleep();
    }

    if (rclcpp::ok()) {
      auto rv = cmd_handler->Get();
      result->finish_code = this->finish_code_to_string(rv.finish_code());
      if (is_control_canceled_)
        result->finish_code = "kCanceled";
      result->success = (result->finish_code == "kOk");
      RCLCPP_INFO(this->get_logger(), "Rby1JointCommand finished: %s",
                  result->finish_code.c_str());
      if (result->success) {
        try {
          goal_handle->succeed(result);
        } catch (...) {
        }
      } else {
        try {
          goal_handle->abort(result);
        } catch (...) {
        }
        this->check_controll_manager();
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in Rby1JointCommand: %s",
                 e.what());
    result->success = false;
    result->finish_code = "kError";
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    this->check_controll_manager();
  }
}

// --- Rby1 Cartesian Command Handlers ---
template <typename ModelType>
rclcpp_action::GoalResponse
RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Rby1CartesianCommand::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received Rby1CartesianCommand request");
  (void)uuid;
  (void)goal;

  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting Rby1CartesianCommand: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_follow_joint_trajectory_goal_ &&
        active_follow_joint_trajectory_goal_->is_active()) {
      RCLCPP_WARN(this->get_logger(),
                  "Rejecting Rby1CartesianCommand: FollowJointTrajectory is "
                  "currently executing.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "Received request to cancel Rby1CartesianCommand goal");
  (void)goal_handle;
  robot_->CancelControl();
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>>
        goal_handle) {
  using namespace std::placeholders;
  std::thread{
      std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_rby1_cartesian_command,
                this, _1),
      goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_rby1_cartesian_command(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>>
        goal_handle) {
  has_printed_collision_log_ = false;
  is_control_canceled_ = false;
  auto result = std::make_shared<Rby1CartesianCommand::Result>();
  const auto goal = goal_handle->get_goal();

  // 1. Determine commanded parts
  std::vector<size_t> commanded_parts;
  if (!goal->torso.ref_link.empty() || !goal->torso.target_link.empty())
    commanded_parts.push_back(PART_TORSO);
  if (!goal->right_arm.ref_link.empty() || !goal->right_arm.target_link.empty())
    commanded_parts.push_back(PART_RIGHT_ARM);
  if (!goal->left_arm.ref_link.empty() || !goal->left_arm.target_link.empty())
    commanded_parts.push_back(PART_LEFT_ARM);

  if (commanded_parts.empty()) {
    result->success = false;
    result->finish_code = "Empty targets";
    RCLCPP_WARN(
        this->get_logger(),
        "\033[1;33m[CONTROL REJECTED] Empty Rby1CartesianCommand.\033[0m");
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // Conflict check: reject if any commanded part is already being controlled
  for (auto part : commanded_parts) {
    if (is_controlling_[part].load(std::memory_order_acquire)) {
      result->success = false;
      result->finish_code =
          "Part " + std::to_string(part) + " already being controlled";
      RCLCPP_WARN(
          this->get_logger(),
          "\033[1;33m[CONTROL REJECTED] Part %zu already controlled.\033[0m",
          part);
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // PartGuard RAII
  struct PartGuard {
    std::atomic<bool> *f;
    std::vector<size_t> p;
    PartGuard(std::atomic<bool> *flags, std::vector<size_t> parts)
        : f(flags), p(std::move(parts)) {
      for (auto i : p)
        f[i].store(true, std::memory_order_release);
    }
    ~PartGuard() {
      for (auto i : p)
        f[i].store(false, std::memory_order_release);
    }
  } guard(is_controlling_, commanded_parts);

  try {
    // Power/servo check
    auto cm_state_val = robot_->GetControlManagerState().state;
    bool is_ready = (cm_state_val == rb::ControlManagerState::State::kEnabled);
    if (!is_ready) {
      try {
        is_ready = robot_->IsPowerOn(".*") && robot_->IsServoOn(".*");
      } catch (...) {
        is_ready = false;
      }
    }
    if (!is_ready) {
      result->success = false;
      result->finish_code = "Robot is disabled (power/servo is off)";
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[CONTROL REJECTED] Robot disabled.\033[0m");
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    if (!robot_->HasEstablishedTimeSync())
      robot_->SyncTime();

    if (!this->check_controll_manager()) {
      result->success = false;
      result->finish_code = "Control Manager Error";
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    // Pre-collision detection (IK and collision check)
    if (dynamics_ && dyn_state_) {
      std::vector<typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget>
          link_targets;
      auto add_target_helper =
          [&](const rby1_msgs::msg::CartesianCommand &cmd) -> bool {
        if (cmd.ref_link.empty() && cmd.target_link.empty())
          return true;
        if (cmd.ref_link.empty() || cmd.target_link.empty())
          return false;
        int ref_idx = get_link_index(cmd.ref_link);
        int target_idx = get_link_index(cmd.target_link);
        if (ref_idx == -1 || target_idx == -1)
          return false;

        typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget target;
        target.ref_link_index = ref_idx;
        target.link_index = target_idx;

        Eigen::Quaterniond q(cmd.transform.rotation.w, cmd.transform.rotation.x,
                             cmd.transform.rotation.y,
                             cmd.transform.rotation.z);
        Eigen::Vector3d t(cmd.transform.translation.x,
                          cmd.transform.translation.y,
                          cmd.transform.translation.z);
        rb::math::SE3::MatrixType T = rb::math::SE3::MatrixType::Identity();
        T.block(0, 0, 3, 3) = q.toRotationMatrix();
        T.block(0, 3, 3, 1) = t;
        target.T = T;
        target.weight_position = 1000.0;
        target.weight_orientation = 100.0;
        link_targets.push_back(target);
        return true;
      };

      bool ok = true;
      ok &= add_target_helper(goal->torso);
      ok &= add_target_helper(goal->right_arm);
      ok &= add_target_helper(goal->left_arm);

      if (ok && !link_targets.empty()) {
        auto solved_q = solve_cartesian_ik(link_targets);
        if (!solved_q.has_value()) {
          RCLCPP_WARN(this->get_logger(),
                      "[PREDICTIVE COLLISION CHECK] IK did not converge — "
                      "skipping collision pre-check. Command will proceed.");
        } else {
          auto reason = get_predicted_collision_reason(solved_q.value());
          if (reason.has_value()) {
            RCLCPP_WARN(this->get_logger(),
                        "\033[1;33m[PREDICTIVE COLLISION WARNING] %s\033[0m",
                        reason.value().c_str());
          }
        }
      }
    }

    // Build Cartesian Command
    rb::ComponentBasedCommandBuilder component_cmd_builder;
    rb::BodyComponentBasedCommandBuilder body_comp;
    bool use_body = false;
    std::string err_msg;

    if (!process_cartesian_part(goal->torso, "torso",
                                goal->stop_position_tracking_error,
                                goal->stop_orientation_tracking_error,
                                body_comp, use_body, err_msg) ||
        !process_cartesian_part(goal->right_arm, "right_arm",
                                goal->stop_position_tracking_error,
                                goal->stop_orientation_tracking_error,
                                body_comp, use_body, err_msg) ||
        !process_cartesian_part(goal->left_arm, "left_arm",
                                goal->stop_position_tracking_error,
                                goal->stop_orientation_tracking_error,
                                body_comp, use_body, err_msg)) {

      result->success = false;
      result->finish_code = err_msg;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    if (!use_body) {
      result->success = false;
      result->finish_code = "Empty targets";
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));

    // ── STREAM MODE: send via upper_body_stream_handler, wait minimum_time ──
    if (stream_active_) {
      {
        std::lock_guard<std::mutex> sl(stream_mutex_);
        if (!upper_body_stream_handler_) {
          result->success = false;
          result->finish_code = "Stream handler lost";
          try {
            goal_handle->abort(result);
          } catch (...) {
          }
          return;
        }
        upper_body_stream_handler_->SendCommand(
            rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
        last_stream_command_time_ns_ = this->now().nanoseconds();
      }
      double max_mt =
          std::max({goal->torso.minimum_time, goal->right_arm.minimum_time,
                    goal->left_arm.minimum_time});
      if (max_mt <= 0.0)
        max_mt = robot_parameter_.minimum_time;
      uint64_t my_seq = ++command_seq_;
      auto deadline = std::chrono::steady_clock::now() +
                      std::chrono::duration<double>(max_mt);
      rclcpp::Rate rate(100);
      while (rclcpp::ok()) {
        if (command_seq_.load() != my_seq) {
          result->success = false;
          result->finish_code = "kPreempted";
          try {
            goal_handle->canceled(result);
          } catch (...) {
          }
          return;
        }
        if (goal_handle->is_canceling() || is_control_canceled_) {
          result->success = false;
          result->finish_code = "kCanceled";
          try {
            goal_handle->canceled(result);
          } catch (...) {
          }
          return;
        }
        if (std::chrono::steady_clock::now() >= deadline)
          break;
        rate.sleep();
      }
      result->success = true;
      result->finish_code = "kOk";
      try {
        goal_handle->succeed(result);
      } catch (...) {
      }
      return;
    }

    // ── NO STREAM: classic blocking SendCommand ──
    auto cmd_handler = robot_->SendCommand(
        rb::RobotCommandBuilder().SetCommand(component_cmd_builder),
        goal->priority);

    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !cmd_handler->IsDone()) {
      if (goal_handle->is_canceling()) {
        cmd_handler->Cancel();
        result->success = false;
        result->finish_code = "kCanceled";
        try {
          goal_handle->canceled(result);
        } catch (...) {
        }
        return;
      }

      auto cm_state = robot_->GetControlManagerState();
      if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
          cm_state.state == rb::ControlManagerState::State::kMinorFault) {
        cmd_handler->Cancel();
        result->success = false;
        result->finish_code = "Fault Detected";
        try {
          goal_handle->abort(result);
        } catch (...) {
        }
        return;
      }
      rate.sleep();
    }

    if (rclcpp::ok()) {
      auto rv = cmd_handler->Get();
      result->finish_code = this->finish_code_to_string(rv.finish_code());
      if (is_control_canceled_)
        result->finish_code = "kCanceled";
      result->success = (result->finish_code == "kOk");

      RCLCPP_INFO(this->get_logger(),
                  "Rby1CartesianCommand finished with code: %s",
                  result->finish_code.c_str());
      if (result->success) {
        try {
          goal_handle->succeed(result);
        } catch (...) {
        }
      } else {
        try {
          goal_handle->abort(result);
        } catch (...) {
        }
        this->check_controll_manager();
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in Rby1CartesianCommand: %s",
                 e.what());
    result->success = false;
    result->finish_code = "kError";
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    this->check_controll_manager();
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::resize_joint_states() {
  constexpr size_t kTorsoDOF = 6;
  constexpr size_t kArmDOF = 7;
  constexpr size_t kHeadDOF = 2;

  auto resize_js = [](JointState &js, size_t n) {
    js.name.assign(n, "");
    js.position.assign(n, 0.0);
    js.velocity.assign(n, 0.0);
    js.effort.assign(n, 0.0);
  };

  resize_js(robot_joint_.joint_torso, kTorsoDOF);
  resize_js(robot_joint_.joint_right_arm, kArmDOF);
  resize_js(robot_joint_.joint_left_arm, kArmDOF);
  resize_js(robot_joint_.joint_head, kHeadDOF);

  size_t kMobilityDOF = info_.mobility_joint_idx.size();
  if (kMobilityDOF == 0) {
    if (model == "a" || model == "A")
      kMobilityDOF = 2;
    else if (model == "m" || model == "M")
      kMobilityDOF = 4;
  }
  resize_js(robot_joint_.joint_wheel, kMobilityDOF);
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
    auto state = robot_->GetState();
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
void RBY1_ROS2_DRIVER<ModelType>::control_manager_callback(
    const std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Request>
        request,
    std::shared_ptr<rby1_msgs::srv::ControlManagerCommand::Response> response) {

  try {
    if (request->command ==
        rby1_msgs::srv::ControlManagerCommand::Request::CMD_ENABLE) {
      RCLCPP_INFO(this->get_logger(),
                  "Received Control Manager ENABLE request");
      if (robot_->EnableControlManager()) {
        response->success = true;
        response->message = "Control Manager enabled successfully.";
      } else {
        response->success = false;
        response->message = "Failed to enable Control Manager.";
      }
    } else if (request->command ==
               rby1_msgs::srv::ControlManagerCommand::Request::CMD_DISABLE) {
      RCLCPP_INFO(this->get_logger(),
                  "Received Control Manager DISABLE request");
      if (robot_->DisableControlManager()) {
        {
          std::lock_guard<std::mutex> stream_lock(stream_mutex_);
          if (upper_body_stream_handler_) {
            upper_body_stream_handler_.reset();
          }
          if (mobility_stream_handler_) {
            mobility_stream_handler_.reset();
          }
          stream_active_ = false;
        }
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
          if (upper_body_stream_handler_) {
            upper_body_stream_handler_.reset();
          }
          if (mobility_stream_handler_) {
            mobility_stream_handler_.reset();
          }
          stream_active_ = false;
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
void RBY1_ROS2_DRIVER<ModelType>::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  has_printed_collision_log_ = false;
  try {
    {
      std::lock_guard<std::mutex> lock(stream_mutex_);
      if (hardware_control_active_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "cmd_vel received but hardware control is active. "
                             "Ignoring command.");
        return;
      }
      if (!stream_active_ || !mobility_stream_handler_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "cmd_vel received but mobility stream control is "
                             "not active. Ignoring command.");
        return;
      }
    }

    double hz = stream_hz_;
    if (hz <= 0.0)
      hz = 15.0;
    double minimum_time = (1.0 / hz) * 1.1;

    rb::ComponentBasedCommandBuilder comp_builder;
    rb::MobilityCommandBuilder mobility_cmd;
    build_mobility_cmd(mobility_cmd, msg->linear.x, msg->linear.y,
                       msg->angular.z, minimum_time);
    comp_builder.SetMobilityCommand(mobility_cmd);

    rb::RobotCommandBuilder robot_cmd;
    robot_cmd.SetCommand(comp_builder);

    std::lock_guard<std::mutex> lock(stream_mutex_);
    if (stream_active_ && mobility_stream_handler_) {
      mobility_stream_handler_->SendCommand(robot_cmd);
      last_stream_command_time_ns_ = this->now().nanoseconds();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in cmd_vel_callback: %s",
                 e.what());
    deactivate_all_streams();
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::stream_control_callback(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request->state) {
    bool is_gc_active = gravity_compensation_torso_ ||
                        gravity_compensation_right_arm_ ||
                        gravity_compensation_left_arm_;
    if (is_gc_active) {
      response->success = false;
      response->message =
          "Failed to activate stream control: Gravity compensation is active.";
      RCLCPP_WARN(this->get_logger(),
                  "stream_control: Rejecting activation request because "
                  "gravity compensation is active.");
      return;
    }

    this->get_parameter("stream_hz", stream_hz_);
    if (stream_hz_ <= 0.0)
      stream_hz_ = 15.0;

    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (stream_active_ && upper_body_stream_handler_ &&
        mobility_stream_handler_) {
      response->success = true;
      response->message = "Persistent stream control is already active.";
      RCLCPP_INFO(
          this->get_logger(),
          "Persistent stream control is already active. Ignoring request.");
      return;
    }
    try {
      // When stream is ON, create command streams
      upper_body_stream_handler_ = robot_->CreateCommandStream(10);
      mobility_stream_handler_ = robot_->CreateCommandStream(10);

      last_stream_command_time_ns_ = this->now().nanoseconds();
      stream_active_ = true;

      RCLCPP_INFO(this->get_logger(), "Persistent stream control activated.");
      response->success = true;
      response->message = "Persistent stream control activated successfully.";
    } catch (const std::exception &e) {
      response->success = false;
      response->message =
          std::string("Failed to activate stream control: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  } else {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    stream_active_ = false;
    if (upper_body_stream_handler_) {
      upper_body_stream_handler_.reset();
    }
    if (mobility_stream_handler_) {
      mobility_stream_handler_.reset();
    }
    for (size_t i = 0; i < PART_COUNT; ++i) {
      is_controlling_[i].store(false, std::memory_order_release);
    }
    response->success = true;
    response->message = "Persistent stream control deactivated.";
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::deactivate_all_streams() {
  std::lock_guard<std::mutex> stream_lock(stream_mutex_);
  if (stream_active_) {
    RCLCPP_WARN(this->get_logger(),
                "\033[1;31m[STREAM SAFETY WARNING] Stream failure detected! "
                "Deactivating all streams.\033[0m");
    stream_active_ = false;
    if (upper_body_stream_handler_) {
      try {
        upper_body_stream_handler_->Cancel();
      } catch (...) {
      }
      upper_body_stream_handler_.reset();
    }
    if (mobility_stream_handler_) {
      try {
        mobility_stream_handler_->Cancel();
      } catch (...) {
      }
      mobility_stream_handler_.reset();
    }
    for (size_t i = 0; i < PART_COUNT; ++i) {
      is_controlling_[i].store(false, std::memory_order_release);
    }
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
void RBY1_ROS2_DRIVER<ModelType>::check_stream_safety() {
  if (stream_active_) {
    uint64_t now_ns = this->now().nanoseconds();
    uint64_t last_ns = last_stream_command_time_ns_.load();
    double elapsed = static_cast<double>(now_ns - last_ns) * 1e-9;
    if (elapsed > 1.0) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;31m[STREAM TIMEOUT] No stream commands received for "
                  "%.2f seconds! Deactivating all streams.\033[0m",
                  elapsed);
      deactivate_all_streams();
      if (hardware_control_active_) {
        hardware_control_active_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "Hardware control released due to stream timeout.");
      }
    }
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::set_trajectory_impedance_callback(
    const std::shared_ptr<rby1_msgs::srv::SetTrajectoryImpedance::Request>
        request,
    std::shared_ptr<rby1_msgs::srv::SetTrajectoryImpedance::Response>
        response) {
  std::lock_guard<std::mutex> lock(mutex_);

  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (stream_active_) {
      response->success = false;
      response->message = "Cannot configure trajectory impedance while persistent stream is active. Please close the stream first.";
      RCLCPP_WARN(this->get_logger(), "[TRAJECTORY IMPEDANCE] %s", response->message.c_str());
      return;
    }
  }

  // For Torso
  bool torso_active = (request->state.size() > 0) ? request->state[0] : false;
  if (torso_active) {
    trajectory_impedance_enabled_torso_ = true;
    if (!request->torso_stiffness.empty()) {
      if (request->torso_stiffness.size() != info_.torso_joint_idx.size()) {
        response->success = false;
        response->message = "Invalid torso stiffness size: expected " +
                            std::to_string(info_.torso_joint_idx.size()) +
                            ", got " +
                            std::to_string(request->torso_stiffness.size());
        RCLCPP_ERROR(this->get_logger(), "[TRAJECTORY IMPEDANCE] %s",
                     response->message.c_str());
        return;
      }
      trajectory_stiffness_torso_ = request->torso_stiffness;
    } else {
      trajectory_stiffness_torso_.assign(info_.torso_joint_idx.size(), 100.0);
    }
  } else {
    trajectory_impedance_enabled_torso_ = false;
  }

  // For Right Arm
  bool right_arm_active =
      (request->state.size() > 1) ? request->state[1] : false;
  if (right_arm_active) {
    trajectory_impedance_enabled_right_arm_ = true;
    if (!request->right_arm_stiffness.empty()) {
      if (request->right_arm_stiffness.size() !=
          info_.right_arm_joint_idx.size()) {
        response->success = false;
        response->message = "Invalid right arm stiffness size: expected " +
                            std::to_string(info_.right_arm_joint_idx.size()) +
                            ", got " +
                            std::to_string(request->right_arm_stiffness.size());
        RCLCPP_ERROR(this->get_logger(), "[TRAJECTORY IMPEDANCE] %s",
                     response->message.c_str());
        return;
      }
      trajectory_stiffness_right_arm_ = request->right_arm_stiffness;
    } else {
      trajectory_stiffness_right_arm_.assign(info_.right_arm_joint_idx.size(),
                                             100.0);
    }
  } else {
    trajectory_impedance_enabled_right_arm_ = false;
  }

  // For Left Arm
  bool left_arm_active =
      (request->state.size() > 2) ? request->state[2] : false;
  if (left_arm_active) {
    trajectory_impedance_enabled_left_arm_ = true;
    if (!request->left_arm_stiffness.empty()) {
      if (request->left_arm_stiffness.size() !=
          info_.left_arm_joint_idx.size()) {
        response->success = false;
        response->message = "Invalid left arm stiffness size: expected " +
                            std::to_string(info_.left_arm_joint_idx.size()) +
                            ", got " +
                            std::to_string(request->left_arm_stiffness.size());
        RCLCPP_ERROR(this->get_logger(), "[TRAJECTORY IMPEDANCE] %s",
                     response->message.c_str());
        return;
      }
      trajectory_stiffness_left_arm_ = request->left_arm_stiffness;
    } else {
      trajectory_stiffness_left_arm_.assign(info_.left_arm_joint_idx.size(),
                                            100.0);
    }
  } else {
    trajectory_impedance_enabled_left_arm_ = false;
  }

  trajectory_damping_ratio_ =
      (!request->damping_ratio.empty()) ? request->damping_ratio[0] : 1.0;
  trajectory_torque_limit_ =
      (!request->torque_limit.empty()) ? request->torque_limit[0] : 10.0;

  response->success = true;
  response->message =
      "Trajectory impedance configured. Torso=" +
      std::to_string(trajectory_impedance_enabled_torso_) +
      ", RightArm=" + std::to_string(trajectory_impedance_enabled_right_arm_) +
      ", LeftArm=" + std::to_string(trajectory_impedance_enabled_left_arm_);
  RCLCPP_INFO(this->get_logger(), "[TRAJECTORY IMPEDANCE] Configured: %s",
              response->message.c_str());
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
  auto cols = dynamics_->DetectCollisionsOrNearestLinks(temp_state, 20);

  for (const auto &col : cols) {
    if (col.distance < collision_threshold_) {
      std::stringstream ss;
      ss << "Collision predicted between [" << col.link1 << "] and ["
         << col.link2 << "] (distance: " << std::fixed << std::setprecision(4)
         << col.distance << " m < threshold: " << collision_threshold_ << " m)";
      return ss.str();
    }
  }
  return std::nullopt;
}

template <typename ModelType>
std::optional<Eigen::VectorXd> RBY1_ROS2_DRIVER<ModelType>::solve_cartesian_ik(
    const std::vector<
        typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget>
        &link_targets) {

  if (!dynamics_ || !dyn_state_) {
    return std::nullopt;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Clone current state as starting position for numerical IK
  auto temp_state =
      std::make_shared<rb::dyn::State<ModelType::kRobotDOF>>(*dyn_state_);

  // Print starting joint configuration for debugging
  std::stringstream ss_q;
  ss_q << "[";
  for (int i = 0; i < temp_state->GetQ().size(); ++i) {
    ss_q << temp_state->GetQ()[i]
         << (i == temp_state->GetQ().size() - 1 ? "" : ", ");
  }
  ss_q << "]";
  RCLCPP_WARN(this->get_logger(), "IK start Q: %s", ss_q.str().c_str());

  // Setup OptimalControl solver
  // Exclude joints with near-infinite limits (e.g. wheel joints whose URDF
  // limit is DBL_MAX). Including them makes the QP ill-conditioned and causes
  // divergence unrelated to the arm workspace being checked.
  constexpr double kFiniteLimitThreshold = 1e6;
  std::vector<unsigned int> joint_idx;
  joint_idx.reserve(ModelType::kRobotDOF);
  for (unsigned int i = 0; i < ModelType::kRobotDOF; ++i) {
    if (q_lower_[i] > -kFiniteLimitThreshold &&
        q_upper_[i] < kFiniteLimitThreshold) {
      joint_idx.push_back(i);
    }
  }
  if (joint_idx.empty()) {
    // Fallback: use all joints if none passed the filter
    joint_idx.resize(ModelType::kRobotDOF);
    std::iota(joint_idx.begin(), joint_idx.end(), 0);
  }
  rb::OptimalControl<ModelType::kRobotDOF> ik_solver(dynamics_, joint_idx);

  double dt = 0.1;
  int max_iterations = 50;
  double tolerance = 1e-3;
  bool converged = false;

  typename rb::OptimalControl<ModelType::kRobotDOF>::Input in;
  in.link_targets = link_targets;

  Eigen::Vector<double, ModelType::kRobotDOF> q_lb =
      q_lower_ - Eigen::Vector<double, ModelType::kRobotDOF>::Constant(0.5);
  Eigen::Vector<double, ModelType::kRobotDOF> q_ub =
      q_upper_ + Eigen::Vector<double, ModelType::kRobotDOF>::Constant(0.5);
  Eigen::Vector<double, ModelType::kRobotDOF> qdot_ub =
      Eigen::Vector<double, ModelType::kRobotDOF>::Constant(100.0);
  Eigen::Vector<double, ModelType::kRobotDOF> qddot_ub =
      Eigen::Vector<double, ModelType::kRobotDOF>::Constant(1000.0);

  auto debug_joint_names = temp_state->GetJointNames();
  for (size_t i = 0; i < debug_joint_names.size(); ++i) {
    RCLCPP_WARN(this->get_logger(),
                "Joint %zu (%s): Q=%.4f, Limit=[%.4f, %.4f]", i,
                std::string(debug_joint_names[i]).c_str(),
                temp_state->GetQ()[i], q_lb[i], q_ub[i]);
  }

  // Compute and print initial T_cur and T_err for targets
  for (size_t i = 0; i < link_targets.size(); ++i) {
    const auto &t = link_targets[i];
    Eigen::Matrix4d T_cur = dynamics_->ComputeTransformation(
        temp_state, t.ref_link_index, t.link_index);
    Eigen::Matrix4d T_err = T_cur.inverse() * t.T;
    RCLCPP_WARN(this->get_logger(),
                "Target %zu initial T_cur translation: [%.4f, %.4f, %.4f]", i,
                T_cur(0, 3), T_cur(1, 3), T_cur(2, 3));
    RCLCPP_WARN(this->get_logger(),
                "Target %zu initial T_err translation: [%.4f, %.4f, %.4f]", i,
                T_err(0, 3), T_err(1, 3), T_err(2, 3));
  }

  int final_iter = 0;
  for (int iter = 0; iter < max_iterations; ++iter) {
    final_iter = iter;
    auto sol = ik_solver.Solve(in, temp_state, dt, 1.0, q_lb, q_ub, qdot_ub,
                               qddot_ub, true);
    if (!sol.has_value()) {
      RCLCPP_WARN(
          this->get_logger(),
          "Iteration %d: Solve returned no value. ExitCode: %d, Msg: %s", iter,
          (int)ik_solver.GetExitCode(), ik_solver.GetExitCodeMessage().c_str());
      break;
    }

    // Clamp joint velocities to velocity limits to ensure numerical stability
    Eigen::VectorXd clamped_sol =
        sol.value().cwiseMax(-qdot_ub).cwiseMin(qdot_ub);

    // Step joint angles using trapezoidal integration (matches SDK solver
    // constraints)
    Eigen::VectorXd q_next =
        temp_state->GetQ() + 0.5 * (temp_state->GetQdot() + clamped_sol) * dt;
    temp_state->SetQ(q_next);
    temp_state->SetQdot(clamped_sol);

    if (ik_solver.GetError() < tolerance) {
      converged = true;
      break;
    }
  }
  RCLCPP_WARN(this->get_logger(),
              "IK loop finished at iteration %d, converged: %d", final_iter,
              converged);

  if (converged) {
    return temp_state->GetQ();
  }

  RCLCPP_WARN(this->get_logger(),
              "IK solver failed to converge. Targets count: %zu",
              link_targets.size());
  for (size_t i = 0; i < link_targets.size(); ++i) {
    const auto &t = link_targets[i];
    Eigen::Matrix4d T_mat = t.T;
    RCLCPP_WARN(this->get_logger(),
                "  Target %zu: ref_link_idx=%d, link_idx=%d", i,
                t.ref_link_index, t.link_index);
    RCLCPP_WARN(this->get_logger(), "    T translation: [%.4f, %.4f, %.4f]",
                T_mat(0, 3), T_mat(1, 3), T_mat(2, 3));
    Eigen::Matrix3d R_mat = T_mat.block(0, 0, 3, 3);
    Eigen::Quaterniond q_rot(R_mat);
    RCLCPP_WARN(this->get_logger(),
                "    T rotation: x=%.4f, y=%.4f, z=%.4f, w=%.4f", q_rot.x(),
                q_rot.y(), q_rot.z(), q_rot.w());
  }
  RCLCPP_WARN(this->get_logger(),
              "  Final solver error: %.6f (tolerance: %.6f)",
              ik_solver.GetError(), tolerance);

  return std::nullopt;
}

template <typename ModelType>
bool RBY1_ROS2_DRIVER<ModelType>::process_joint_part(
    const rby1_msgs::msg::JointCommand &cmd, const std::string &part_name,
    size_t expected_dof, rb::BodyComponentBasedCommandBuilder &body_comp,
    rb::ComponentBasedCommandBuilder &component_cmd_builder, bool &use_body,
    bool &use_head, std::string &err_msg) {
  if (cmd.position.empty())
    return true;

  double hold_time = cmd.control_hold_time;
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (stream_active_) {
      hold_time = 1e6;
    }
  }

  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(cmd.position.data(),
                                                        cmd.position.size());
  Eigen::VectorXd clamped_vel_limits(cmd.position.size());
  Eigen::VectorXd clamped_accel_limits(cmd.position.size());

  for (size_t i = 0; i < cmd.position.size(); ++i) {
    clamped_vel_limits[i] = cmd.velocity_limit;
    clamped_accel_limits[i] = cmd.acceleration_limit;
  }

  if (dynamics_ && dyn_state_ && q_lower_.size() > 0) {
    if (cmd.use_group_joint) {
      if (cmd.position.size() == cmd.joint_names.size()) {
        for (size_t i = 0; i < cmd.joint_names.size(); ++i) {
          std::string commanded_name = cmd.joint_names[i];

          int pinocchio_idx = -1;
          for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
            if (dyn_joint_names_[k] == commanded_name) {
              pinocchio_idx = k;
              break;
            }
          }
          if (pinocchio_idx == -1)
            continue;

          double pos = cmd.position[i];
          double min_p = q_lower_[pinocchio_idx];
          double max_p = q_upper_[pinocchio_idx];

          if (pos < min_p || pos > max_p) {
            err_msg = "Safety Limit Exceeded: Commanded position for joint '" +
                      commanded_name + "' (" + std::to_string(pos) +
                      " rad) is out of safety range [" + std::to_string(min_p) +
                      ", " + std::to_string(max_p) + "] rad.";
            RCLCPP_ERROR(this->get_logger(),
                         "\033[1;31m[SAFETY ERROR] %s\033[0m", err_msg.c_str());
            return false;
          }
        }
      }
    } else {
      const std::vector<unsigned int> *part_indices = nullptr;
      if (part_name == "torso")
        part_indices = &info_.torso_joint_idx;
      else if (part_name == "right_arm")
        part_indices = &info_.right_arm_joint_idx;
      else if (part_name == "left_arm")
        part_indices = &info_.left_arm_joint_idx;
      else if (part_name == "head")
        part_indices = &info_.head_joint_idx;

      if (part_indices && cmd.position.size() == part_indices->size()) {
        for (size_t i = 0; i < cmd.position.size(); ++i) {
          size_t global_idx = (*part_indices)[i];
          std::string joint_name = info_.joint_infos[global_idx].name;

          int pinocchio_idx = -1;
          for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
            if (dyn_joint_names_[k] == joint_name) {
              pinocchio_idx = k;
              break;
            }
          }
          if (pinocchio_idx == -1)
            continue;

          double pos = cmd.position[i];
          double min_p = q_lower_[pinocchio_idx];
          double max_p = q_upper_[pinocchio_idx];

          if (pos < min_p || pos > max_p) {
            err_msg = "Safety Limit Exceeded: Commanded position for joint '" +
                      joint_name + "' (" + std::to_string(pos) +
                      " rad) is out of safety range [" + std::to_string(min_p) +
                      ", " + std::to_string(max_p) + "] rad.";
            RCLCPP_ERROR(this->get_logger(),
                         "\033[1;31m[SAFETY ERROR] %s\033[0m", err_msg.c_str());
            return false;
          }
        }
      }
    }
  }

  if (cmd.use_group_joint && part_name == "torso") {
    if (cmd.joint_names.empty()) {
      err_msg = "Empty Joint Names: torso joint_names cannot be empty when "
                "use_group_joint is True.";
      RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
      return false;
    }
    if (cmd.position.size() != cmd.joint_names.size()) {
      err_msg = "Invalid Array Size: torso position size (" +
                std::to_string(cmd.position.size()) +
                ") must match joint_names size (" +
                std::to_string(cmd.joint_names.size()) +
                ") for Joint Group Control.";
      RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
      return false;
    }
    auto b = rb::JointGroupPositionCommandBuilder();
    b.SetJointNames(cmd.joint_names);
    b.SetPosition(q);
    b.SetMinimumTime(cmd.minimum_time);
    if (cmd.velocity_limit > 0.0) {
      b.SetVelocityLimit(clamped_vel_limits);
    }
    if (cmd.acceleration_limit > 0.0) {
      b.SetAccelerationLimit(clamped_accel_limits);
    }
    body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
    use_body = true;
    return true;
  }

  if (cmd.use_group_joint) {
    RCLCPP_WARN(this->get_logger(),
                "Joint group control is only supported for torso. Falling back "
                "to standard control for %s",
                part_name.c_str());
  }

  if (cmd.position.size() != expected_dof) {
    err_msg = "Invalid Array Size: " + part_name + " position requires " +
              std::to_string(expected_dof) + " elements, got " +
              std::to_string(cmd.position.size());
    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
    return false;
  }

  // Head early-return
  if (part_name == "head") {
    rb::JointPositionCommandBuilder b;
    make_joint_pos_builder(b, q, cmd.minimum_time, hold_time,
                           cmd.velocity_limit, cmd.acceleration_limit);
    component_cmd_builder.SetHeadCommand(rb::HeadCommandBuilder(b));
    use_head = true;
    return true;
  }

  if (cmd.use_impedance) {
    std::vector<double> stiffness = cmd.stiffness;
    if (stiffness.empty()) {
      stiffness.assign(expected_dof, 100.0);
    } else if (stiffness.size() != expected_dof) {
      err_msg = "Invalid Array Size: " + part_name + " stiffness requires " +
                std::to_string(expected_dof) + " elements, got " +
                std::to_string(stiffness.size());
      RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
      return false;
    }
    Eigen::VectorXd stiffness_vec =
        Eigen::Map<const Eigen::VectorXd>(stiffness.data(), stiffness.size());

    rb::JointImpedanceControlCommandBuilder b;
    make_joint_impedance_builder(
        b, q, cmd.minimum_time, hold_time, stiffness_vec, cmd.damping_ratio,
        cmd.torque_limit, cmd.velocity_limit, cmd.acceleration_limit);

    if (part_name == "torso")
      body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
    else if (part_name == "right_arm")
      body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "left_arm")
      body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
    use_body = true;
  } else {
    rb::JointPositionCommandBuilder b;
    make_joint_pos_builder(b, q, cmd.minimum_time, hold_time,
                           cmd.velocity_limit, cmd.acceleration_limit);

    if (part_name == "torso")
      body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
    else if (part_name == "right_arm")
      body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "left_arm")
      body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
    use_body = true;
  }
  return true;
}

template <typename ModelType>
bool RBY1_ROS2_DRIVER<ModelType>::process_cartesian_part(
    const rby1_msgs::msg::CartesianCommand &cmd, const std::string &part_name,
    double stop_position_tracking_error, double stop_orientation_tracking_error,
    rb::BodyComponentBasedCommandBuilder &body_comp, bool &use_body,
    std::string &err_msg) {
  if (cmd.ref_link.empty() && cmd.target_link.empty())
    return true;

  double hold_time = cmd.control_hold_time;
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (stream_active_) {
      hold_time = 1e6;
    }
  }

  if (cmd.ref_link.empty() || cmd.target_link.empty()) {
    err_msg = "Invalid CartesianCommand: Both ref_link and target_link must be "
              "provided for " +
              part_name;
    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
    return false;
  }

  auto get_weight = [](const std::vector<double> &input, double default_val,
                       const std::string &err_prefix,
                       std::string &err) -> std::optional<Eigen::Vector3d> {
    if (input.empty())
      return Eigen::Vector3d(default_val, default_val, default_val);
    if (input.size() != 3) {
      err = err_prefix + " requires 3 elements, got " +
            std::to_string(input.size());
      return std::nullopt;
    }
    return Eigen::Vector3d(input[0], input[1], input[2]);
  };

  auto t_weight = get_weight(
      cmd.translation_weight, 1000.0,
      "Invalid Array Size: " + part_name + " translation_weight", err_msg);
  if (!t_weight)
    return false;

  auto r_weight = get_weight(
      cmd.rotation_weight, 100.0,
      "Invalid Array Size: " + part_name + " rotation_weight", err_msg);
  if (!r_weight)
    return false;

  Eigen::Quaterniond q(cmd.transform.rotation.w, cmd.transform.rotation.x,
                       cmd.transform.rotation.y, cmd.transform.rotation.z);
  Eigen::Vector3d t(cmd.transform.translation.x, cmd.transform.translation.y,
                    cmd.transform.translation.z);

  rb::math::SE3::MatrixType T = rb::math::SE3::MatrixType::Identity();
  T.block(0, 0, 3, 3) = q.toRotationMatrix();
  T.block(0, 3, 3, 1) = t;

  if (cmd.use_impedance) {
    auto b = rb::ImpedanceControlCommandBuilder();
    b.SetCommandHeader(
        rb::CommandHeaderBuilder().SetControlHoldTime(hold_time));
    b.SetReferenceLinkName(cmd.ref_link);
    b.SetLinkName(cmd.target_link);
    b.SetTranslationWeight(*t_weight);
    b.SetRotationWeight(*r_weight);
    b.SetTransformation(T);

    if (cmd.add_joint_position_target) {
      RCLCPP_WARN(this->get_logger(),
                  "add_joint_position_target is not supported for Cartesian "
                  "Impedance Control in this SDK version. Ignoring for %s",
                  part_name.c_str());
    }

    if (part_name == "right_arm")
      body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "left_arm")
      body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "torso")
      body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));

    use_body = true;
    return true;
  } else {
    auto b = rb::CartesianCommandBuilder();
    b.SetCommandHeader(
        rb::CommandHeaderBuilder().SetControlHoldTime(hold_time));
    b.SetMinimumTime(cmd.minimum_time);

    double lin_vel = (cmd.linear_velocity_limit <= 0.0)
                         ? robot_parameter_.linear_velocity_limit
                         : cmd.linear_velocity_limit;
    double ang_vel = (cmd.angular_velocity_limit <= 0.0)
                         ? robot_parameter_.angular_velocity_limit
                         : cmd.angular_velocity_limit;
    double acc_scale = (cmd.acceleration_limit_scaling <= 0.0)
                           ? robot_parameter_.acceleration_limit
                           : cmd.acceleration_limit_scaling;

    b.AddTarget(cmd.ref_link, cmd.target_link, T, lin_vel, ang_vel, acc_scale);
    b.SetStopPositionTrackingError(stop_position_tracking_error);
    b.SetStopOrientationTrackingError(stop_orientation_tracking_error);

    if (cmd.add_joint_position_target) {
      int pinocchio_idx = -1;
      for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
        if (dyn_joint_names_[k] == cmd.add_joint_name) {
          pinocchio_idx = k;
          break;
        }
      }
      if (pinocchio_idx != -1) {
        double min_p = q_lower_[pinocchio_idx];
        double max_p = q_upper_[pinocchio_idx];
        if (cmd.add_joint_value < min_p || cmd.add_joint_value > max_p) {
          err_msg = "Safety Limit Exceeded: Commanded additional joint '" +
                    cmd.add_joint_name + "' value (" +
                    std::to_string(cmd.add_joint_value) +
                    " rad) is out of safety range [" + std::to_string(min_p) +
                    ", " + std::to_string(max_p) + "] rad.";
          RCLCPP_ERROR(this->get_logger(), "\033[1;31m[SAFETY ERROR] %s\033[0m",
                       err_msg.c_str());
          return false;
        }
      }
      b.AddJointPositionTarget(cmd.add_joint_name, cmd.add_joint_value);
    }

    if (part_name == "right_arm")
      body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "left_arm")
      body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "torso")
      body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));

    use_body = true;
    return true;
  }
}

template <typename ModelType>
rclcpp_action::GoalResponse
RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const StreamJoint::Goal> goal) {
  (void)uuid;
  (void)goal;
  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting StreamJoint: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamJoint>>
        goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamJoint>>
        goal_handle) {
  std::thread{
      std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_stream_joint, this, _1),
      goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_stream_joint(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamJoint>>
        goal_handle) {
  has_printed_collision_log_ = false;

  auto result = std::make_shared<StreamJoint::Result>();
  const auto goal = goal_handle->get_goal();
  const auto &cmd = goal->command;

  // 1. Check if stream is active
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (!stream_active_ || !upper_body_stream_handler_) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[STREAM REJECTED] stream_joint ignored: Stream is "
                  "not active.\033[0m");
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // 2. Check if trajectory is executing
  if (active_follow_joint_trajectory_goal_ &&
      active_follow_joint_trajectory_goal_->is_active()) {
    RCLCPP_WARN(this->get_logger(),
                "\033[1;33m[STREAM REJECTED] stream_joint ignored: Trajectory "
                "execution is active.\033[0m");
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // Determine commanded parts
  std::vector<size_t> commanded_parts;
  if (!cmd.torso.position.empty())
    commanded_parts.push_back(PART_TORSO);
  if (!cmd.right_arm.position.empty())
    commanded_parts.push_back(PART_RIGHT_ARM);
  if (!cmd.left_arm.position.empty())
    commanded_parts.push_back(PART_LEFT_ARM);
  if (!cmd.head.position.empty())
    commanded_parts.push_back(PART_HEAD);

  if (commanded_parts.empty()) {
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // 3. Check for non-stream control conflict
  for (auto part : commanded_parts) {
    if (is_controlling_[part].load(std::memory_order_acquire)) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[STREAM REJECTED] stream_joint ignored: Part %zu "
                  "is controlled by non-stream command.\033[0m",
                  part);
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // // 4. Calculate required acceleration and verify limits
  // uble dt = part_cmd.minimum_time;
  //     iauto check_accel_limit = [&](const rby1_msgs::msg::JointCommand&
  //     part_cmd, const std::vector<unsigned int>& joint_indices, const
  //     std::string& part_name) -> bool { if (part_cmd.position.empty()) return
  //     true;

  //     dof (dt <= 0.0) dt = 0.1;

  //     auto state = robot_->GetState();

  //     for (size_t i = 0; i < part_cmd.position.size() && i <
  //     joint_indices.size(); ++i) {
  //         unsigned int sdk_idx = joint_indices[i];
  //         std::string joint_name = info_.joint_infos[sdk_idx].name;
  //         //double current_pos = state.position[sdk_idx];
  //         //double target_pos = part_cmd.position[i];

  //         //double req_accel = 2.0 * std::abs(target_pos - current_pos) / (dt
  //         * dt);

  //         double limit = robot_parameter_.acceleration_limit;
  //         if (dynamics_ && dyn_state_) {
  //             auto it = std::find(dyn_joint_names_.begin(),
  //             dyn_joint_names_.end(), joint_name); if (it !=
  //             dyn_joint_names_.end()) {
  //                 int dyn_idx = std::distance(dyn_joint_names_.begin(), it);
  //                 limit = qddot_upper_[dyn_idx];
  //             }
  //         }

  //         // if (req_accel > limit) {
  //         //     RCLCPP_WARN(this->get_logger(),
  //         //                 "\033[1;31m[STREAM REJECTED] Joint '%s' (%s)
  //         required acceleration (%.4f rad/s^2) exceeds limit (%.4f rad/s^2)
  //         with dt=%.4f s. Current: %.4f, Target: %.4f\033[0m",
  //         //                 joint_name.c_str(), part_name.c_str(),
  //         req_accel, limit, dt, current_pos, target_pos);
  //         //     //return false;
  //         // }
  //     }
  //     return true;
  // };

  // if (!check_accel_limit(cmd.torso, info_.torso_joint_idx, "torso") ||
  //     !check_accel_limit(cmd.right_arm, info_.right_arm_joint_idx,
  //     "right_arm") || !check_accel_limit(cmd.left_arm,
  //     info_.left_arm_joint_idx, "left_arm") || !check_accel_limit(cmd.head,
  //     info_.head_joint_idx, "head")) { result->success = false; try {
  //     goal_handle->abort(result); } catch (...) {} return;
  // }

  // 5. Build and send command
  try {
    rb::BodyComponentBasedCommandBuilder body_comp_builder;
    rb::ComponentBasedCommandBuilder comp_builder;
    bool use_body = false;

    if (!cmd.torso.position.empty()) {
      Eigen::VectorXd torso_q = Eigen::Map<const Eigen::VectorXd>(
          cmd.torso.position.data(), cmd.torso.position.size());
      rb::TorsoCommandBuilder torso_builder;
      torso_builder.SetCommand(
          rb::JointPositionCommandBuilder()
              .SetCommandHeader(
                  rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
              .SetPosition(torso_q)
              .SetMinimumTime(cmd.torso.minimum_time <= 0.0
                                  ? 0.1
                                  : cmd.torso.minimum_time));
      body_comp_builder.SetTorsoCommand(torso_builder);
      use_body = true;
    }

    if (!cmd.right_arm.position.empty()) {
      Eigen::VectorXd right_arm_q = Eigen::Map<const Eigen::VectorXd>(
          cmd.right_arm.position.data(), cmd.right_arm.position.size());
      rb::ArmCommandBuilder right_arm_builder;
      right_arm_builder.SetCommand(
          rb::JointPositionCommandBuilder()
              .SetCommandHeader(
                  rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
              .SetPosition(right_arm_q)
              .SetMinimumTime(cmd.right_arm.minimum_time <= 0.0
                                  ? 0.1
                                  : cmd.right_arm.minimum_time));
      body_comp_builder.SetRightArmCommand(right_arm_builder);
      use_body = true;
    }

    if (!cmd.left_arm.position.empty()) {
      Eigen::VectorXd left_arm_q = Eigen::Map<const Eigen::VectorXd>(
          cmd.left_arm.position.data(), cmd.left_arm.position.size());
      rb::ArmCommandBuilder left_arm_builder;
      left_arm_builder.SetCommand(
          rb::JointPositionCommandBuilder()
              .SetCommandHeader(
                  rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
              .SetPosition(left_arm_q)
              .SetMinimumTime(cmd.left_arm.minimum_time <= 0.0
                                  ? 0.1
                                  : cmd.left_arm.minimum_time));
      body_comp_builder.SetLeftArmCommand(left_arm_builder);
      use_body = true;
    }

    if (!cmd.head.position.empty()) {
      Eigen::VectorXd head_q = Eigen::Map<const Eigen::VectorXd>(
          cmd.head.position.data(), cmd.head.position.size());
      rb::HeadCommandBuilder head_builder;
      head_builder.SetCommand(
          rb::JointPositionCommandBuilder()
              .SetCommandHeader(
                  rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
              .SetPosition(head_q)
              .SetMinimumTime(
                  cmd.head.minimum_time <= 0.0 ? 0.1 : cmd.head.minimum_time));
      comp_builder.SetHeadCommand(head_builder);
    }

    if (use_body) {
      comp_builder.SetBodyCommand(
          rb::BodyCommandBuilder(std::move(body_comp_builder)));
    }

    rb::RobotCommandBuilder robot_cmd;
    robot_cmd.SetCommand(comp_builder);

    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (upper_body_stream_handler_) {
      upper_body_stream_handler_->SendCommand(robot_cmd);
      last_stream_command_time_ns_ = this->now().nanoseconds();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error in StreamJoint SendCommand: %s",
                 e.what());
    deactivate_all_streams();
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  result->success = true;
  try {
    goal_handle->succeed(result);
  } catch (...) {
  }
}

template <typename ModelType>
rclcpp_action::GoalResponse
RBY1_ROS2_DRIVER<ModelType>::handle_stream_cartesian_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const StreamCartesian::Goal> goal) {
  (void)uuid;
  (void)goal;
  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting StreamCartesian: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_stream_cartesian_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamCartesian>>
        goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_stream_cartesian_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamCartesian>>
        goal_handle) {
  std::thread{std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_stream_cartesian,
                        this, _1),
              goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_stream_cartesian(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamCartesian>>
        goal_handle) {
  has_printed_collision_log_ = false;

  auto result = std::make_shared<StreamCartesian::Result>();
  const auto goal = goal_handle->get_goal();
  const auto &cmd = goal->command;

  // 1. Check if stream is active
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (!stream_active_ || !upper_body_stream_handler_) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[STREAM REJECTED] stream_cartesian ignored: "
                  "Stream is not active.\033[0m");
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // 2. Check if trajectory is executing
  if (active_follow_joint_trajectory_goal_ &&
      active_follow_joint_trajectory_goal_->is_active()) {
    RCLCPP_WARN(this->get_logger(),
                "\033[1;33m[STREAM REJECTED] stream_cartesian ignored: "
                "Trajectory execution is active.\033[0m");
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // Determine commanded parts
  std::vector<size_t> commanded_parts;
  if (!cmd.torso.ref_link.empty() || !cmd.torso.target_link.empty())
    commanded_parts.push_back(PART_TORSO);
  if (!cmd.right_arm.ref_link.empty() || !cmd.right_arm.target_link.empty())
    commanded_parts.push_back(PART_RIGHT_ARM);
  if (!cmd.left_arm.ref_link.empty() || !cmd.left_arm.target_link.empty())
    commanded_parts.push_back(PART_LEFT_ARM);

  if (commanded_parts.empty()) {
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // 3. Check for non-stream control conflict
  for (auto part : commanded_parts) {
    if (is_controlling_[part].load(std::memory_order_acquire)) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[STREAM REJECTED] stream_cartesian ignored: Part "
                  "%zu is controlled by non-stream command.\033[0m",
                  part);
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // 4. Parse Cartesian commands and solve IK
  std::vector<typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget>
      link_targets;

  auto add_target_helper =
      [&](const rby1_msgs::msg::CartesianCommand &part_cmd) -> bool {
    if (part_cmd.ref_link.empty() && part_cmd.target_link.empty())
      return true;
    if (part_cmd.ref_link.empty() || part_cmd.target_link.empty())
      return false;

    int ref_idx = get_link_index(part_cmd.ref_link);
    int target_idx = get_link_index(part_cmd.target_link);
    if (ref_idx == -1 || target_idx == -1)
      return false;

    typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget target;
    target.ref_link_index = ref_idx;
    target.link_index = target_idx;

    Eigen::Quaterniond q_rot(
        part_cmd.transform.rotation.w, part_cmd.transform.rotation.x,
        part_cmd.transform.rotation.y, part_cmd.transform.rotation.z);
    Eigen::Vector3d t(part_cmd.transform.translation.x,
                      part_cmd.transform.translation.y,
                      part_cmd.transform.translation.z);

    rb::math::SE3::MatrixType T = rb::math::SE3::MatrixType::Identity();
    T.block(0, 0, 3, 3) = q_rot.toRotationMatrix();
    T.block(0, 3, 3, 1) = t;
    target.T = T;

    target.weight_position = 1000.0;
    target.weight_orientation = 100.0;

    link_targets.push_back(target);
    return true;
  };

  bool ok = true;
  ok &= add_target_helper(cmd.torso);
  ok &= add_target_helper(cmd.right_arm);
  ok &= add_target_helper(cmd.left_arm);

  if (!ok || link_targets.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "stream_cartesian: Invalid Cartesian targets");
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // Best-effort collision pre-check: in the regular Cartesian action the IK
  // is used only for collision prediction. However in stream_cartesian the
  // solved joint positions ARE the actual command, so we must abort if IK
  // fails.
  auto solved_q = solve_cartesian_ik(link_targets);
  if (!solved_q.has_value()) {
    RCLCPP_WARN(this->get_logger(), "stream_cartesian: IK did not converge — "
                                    "cannot compute joint targets.");
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // 5. Find max minimum_time
  double dt = 0.0;
  if (!cmd.torso.ref_link.empty())
    dt = std::max(dt, cmd.torso.minimum_time);
  if (!cmd.right_arm.ref_link.empty())
    dt = std::max(dt, cmd.right_arm.minimum_time);
  if (!cmd.left_arm.ref_link.empty())
    dt = std::max(dt, cmd.left_arm.minimum_time);

  if (dt <= 0.0)
    dt = 0.1;

  // 6. Calculate required acceleration and verify limits
  auto state = robot_->GetState();
  for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
    std::string joint_name = dyn_joint_names_[k];
    double target_pos = solved_q.value()[k];

    double current_pos = 0.0;
    bool found = false;
    for (size_t j = 0; j < info_.joint_infos.size(); ++j) {
      if (info_.joint_infos[j].name == joint_name) {
        current_pos = state.position[j];
        found = true;
        break;
      }
    }
    if (!found)
      continue;

    double req_accel = 2.0 * std::abs(target_pos - current_pos) / (dt * dt);
    double limit = qddot_upper_[k];

    if (req_accel > limit) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;31m[STREAM REJECTED] Cartesian target: Joint '%s' "
                  "required acceleration (%.4f rad/s^2) exceeds limit (%.4f "
                  "rad/s^2) with dt=%.4f s. Current: %.4f, Target: %.4f\033[0m",
                  joint_name.c_str(), req_accel, limit, dt, current_pos,
                  target_pos);
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // 7. Map solved joint values to target command arrays
  Eigen::VectorXd torso_q(info_.torso_joint_idx.size());
  Eigen::VectorXd right_q(info_.right_arm_joint_idx.size());
  Eigen::VectorXd left_q(info_.left_arm_joint_idx.size());

  for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
    std::string joint_name = dyn_joint_names_[k];
    double pos_val = solved_q.value()[k];

    for (size_t i = 0; i < info_.torso_joint_idx.size(); ++i) {
      if (info_.joint_infos[info_.torso_joint_idx[i]].name == joint_name) {
        torso_q[i] = pos_val;
        break;
      }
    }
    for (size_t i = 0; i < info_.right_arm_joint_idx.size(); ++i) {
      if (info_.joint_infos[info_.right_arm_joint_idx[i]].name == joint_name) {
        right_q[i] = pos_val;
        break;
      }
    }
    for (size_t i = 0; i < info_.left_arm_joint_idx.size(); ++i) {
      if (info_.joint_infos[info_.left_arm_joint_idx[i]].name == joint_name) {
        left_q[i] = pos_val;
        break;
      }
    }
  }

  rb::BodyComponentBasedCommandBuilder body_comp_builder;
  rb::ComponentBasedCommandBuilder comp_builder;
  bool use_body = false;

  if (!cmd.torso.ref_link.empty()) {
    rb::TorsoCommandBuilder torso_builder;
    torso_builder.SetCommand(
        rb::JointPositionCommandBuilder()
            .SetCommandHeader(
                rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
            .SetPosition(torso_q)
            .SetMinimumTime(
                cmd.torso.minimum_time <= 0.0 ? 0.1 : cmd.torso.minimum_time));
    body_comp_builder.SetTorsoCommand(torso_builder);
    use_body = true;
  }
  if (!cmd.right_arm.ref_link.empty()) {
    rb::ArmCommandBuilder right_builder;
    right_builder.SetCommand(
        rb::JointPositionCommandBuilder()
            .SetCommandHeader(
                rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
            .SetPosition(right_q)
            .SetMinimumTime(cmd.right_arm.minimum_time <= 0.0
                                ? 0.1
                                : cmd.right_arm.minimum_time));
    body_comp_builder.SetRightArmCommand(right_builder);
    use_body = true;
  }
  if (!cmd.left_arm.ref_link.empty()) {
    rb::ArmCommandBuilder left_builder;
    left_builder.SetCommand(
        rb::JointPositionCommandBuilder()
            .SetCommandHeader(
                rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
            .SetPosition(left_q)
            .SetMinimumTime(cmd.left_arm.minimum_time <= 0.0
                                ? 0.1
                                : cmd.left_arm.minimum_time));
    body_comp_builder.SetLeftArmCommand(left_builder);
    use_body = true;
  }

  if (use_body) {
    try {
      comp_builder.SetBodyCommand(
          rb::BodyCommandBuilder(std::move(body_comp_builder)));
      rb::RobotCommandBuilder robot_cmd;
      robot_cmd.SetCommand(comp_builder);

      std::lock_guard<std::mutex> stream_lock(stream_mutex_);
      if (upper_body_stream_handler_) {
        upper_body_stream_handler_->SendCommand(robot_cmd);
        last_stream_command_time_ns_ = this->now().nanoseconds();
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Error in StreamCartesian SendCommand: %s", e.what());
      deactivate_all_streams();
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  result->success = true;
  try {
    goal_handle->succeed(result);
  } catch (...) {
  }
}

template <typename ModelType>
bool RBY1_ROS2_DRIVER<ModelType>::any_part_controlling() const {
  for (size_t i = 0; i < PART_COUNT; ++i) {
    if (is_controlling_[i].load(std::memory_order_acquire)) {
      return true;
    }
  }
  return false;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::make_joint_pos_builder(
    rb::JointPositionCommandBuilder &builder, const Eigen::VectorXd &q,
    double minimum_time, double hold_time, double vel_limit, double acc_limit) {
  rb::CommandHeaderBuilder header;
  header.SetControlHoldTime(hold_time);

  builder.SetCommandHeader(header).SetPosition(q).SetMinimumTime(minimum_time);

  if (vel_limit > 0.0) {
    builder.SetVelocityLimit(Eigen::VectorXd::Constant(q.size(), vel_limit));
  }
  if (acc_limit > 0.0) {
    builder.SetAccelerationLimit(
        Eigen::VectorXd::Constant(q.size(), acc_limit));
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::make_joint_impedance_builder(
    rb::JointImpedanceControlCommandBuilder &builder, const Eigen::VectorXd &q,
    double minimum_time, double hold_time, const Eigen::VectorXd &stiffness,
    double damping_ratio, double torque_limit, double vel_limit,
    double acc_limit) {
  rb::CommandHeaderBuilder header;
  header.SetControlHoldTime(hold_time);

  builder.SetCommandHeader(header)
      .SetPosition(q)
      .SetMinimumTime(minimum_time)
      .SetStiffness(stiffness)
      .SetDampingRatio(damping_ratio)
      .SetTorqueLimit(Eigen::VectorXd::Constant(q.size(), torque_limit));

  if (vel_limit > 0.0) {
    builder.SetVelocityLimit(Eigen::VectorXd::Constant(q.size(), vel_limit));
  }
  if (acc_limit > 0.0) {
    builder.SetAccelerationLimit(
        Eigen::VectorXd::Constant(q.size(), acc_limit));
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::build_mobility_cmd(
    rb::MobilityCommandBuilder &builder, double vx, double vy, double wz,
    double minimum_time) {

  Eigen::Vector2d linear_vel(vx, vy);
  Eigen::Vector2d acceleration_limit(
      robot_parameter_.se2_linear_acceleration_limit,
      robot_parameter_.se2_linear_acceleration_limit);

  rb::SE2VelocityCommandBuilder se2_cmd;
  se2_cmd.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(1.0));
  se2_cmd.SetMinimumTime(minimum_time);
  se2_cmd.SetVelocity(linear_vel, wz);
  se2_cmd.SetAccelerationLimit(acceleration_limit,
                               robot_parameter_.se2_angular_acceleration_limit);

  builder.SetCommand(se2_cmd);
}

// Explicit template instantiations
template class RBY1_ROS2_DRIVER<rb::y1_model::A>;
template class RBY1_ROS2_DRIVER<rb::y1_model::M>;
} // namespace rby1_ros2
