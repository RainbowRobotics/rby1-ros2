#include "rby1_hardware/rby1_system_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <chrono>
#include <thread>

namespace rby1_hardware {

hardware_interface::CallbackReturn
RBY1SystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse parameters from URDF ros2_control configuration
  auto ip_it = info_.hardware_parameters.find("robot_ip");
  if (ip_it != info_.hardware_parameters.end()) {
    robot_ip_ = ip_it->second;
  } else {
    robot_ip_ = "127.0.0.1:50051";
  }

  auto model_it = info_.hardware_parameters.find("model");
  if (model_it != info_.hardware_parameters.end()) {
    model_type_ = model_it->second;
  } else {
    model_type_ = "a";
  }

  auto col_check_it = info_.hardware_parameters.find("collision_check_enable");
  if (col_check_it != info_.hardware_parameters.end()) {
    collision_check_enable_ =
        (col_check_it->second == "true" || col_check_it->second == "1");
  } else {
    collision_check_enable_ = false;
  }

  auto col_thresh_it = info_.hardware_parameters.find("collision_threshold");
  if (col_thresh_it != info_.hardware_parameters.end()) {
    try {
      collision_threshold_ = std::stod(col_thresh_it->second);
    } catch (...) {
      collision_threshold_ = 0.01;
    }
  } else {
    collision_threshold_ = 0.01;
  }

  std::string driver_ns = "rby1";
  auto driver_ns_it = info_.hardware_parameters.find("driver_namespace");
  if (driver_ns_it != info_.hardware_parameters.end()) {
    driver_ns = driver_ns_it->second;
  }

  if (driver_ns.empty() || driver_ns == "/") {
    driver_ns_prefix_ = "";
  } else {
    if (driver_ns.front() != '/') {
      driver_ns = "/" + driver_ns;
    }
    if (driver_ns.back() == '/') {
      driver_ns.pop_back();
    }
    driver_ns_prefix_ = driver_ns;
  }

  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
              "Initializing RBY1SystemHardware on %s (Model: %s, Collision "
              "Check: %s, Threshold: %.4f m, Namespace: %s)",
              robot_ip_.c_str(), model_type_.c_str(),
              collision_check_enable_ ? "ON" : "OFF", collision_threshold_,
              driver_ns_prefix_.c_str());

  // Resize internal buffers for joints
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  joint_name_to_sdk_index_.resize(info_.joints.size(), 0);

  // Check joint interface configuration
  for (const auto &joint : info_.joints) {
    // Check command interfaces
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                   "Joint '%s' has %zu command interfaces. Expected exactly 1 "
                   "(position).",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RBY1SystemHardware"),
          "Joint '%s' command interface is '%s'. Expected '%s' (position).",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check state interfaces
    if (joint.state_interfaces.size() < 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                   "Joint '%s' has %zu state interfaces. Expected at least 1 "
                   "(position).",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RBY1SystemHardware"),
          "Joint '%s' first state interface is '%s'. Expected '%s' (position).",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  node_ = std::make_shared<rclcpp::Node>("rby1_hardware_node");

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      driver_ns_prefix_ + "/joint_states", 10,
      std::bind(&RBY1SystemHardware::joint_state_callback, this,
                std::placeholders::_1));
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&RBY1SystemHardware::cmd_vel_callback, this,
                std::placeholders::_1));

  hardware_control_client_ =
      node_->create_client<rby1_msgs::srv::StateOnOff>(driver_ns_prefix_ + "/hardware_control");
  power_control_client_ =
      node_->create_client<rby1_msgs::srv::StateOnOff>(driver_ns_prefix_ + "/robot_power");
  servo_control_client_ =
      node_->create_client<rby1_msgs::srv::StateOnOff>(driver_ns_prefix_ + "/robot_servo");

  auto vel_limit_it = info_.hardware_parameters.find("velocity_limit");
  if (vel_limit_it != info_.hardware_parameters.end()) {
    try {
      velocity_limit_ = std::stod(vel_limit_it->second);
    } catch (...) {
      velocity_limit_ = 4.712388;
    }
  } else {
    velocity_limit_ = 4.712388;
  }

  auto acc_limit_it = info_.hardware_parameters.find("acceleration_limit");
  if (acc_limit_it != info_.hardware_parameters.end()) {
    try {
      acceleration_limit_ = std::stod(acc_limit_it->second);
    } catch (...) {
      acceleration_limit_ = 1.0;
    }
  } else {
    acceleration_limit_ = 1.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RBY1SystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RBY1SystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RBY1SystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
              "Activating RBY1 Hardware Interface...");

  // Instantiate SDK Wrapper based on model parameter
  if (model_type_ == "a" || model_type_ == "A") {
    robot_ = std::make_unique<RBY1RobotWrapperImpl<rb::y1_model::A>>();
  } else if (model_type_ == "m" || model_type_ == "M") {
    robot_ = std::make_unique<RBY1RobotWrapperImpl<rb::y1_model::M>>();
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Unsupported robot model: %s", model_type_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Connect to physical robot/simulator via SDK
  if (!robot_->connect(robot_ip_)) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Failed to connect to RBY1 SDK at %s", robot_ip_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
              "Connected to RBY1 SDK successfully.");

  // Claim control rights from driver
  if (!hardware_control_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Service /hardware_control not available.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }
  auto request = std::make_shared<rby1_msgs::srv::StateOnOff::Request>();
  request->state = true;
  auto future = hardware_control_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future,
                                         std::chrono::seconds(2)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
                  "Claimed hardware control from driver: %s",
                  response->message.c_str());
    } else {
      RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                   "Rejected claiming hardware control: %s",
                   response->message.c_str());
      robot_->disconnect();
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Timeout claiming hardware control.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto robot_info = robot_->get_robot_info();

  // Model verification
  std::string exp_model = model_type_; // "a" or "m"
  std::string conn_model = robot_info.robot_model_name;
  std::transform(exp_model.begin(), exp_model.end(), exp_model.begin(), ::tolower);
  std::transform(conn_model.begin(), conn_model.end(), conn_model.begin(), ::tolower);

  bool model_match = false;
  if (exp_model == "a" && (conn_model.find("a") != std::string::npos || conn_model.find("rby1a") != std::string::npos)) {
    model_match = true;
  } else if (exp_model == "m" && (conn_model.find("m") != std::string::npos || conn_model.find("rby1m") != std::string::npos)) {
    model_match = true;
  }

  // Version verification
  // Parse expected version from info_.name (e.g. "RBY1_A_v1_0_ros2_control")
  std::string name = info_.name;
  std::string exp_ver = "unknown";
  size_t pos = name.find("_v");
  if (pos != std::string::npos) {
    exp_ver = name.substr(pos + 2, 3); // "1_0", "1_1" etc.
    std::replace(exp_ver.begin(), exp_ver.end(), '_', '.');
  }

  std::string conn_ver = robot_info.robot_model_version;
  std::replace(conn_ver.begin(), conn_ver.end(), '_', '.');

  // Strip leading 'v' or 'V' if present
  if (!exp_ver.empty() && (exp_ver[0] == 'v' || exp_ver[0] == 'V')) {
    exp_ver = exp_ver.substr(1);
  }
  if (!conn_ver.empty() && (conn_ver[0] == 'v' || conn_ver[0] == 'V')) {
    conn_ver = conn_ver.substr(1);
  }

  if (!model_match || exp_ver != conn_ver) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "\033[1;31m====================================================================\033[0m");
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "\033[1;31m[MODEL/VERSION MISMATCH] MoveIt configuration does not match connected robot!\033[0m");
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "\033[1;31m  - MoveIt Expects: Model %s, Version %s (from node name: %s)\033[0m",
                 model_type_.c_str(), exp_ver.c_str(), name.c_str());
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "\033[1;31m  - Connected Robot: Model %s, Version %s\033[0m",
                 robot_info.robot_model_name.c_str(), robot_info.robot_model_version.c_str());
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "\033[1;31mPlease check that you launched the correct moveit demo.launch.py package.\033[0m");
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "\033[1;31m====================================================================\033[0m");
                 
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Map URDF joint names to SDK index positions
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto &joint_name = info_.joints[i].name;
    auto joint_it = std::find_if(robot_info.joint_infos.begin(),
                                 robot_info.joint_infos.end(),
                                 [&joint_name](const rb::JointInfo &info) {
                                   return info.name == joint_name;
                                 });

    if (joint_it == robot_info.joint_infos.end()) {
      if (joint_name.find("gripper_") != std::string::npos) {
        continue;
      }
      RCLCPP_FATAL(
          rclcpp::get_logger("RBY1SystemHardware"),
          "Joint '%s' declared in URDF was not found in robot SDK joint list.",
          joint_name.c_str());
      robot_->disconnect();
      return hardware_interface::CallbackReturn::ERROR;
    }

    unsigned int sdk_idx =
        std::distance(robot_info.joint_infos.begin(), joint_it);
    joint_name_to_sdk_index_[i] = sdk_idx;
  }

  // Setup joint prefix classification lists
  torso_joint_indices_.clear();
  right_arm_joint_indices_.clear();
  left_arm_joint_indices_.clear();
  head_joint_indices_.clear();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto &joint_name = info_.joints[i].name;
    if (joint_name.rfind("torso_", 0) == 0) {
      torso_joint_indices_.push_back(i);
    } else if (joint_name.rfind("right_arm_", 0) == 0) {
      right_arm_joint_indices_.push_back(i);
    } else if (joint_name.rfind("left_arm_", 0) == 0) {
      left_arm_joint_indices_.push_back(i);
    } else if (joint_name.rfind("head_", 0) == 0) {
      head_joint_indices_.push_back(i);
    }
  }

  // Power On and Servo On via Driver Services
  std::string power_dev = "all";
  std::string servo_dev = "all";
  auto power_dev_it = info_.hardware_parameters.find("power_on");
  if (power_dev_it != info_.hardware_parameters.end()) {
    power_dev = power_dev_it->second;
  }
  auto servo_dev_it = info_.hardware_parameters.find("servo_on");
  if (servo_dev_it != info_.hardware_parameters.end()) {
    servo_dev = servo_dev_it->second;
  }

  // 1. Wait for robot_power service and call it
  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
              "Requesting Power ON [%s] via driver...", power_dev.c_str());
  if (!power_control_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Service /robot_power not available.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }
  auto power_req = std::make_shared<rby1_msgs::srv::StateOnOff::Request>();
  power_req->state = true;
  power_req->parameters = power_dev;

  auto power_future = power_control_client_->async_send_request(power_req);
  if (rclcpp::spin_until_future_complete(node_, power_future,
                                         std::chrono::seconds(15)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Timeout waiting for /robot_power service.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!power_future.get()->success) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Failed to power ON via driver: %s",
                 power_future.get()->message.c_str());
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 2. Wait for robot_servo service and call it
  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
              "Requesting Servo ON [%s] via driver...", servo_dev.c_str());
  if (!servo_control_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Service /robot_servo not available.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }
  auto servo_req = std::make_shared<rby1_msgs::srv::StateOnOff::Request>();
  servo_req->state = true;
  servo_req->parameters = servo_dev;

  auto servo_future = servo_control_client_->async_send_request(servo_req);
  if (rclcpp::spin_until_future_complete(node_, servo_future,
                                         std::chrono::seconds(15)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Timeout waiting for /robot_servo service.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!servo_future.get()->success) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Failed to enable Servo ON via driver: %s",
                 servo_future.get()->message.c_str());
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Enable Control Manager
  if (!robot_->enable_control_manager()) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Failed to enable SDK Control Manager.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!robot_->wait_for_control_ready(2000)) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Control Manager failed to become ready in time.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize command stream
  if (!robot_->init_stream()) {
    RCLCPP_FATAL(rclcpp::get_logger("RBY1SystemHardware"),
                 "Failed to initialize gRPC Command Stream.");
    robot_->disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Start asynchronous IO worker thread
  worker_running_ = true;
  is_stream_alive_ = true;
  has_pending_command_ = false;
  io_worker_thread_ = std::thread(&RBY1SystemHardware::io_worker_loop, this);

  // Populate initial state
  std::vector<double> sdk_positions, sdk_velocities, sdk_torques;
  robot_->get_joint_states(sdk_positions, sdk_velocities, sdk_torques);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    unsigned int sdk_idx = joint_name_to_sdk_index_[i];
    hw_positions_[i] = sdk_positions[sdk_idx];
    hw_velocities_[i] = sdk_velocities[sdk_idx];
    hw_commands_[i] =
        hw_positions_[i]; // Commanded targets initialized to current positions
  }

  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
              "RBY1 Hardware Interface activated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBY1SystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
              "Deactivating RBY1 Hardware Interface...");

  // Stop IO worker thread
  worker_running_ = false;
  {
    std::lock_guard<std::mutex> lock(cmd_exchange_mutex_);
    pending_command_.cancel_requested = true;
    has_pending_command_ = true;
  }
  cmd_exchange_cv_.notify_all();

  if (io_worker_thread_.joinable()) {
    io_worker_thread_.join();
  }

  if (hardware_control_client_->service_is_ready()) {
    auto request = std::make_shared<rby1_msgs::srv::StateOnOff::Request>();
    request->state = false;
    auto future = hardware_control_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2));
  }

  if (robot_) {
    robot_->close_stream();
    robot_->disable_control_manager();
    robot_->cancel_control();

    robot_->disconnect();
    robot_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("RBY1SystemHardware"),
              "RBY1 Hardware Interface deactivated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
RBY1SystemHardware::read(const rclcpp::Time & /*time*/,
                         const rclcpp::Duration & /*period*/) {
  if (!robot_ || !robot_->is_connected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Spin node to process callbacks
  rclcpp::spin_some(node_);

  sensor_msgs::msg::JointState::SharedPtr joint_state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    joint_state = latest_joint_state_;
  }

  bool use_fallback = false;
  if (!joint_state) {
    use_fallback = true;
  } else {
    double age = (node_->now() - joint_state->header.stamp).seconds();
    if (age > 0.2) {
      use_fallback = true;
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RBY1SystemHardware"),
                           *node_->get_clock(), 2000,
                           "Joint state from driver is stale (age: %.3f s). Using SDK fallback.", age);
    }
  }

  if (use_fallback) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RBY1SystemHardware"),
                         *node_->get_clock(), 5000,
                         "No active joint state from driver. Querying SDK directly...");

    std::vector<double> sdk_positions, sdk_velocities, sdk_torques;
    robot_->get_joint_states(sdk_positions, sdk_velocities, sdk_torques);
    if (sdk_positions.size() >= (size_t)robot_->get_dof()) {
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        unsigned int sdk_idx = joint_name_to_sdk_index_[i];
        hw_positions_[i] = sdk_positions[sdk_idx];
        hw_velocities_[i] = sdk_velocities[sdk_idx];
      }
    }
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      hw_commands_[i] = hw_positions_[i];
    }
    return hardware_interface::return_type::OK;
  }

  // Map joint names to URDF indexes
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto &joint_name = info_.joints[i].name;
    auto it = std::find(joint_state->name.begin(), joint_state->name.end(),
                        joint_name);
    if (it != joint_state->name.end()) {
      size_t idx = std::distance(joint_state->name.begin(), it);
      if (idx < joint_state->position.size()) {
        hw_positions_[i] = joint_state->position[idx];
      }
      if (idx < joint_state->velocity.size()) {
        hw_velocities_[i] = joint_state->velocity[idx];
      }
    }
  }

  // Initialize command targets to read actual positions
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    hw_commands_[i] = hw_positions_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
RBY1SystemHardware::write(const rclcpp::Time & /*time*/,
                          const rclcpp::Duration & /*period*/) {
  if (!robot_ || !robot_->is_connected()) {
    return hardware_interface::return_type::ERROR;
  }

  int dof = robot_->get_dof();
  std::vector<double> sdk_target_positions(dof, 0.0);
  std::vector<double> sdk_positions(dof, 0.0);

  // Populate actual sdk_positions from hw_positions_
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    unsigned int sdk_idx = joint_name_to_sdk_index_[i];
    sdk_positions[sdk_idx] = hw_positions_[i];
  }
  sdk_target_positions = sdk_positions;

  // Fill in active commands from ros2_control
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    unsigned int sdk_idx = joint_name_to_sdk_index_[i];
    sdk_target_positions[sdk_idx] = hw_commands_[i];
  }

  // Predictive collision checking
  if (collision_check_enable_) {
    auto collision_reason = robot_->get_predicted_collision_reason(
        sdk_target_positions, collision_threshold_);
    if (collision_reason.has_value()) {
      auto &clock = *rclcpp::Clock::make_shared();
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("RBY1SystemHardware"), clock, 1000,
          "[PREDICTIVE COLLISION REJECTED] %s. Holding current position.",
          collision_reason->c_str());

      // Safety: overwrite commanded targets with current actual positions to
      // force a stop
      sdk_target_positions = sdk_positions;
    }
  }

  // Process mobility command if twist has been received and is fresh (< 0.5s)
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  bool has_twist = false;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (latest_twist_) {
      double elapsed = (node_->now() - cmd_vel_recv_time_).seconds();
      if (elapsed < 0.5) {
        vx = latest_twist_->linear.x;
        vy = latest_twist_->linear.y;
        wz = latest_twist_->angular.z;
        has_twist = true;
      } else {
        // Stop base on timeout
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
        has_twist = true;
      }
    }
  }

  // Non-blocking: update pending_command_ and notify worker thread
  {
    std::lock_guard<std::mutex> lock(cmd_exchange_mutex_);
    pending_command_.joint_positions = std::move(sdk_target_positions);
    pending_command_.vel_limit = velocity_limit_;
    pending_command_.acc_limit = acceleration_limit_;
    pending_command_.has_twist = has_twist;
    pending_command_.vx = vx;
    pending_command_.vy = vy;
    pending_command_.wz = wz;
    pending_command_.cancel_requested = false;
    has_pending_command_ = true;
  }
  cmd_exchange_cv_.notify_one();

  if (!is_stream_alive_.load()) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void RBY1SystemHardware::io_worker_loop() {
  while (worker_running_) {
    ControlStreamCommand cmd;
    {
      std::unique_lock<std::mutex> lock(cmd_exchange_mutex_);
      cmd_exchange_cv_.wait(lock, [this] {
        return has_pending_command_ || !worker_running_;
      });

      if (!worker_running_) {
        break;
      }

      cmd = pending_command_;
      has_pending_command_ = false;
    }

    if (cmd.cancel_requested) {
      if (robot_) {
        robot_->close_stream();
      }
      continue;
    }

    if (!robot_ || !robot_->is_connected()) {
      continue;
    }

    try {
      if (!cmd.joint_positions.empty()) {
        robot_->send_stream_command(cmd.joint_positions, cmd.vel_limit,
                                    cmd.acc_limit);
      }

      if (cmd.has_twist) {
        robot_->send_mobility_command(cmd.vx, cmd.vy, cmd.wz, 0.1);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Exception in RBY1SystemHardware IO worker thread: %s",
                   e.what());
      is_stream_alive_ = false;
    } catch (...) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Unknown exception in RBY1SystemHardware IO worker thread.");
      is_stream_alive_ = false;
    }
  }
}

void RBY1SystemHardware::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_joint_state_ = msg;
}

void RBY1SystemHardware::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_twist_ = msg;
  cmd_vel_recv_time_ = node_->now();
}

} // namespace rby1_hardware

PLUGINLIB_EXPORT_CLASS(rby1_hardware::RBY1SystemHardware,
                       hardware_interface::SystemInterface)
