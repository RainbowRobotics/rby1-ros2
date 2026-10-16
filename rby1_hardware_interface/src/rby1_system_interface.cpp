#include "rby1_hardware_interface/rby1_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

#include "rby1_hardware_interface/fake_robot_backend.hpp"

namespace rby1_hardware_interface
{
namespace
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

const std::set<std::string> kJointCommandInterfaces = {
  hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT,
  kStiffness, kDampingRatio, kTorqueLimit, kFeedbackGain};

}  // namespace

enum class Rby1SystemInterface::StatusField : uint8_t
{
  kControlManagerState, kControlState, kTimeScale, kEmoPressed, kStreamActive, kStreamRestarts,
  kClampCount, kStateAge, kOdometryX, kOdometryY, kOdometryYaw,
  kFtRight0, kFtRight1, kFtRight2, kFtRight3, kFtRight4, kFtRight5,
  kFtLeft0, kFtLeft1, kFtLeft2, kFtLeft3, kFtLeft4, kFtLeft5,
  kModeTorso, kModeRightArm, kModeLeftArm, kModeHead, kModeMobility, kModeBody,
};

namespace
{

using StatusField = Rby1SystemInterface::StatusField;

const std::map<std::string, StatusField> kStatusInterfaces = {
  {"control_manager_state", StatusField::kControlManagerState}, {"control_state", StatusField::kControlState},
  {"time_scale", StatusField::kTimeScale}, {"emo_pressed", StatusField::kEmoPressed},
  {"stream_active", StatusField::kStreamActive}, {"stream_restarts", StatusField::kStreamRestarts},
  {"clamp_count", StatusField::kClampCount}, {"state_age", StatusField::kStateAge},
  {"odometry_x", StatusField::kOdometryX}, {"odometry_y", StatusField::kOdometryY},
  {"odometry_yaw", StatusField::kOdometryYaw},
  {"ft_right_fx", StatusField::kFtRight0}, {"ft_right_fy", StatusField::kFtRight1},
  {"ft_right_fz", StatusField::kFtRight2}, {"ft_right_tx", StatusField::kFtRight3},
  {"ft_right_ty", StatusField::kFtRight4}, {"ft_right_tz", StatusField::kFtRight5},
  {"ft_left_fx", StatusField::kFtLeft0}, {"ft_left_fy", StatusField::kFtLeft1},
  {"ft_left_fz", StatusField::kFtLeft2}, {"ft_left_tx", StatusField::kFtLeft3},
  {"ft_left_ty", StatusField::kFtLeft4}, {"ft_left_tz", StatusField::kFtLeft5},
  {"mode_torso", StatusField::kModeTorso}, {"mode_right_arm", StatusField::kModeRightArm},
  {"mode_left_arm", StatusField::kModeLeftArm}, {"mode_head", StatusField::kModeHead},
  {"mode_mobility", StatusField::kModeMobility}, {"mode_body", StatusField::kModeBody}};

// Status value of a mode, for the mode_<component> status interfaces.
double mode_code(const CommandMode * mode)
{
  if (mode == nullptr) {
    return 0.0;
  }
  // Codes of the mode_<component> status interfaces (0: hold / none).
  static constexpr const char * kNames[] = {
    "joint_position", "joint_group_position", "joint_impedance", "gravity_compensation", "joint_velocity",
    "se2_velocity", "cartesian", "cartesian_impedance", "impedance", "optimal_control", "realtime"};
  for (size_t i = 0; i < sizeof(kNames) / sizeof(kNames[0]); ++i) {
    if (std::strcmp(kNames[i], mode->name()) == 0) {
      return static_cast<double>(i + 1);
    }
  }
  return -1.0;
}

std::optional<Component> component_from_joint_name(const std::string & name)
{
  auto starts = [&](const char * prefix) {return name.rfind(prefix, 0) == 0;};
  if (starts("torso_")) {return Component::kTorso;}
  if (starts("right_arm_")) {return Component::kRightArm;}
  if (starts("left_arm_")) {return Component::kLeftArm;}
  if (starts("head_")) {return Component::kHead;}
  if (name.find("wheel") != std::string::npos) {return Component::kMobility;}
  return std::nullopt;
}

std::string normalize_model(std::string model)
{
  std::transform(model.begin(), model.end(), model.begin(), [](unsigned char c) {return std::tolower(c);});
  if (model == "rby1a") {return "a";}
  if (model == "rby1m") {return "m";}
  if (model == "rby1ub") {return "ub";}
  return model;
}

}  // namespace

Rby1SystemInterface::Rby1SystemInterface()
: logger_(rclcpp::get_logger("rby1_system_interface"))
{
}

Rby1SystemInterface::~Rby1SystemInterface()
{
  stop_streaming();
  if (backend_) {
    backend_->stop_state_stream();
  }
}

void Rby1SystemInterface::set_backend_for_testing(std::shared_ptr<RobotBackend> backend)
{
  backend_ = std::move(backend);
  backend_injected_ = true;
}

std::string Rby1SystemInterface::active_mode(Component component) const
{
  std::lock_guard<std::mutex> lock(mode_mutex_);
  const auto * mode = active_[index_of(component)];
  return mode == nullptr ? "" : mode->name();
}

// ------------------------------------------------------------------------------------------
// Initialization

#ifdef RBY1_HARDWARE_PARAMS_INIT
CallbackReturn Rby1SystemInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return initialize(params.hardware_info);
}
#else
CallbackReturn Rby1SystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return initialize(info);
}
#endif

CallbackReturn Rby1SystemInterface::initialize(const hardware_interface::HardwareInfo & info)
{
  logger_ = rclcpp::get_logger("rby1_system_interface." + info.name);
  try {
    parameters_ = ParameterTable(std::map<std::string, std::string>(
        info.hardware_parameters.begin(), info.hardware_parameters.end()));
    model_ = normalize_model(parameters_.get_string("model", "a"));
    address_ = parameters_.get_string("robot_ip", "192.168.30.1:50051");
    backend_type_ = parameters_.get_string("backend", "sdk");
    state_rate_ = parameters_.get_double("state_rate", 200.0);
    command_rate_ = parameters_.get_double("command_rate", 100.0);
    state_timeout_ = parameters_.get_double("state_timeout", 0.1);
    stream_priority_ = static_cast<int>(parameters_.get_double("stream_priority", 1));
    send_timeout_ms_ = static_cast<int>(parameters_.get_double("send_timeout", 0.2) * 1000.0);
    max_consecutive_failures_ = static_cast<int>(parameters_.get_double("max_consecutive_failures", 10));
    {
      const auto policy = parameters_.get_string("on_stream_preempted", "stop");
      if (policy != "stop" && policy != "resume") {
        throw std::invalid_argument("on_stream_preempted must be 'stop' or 'resume'");
      }
      stop_on_preemption_ = policy == "stop";
    }
    rt_max_step_ = parameters_.get_double("realtime.max_target_step", 0.004);
    rt_max_tracking_error_ = parameters_.get_double("realtime.max_tracking_error", 0.15);
    rt_stale_ticks_ = static_cast<int>(parameters_.get_double("realtime.stale_ticks", 25));
    rt_priority_ = static_cast<int>(parameters_.get_double("realtime.priority", stream_priority_));
    ready_options_.auto_enable = parameters_.get_bool("auto_enable", false);
    ready_options_.power_selector = parameters_.get_string("power_selector", ".*");
    ready_options_.servo_selector = parameters_.get_string("servo_selector", ".*");
    ready_options_.reset_minor_fault = parameters_.get_bool("reset_minor_fault", true);
    if (backend_type_ != "sdk" && backend_type_ != "fake") {
      throw std::invalid_argument("parameter 'backend' must be 'sdk' or 'fake'");
    }
    if (model_ != "a" && model_ != "m" && model_ != "ub") {
      throw std::invalid_argument("parameter 'model' must be a, m or ub");
    }
    if (command_rate_ <= 0.0 || state_rate_ <= 0.0) {
      throw std::invalid_argument("state_rate and command_rate must be positive");
    }

    if (!parse_joints(info) || !parse_gpios(info)) {
      return CallbackReturn::ERROR;
    }
    load_urdf_limits(info.original_xml);
    modes_ = make_modes(joints_, gpios_, parameters_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Invalid RB-Y1 hardware configuration: %s", e.what());
    return CallbackReturn::ERROR;
  }

  commands_.assign(command_names_.size(), kNaN);
  conditioned_.assign(command_names_.size(), kNaN);
  slot_.values.assign(command_names_.size(), kNaN);
  position_.assign(joints_.size(), kNaN);
  velocity_.assign(joints_.size(), 0.0);
  effort_.assign(joints_.size(), 0.0);
  hold_position_.assign(joints_.size(), kNaN);
  slot_.measured.assign(joints_.size(), kNaN);

  std::string mode_list;
  for (const auto & mode : modes_) {
    mode_list += std::string(" ") + to_string(mode->component()) + ":" + mode->name();
  }
  RCLCPP_INFO(
    logger_, "RB-Y1 model '%s' at %s (%s backend): %zu joints, %zu command interfaces, modes:%s",
    model_.c_str(), address_.c_str(), backend_type_.c_str(), joints_.size(), command_names_.size(),
    mode_list.c_str());
  return CallbackReturn::SUCCESS;
}

bool Rby1SystemInterface::parse_joints(const hardware_interface::HardwareInfo & info)
{
  joints_.clear();
  for (const auto & joint : info.joints) {
    JointEntry entry;
    entry.name = joint.name;
    const auto component_param = joint.parameters.find("component");
    const auto component = component_param != joint.parameters.end() ?
      component_from_string(component_param->second) : component_from_joint_name(joint.name);
    if (!component || *component == Component::kBody) {
      RCLCPP_ERROR(logger_, "Cannot tell the component of joint '%s'; set its 'component' parameter", joint.name.c_str());
      return false;
    }
    entry.component = *component;

    std::set<std::string> states;
    for (const auto & s : joint.state_interfaces) {
      states.insert(s.name);
    }
    for (const auto * required : {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT})
    {
      if (states.count(required) == 0) {
        RCLCPP_ERROR(logger_, "Joint '%s' must declare state interface '%s'", joint.name.c_str(), required);
        return false;
      }
    }

    for (const auto & c : joint.command_interfaces) {
      if (kJointCommandInterfaces.count(c.name) == 0) {
        RCLCPP_ERROR(logger_, "Joint '%s': unsupported command interface '%s'", joint.name.c_str(), c.name.c_str());
        return false;
      }
      const size_t slot = command_names_.size();
      const auto full = joint.name + "/" + c.name;
      command_names_.push_back(full);
      command_slot_[full] = slot;
      slot_component_.push_back(entry.component);
      if (c.name == hardware_interface::HW_IF_POSITION) {entry.cmd_position = slot;}
      if (c.name == hardware_interface::HW_IF_VELOCITY) {entry.cmd_velocity = slot;}
      if (c.name == hardware_interface::HW_IF_EFFORT) {entry.cmd_effort = slot;}
      if (c.name == kStiffness) {entry.cmd_stiffness = slot;}
      if (c.name == kDampingRatio) {entry.cmd_damping_ratio = slot;}
      if (c.name == kTorqueLimit) {entry.cmd_torque_limit = slot;}
      if (c.name == kFeedbackGain) {entry.cmd_feedback_gain = slot;}
    }
    joints_.push_back(entry);
  }
  if (joints_.empty()) {
    RCLCPP_ERROR(logger_, "The RB-Y1 ros2_control block declares no joints");
    return false;
  }
  return true;
}

bool Rby1SystemInterface::parse_gpios(const hardware_interface::HardwareInfo & info)
{
  gpios_.clear();
  gpios_.reserve(info.gpios.size());
  for (const auto & gpio : info.gpios) {
    GpioEntry entry;
    entry.name = gpio.name;
    entry.parameters = ParameterTable(std::map<std::string, std::string>(gpio.parameters.begin(), gpio.parameters.end()));
    entry.kind = entry.parameters.get_string("kind", "");
    if (entry.kind.empty()) {
      RCLCPP_ERROR(logger_, "gpio '%s' needs a 'kind' parameter", gpio.name.c_str());
      return false;
    }
    if (entry.kind == "status") {
      for (const auto & s : gpio.state_interfaces) {
        if (kStatusInterfaces.count(s.name) == 0) {
          RCLCPP_ERROR(logger_, "gpio '%s': unknown status interface '%s'", gpio.name.c_str(), s.name.c_str());
          return false;
        }
        status_values_[s.name] = 0.0;
        status_interfaces_.emplace_back(gpio.name, s.name);
        status_slots_.emplace_back(kStatusInterfaces.at(s.name), &status_values_[s.name]);
      }
      entry.component = Component::kBody;
    } else {
      const auto component = component_from_string(entry.parameters.get_string("component", ""));
      if (!component) {
        RCLCPP_ERROR(logger_, "gpio '%s' needs a 'component' parameter (torso, right_arm, left_arm, head, mobility, body)",
          gpio.name.c_str());
        return false;
      }
      entry.component = *component;
      for (const auto & c : gpio.command_interfaces) {
        const size_t slot = command_names_.size();
        const auto full = gpio.name + "/" + c.name;
        command_names_.push_back(full);
        command_slot_[full] = slot;
        slot_component_.push_back(entry.component);
        entry.command_slots[c.name] = slot;
      }
      if (!gpio.state_interfaces.empty()) {
        RCLCPP_WARN(logger_, "gpio '%s': state interfaces are only supported on kind 'status'; ignored", gpio.name.c_str());
      }
    }
    gpios_.push_back(entry);
  }
  return true;
}

void Rby1SystemInterface::load_urdf_limits(const std::string & urdf_xml)
{
  if (urdf_xml.empty()) {
    RCLCPP_WARN(logger_, "No URDF in hardware info; joint position limits are not enforced by the driver");
    return;
  }
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    RCLCPP_WARN(logger_, "Could not parse the URDF; joint position limits are not enforced by the driver");
    return;
  }
  for (auto & entry : joints_) {
    const auto joint = model.getJoint(entry.name);
    if (!joint || !joint->limits) {
      continue;
    }
    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) {
      entry.has_position_limits = joint->limits->lower < joint->limits->upper;
      entry.lower = joint->limits->lower;
      entry.upper = joint->limits->upper;
    }
    entry.velocity_limit = joint->limits->velocity;
    entry.effort_limit = joint->limits->effort;
  }
}

// ------------------------------------------------------------------------------------------
// Interfaces

std::vector<hardware_interface::StateInterface> Rby1SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < joints_.size(); ++i) {
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_EFFORT, &effort_[i]);
  }
  for (const auto & [gpio, name] : status_interfaces_) {
    out.emplace_back(gpio, name, &status_values_[name]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> Rby1SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t slot = 0; slot < command_names_.size(); ++slot) {
    const auto & full = command_names_[slot];
    const auto separator = full.find('/');
    out.emplace_back(full.substr(0, separator), full.substr(separator + 1), &commands_[slot]);
  }
  return out;
}

// ------------------------------------------------------------------------------------------
// Lifecycle

CallbackReturn Rby1SystemInterface::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    if (!backend_injected_) {
      if (backend_type_ == "fake") {
        backend_ = std::make_shared<FakeRobotBackend>(model_);
      } else {
        backend_ = make_sdk_backend(model_, address_);
      }
    }
    description_ = backend_->connect(parameters_.get_double("connect_timeout", 3.0));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to connect to the RB-Y1: %s", e.what());
    return CallbackReturn::ERROR;
  }

  for (auto & entry : joints_) {
    const auto it = std::find(description_.joint_names.begin(), description_.joint_names.end(), entry.name);
    if (it == description_.joint_names.end()) {
      RCLCPP_ERROR(
        logger_, "Joint '%s' does not exist on the robot (model '%s' %s); check model and URDF",
        entry.name.c_str(), description_.model_name.c_str(), description_.model_version.c_str());
      return CallbackReturn::ERROR;
    }
    entry.robot_index = static_cast<size_t>(it - description_.joint_names.begin());
    const auto & ids = description_.component_joints[index_of(entry.component)];
    if (std::find(ids.begin(), ids.end(), entry.robot_index) == ids.end()) {
      RCLCPP_WARN(logger_, "Joint '%s' is not in the robot's %s group", entry.name.c_str(), to_string(entry.component));
    }
  }

  state_buffer_.reset(StateSample{});
  backend_->start_state_stream(state_rate_, [this](const StateSample & sample) {on_state(sample);});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    if (state_buffer_.has_data()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!copy_state()) {
    RCLCPP_ERROR(logger_, "No state received from the RB-Y1 within 3 s");
    backend_->stop_state_stream();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger_, "Connected to RB-Y1 %s %s (robot %s, SDK %s); state stream at %.0f Hz",
    description_.model_name.c_str(), description_.model_version.c_str(), description_.robot_version.c_str(),
    description_.sdk_version.c_str(), state_rate_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rby1SystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!backend_ || !backend_->is_connected()) {
    RCLCPP_ERROR(logger_, "Cannot activate: not connected");
    return CallbackReturn::ERROR;
  }
  std::string message;
  try {
    if (!backend_->ensure_ready(ready_options_, message)) {
      RCLCPP_ERROR(
        logger_, "RB-Y1 is not ready for control: %s. Enable it with the robot manager or set auto_enable:=true.",
        message.c_str());
      return CallbackReturn::ERROR;
    }
    // Robot parameters: hardware parameter "robot_parameter.<name>" -> SetParameter(<name>), not persisted.
    for (const auto & [key, value] : info_.hardware_parameters) {
      const std::string prefix = "robot_parameter.";
      if (key.rfind(prefix, 0) == 0 && !backend_->set_parameter(key.substr(prefix.size()), value)) {
        RCLCPP_ERROR(logger_, "Failed to set robot parameter '%s'", key.substr(prefix.size()).c_str());
        return CallbackReturn::ERROR;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Activation failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (!copy_state()) {
    RCLCPP_ERROR(logger_, "No fresh state at activation");
    return CallbackReturn::ERROR;
  }

  {
    std::lock_guard<std::mutex> mode_lock(mode_mutex_);
    claimed_.clear();
    active_.fill(nullptr);
    pending_.fill(nullptr);
    ready_.fill(false);
  }
  std::fill(commands_.begin(), commands_.end(), kNaN);
  std::fill(conditioned_.begin(), conditioned_.end(), kNaN);
  {
    std::lock_guard<std::mutex> lock(slot_mutex_);
    hold_position_ = position_;
    slot_.sequence = 0;
  }
  {
    std::lock_guard<std::mutex> lock(fatal_mutex_);
    fatal_message_.clear();
  }
  fatal_ = false;
  streaming_ = false;
  stream_restarts_ = 0;
  clamp_count_ = 0;
  stream_thread_run_ = true;
  stream_thread_ = std::thread([this]() {stream_loop();});
  active_hw_ = true;
  RCLCPP_INFO(logger_, "RB-Y1 hardware active (%s); streaming starts with the first claiming controller", message.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rby1SystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_hw_ = false;
  stop_streaming();
  RCLCPP_INFO(logger_, "RB-Y1 hardware deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rby1SystemInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_hw_ = false;
  stop_streaming();
  if (backend_) {
    backend_->stop_state_stream();
    backend_->disconnect();
    if (!backend_injected_) {
      backend_.reset();
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rby1SystemInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

CallbackReturn Rby1SystemInterface::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(logger_, "RB-Y1 hardware error: %s", fatal_message_.c_str());
  active_hw_ = false;
  stop_streaming();
  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
// Mode switching

std::optional<Component> Rby1SystemInterface::component_of_interface(const std::string & full_name) const
{
  const auto it = command_slot_.find(full_name);
  if (it == command_slot_.end()) {
    return std::nullopt;
  }
  return slot_component_[it->second];
}

bool Rby1SystemInterface::resolve_modes(
  const std::set<std::string> & claimed, ModeArray & modes, std::string & error) const
{
  std::array<std::set<std::string>, kComponentCount> by_component;
  // Real-time control claims effort and feedback_gain: its torso, arm and head claims are
  // matched together against the body-level real-time mode.
  const bool realtime = std::any_of(claimed.begin(), claimed.end(), [](const std::string & n) {
        return n.size() > 7 && (n.compare(n.size() - 7, 7, "/effort") == 0 ||
        (n.size() > 14 && n.compare(n.size() - 14, 14, "/feedback_gain") == 0));
      });
  for (const auto & name : claimed) {
    auto component = component_of_interface(name);
    if (!component) {
      continue;
    }
    if (realtime && (*component == Component::kTorso || *component == Component::kRightArm ||
      *component == Component::kLeftArm || *component == Component::kHead))
    {
      component = Component::kBody;
    }
    by_component[index_of(*component)].insert(name);
  }
  modes.fill(nullptr);
  for (const auto component : kAllComponents) {
    const auto & claims = by_component[index_of(component)];
    if (claims.empty()) {
      continue;
    }
    std::vector<const CommandMode *> candidates;
    for (const auto & mode : modes_) {
      if (mode->component() == component && mode->matches(claims)) {
        candidates.push_back(mode.get());
      }
    }
    if (candidates.size() != 1) {
      std::string list;
      for (const auto & c : claims) {
        list += " " + c;
      }
      error = std::string(candidates.empty() ? "no RB-Y1 command mode accepts" : "ambiguous command mode for") +
        " the " + to_string(component) + " interfaces:" + list;
      return false;
    }
    modes[index_of(component)] = candidates.front();
  }
  if (modes[index_of(Component::kBody)] != nullptr &&
    (modes[index_of(Component::kTorso)] || modes[index_of(Component::kRightArm)] ||
    modes[index_of(Component::kLeftArm)]))
  {
    error = "a body-level mode cannot run together with torso or arm modes";
    return false;
  }
  const auto * body = modes[index_of(Component::kBody)];
  if (body != nullptr && body->is_realtime() &&
    (modes[index_of(Component::kHead)] != nullptr || modes[index_of(Component::kMobility)] != nullptr))
  {
    error = "real-time control is exclusive: deactivate the head and base controllers";
    return false;
  }
  return true;
}

return_type Rby1SystemInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
  std::lock_guard<std::mutex> lock(mode_mutex_);
  auto claimed = claimed_;
  for (const auto & name : stop_interfaces) {
    claimed.erase(name);
  }
  for (const auto & name : start_interfaces) {
    if (command_slot_.count(name) != 0) {
      claimed.insert(name);
    }
  }
  std::string error;
  if (!resolve_modes(claimed, pending_, error)) {
    RCLCPP_ERROR(logger_, "Rejected controller switch: %s", error.c_str());
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type Rby1SystemInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
  std::lock_guard<std::mutex> lock(mode_mutex_);
  auto claimed = claimed_;
  for (const auto & name : stop_interfaces) {
    claimed.erase(name);
  }
  for (const auto & name : start_interfaces) {
    if (command_slot_.count(name) != 0) {
      claimed.insert(name);
    }
  }
  ModeArray modes{};
  std::string error;
  if (!resolve_modes(claimed, modes, error)) {
    RCLCPP_ERROR(logger_, "Controller switch failed: %s", error.c_str());
    return return_type::ERROR;
  }

  const StateView state{&position_, &velocity_, &effort_};
  for (const auto component : kAllComponents) {
    const auto i = index_of(component);
    if (modes[i] == active_[i]) {
      continue;
    }
    if (active_[i] != nullptr) {
      // The component (or the components covered by body) is held where it is now.
      std::lock_guard<std::mutex> slot_lock(slot_mutex_);
      for (size_t j = 0; j < joints_.size(); ++j) {
        const bool covered = joints_[j].component == component ||
          (component == Component::kBody && joints_[j].component != Component::kHead &&
          joints_[j].component != Component::kMobility);
        if (covered) {
          hold_position_[j] = position_[j];
        }
      }
    }
    if (modes[i] != nullptr) {
      modes[i]->on_activate(commands_, state);
      for (size_t slot = 0; slot < commands_.size(); ++slot) {
        if (slot_component_[slot] == component) {
          conditioned_[slot] = commands_[slot];
        }
      }
      ready_[i] = false;
    }
    RCLCPP_INFO(
      logger_, "%s: %s -> %s", to_string(component), active_[i] ? active_[i]->name() : "hold",
      modes[i] ? modes[i]->name() : "hold");
  }
  active_ = modes;
  claimed_ = claimed;
  if (std::any_of(active_.begin(), active_.end(), [](const CommandMode * m) {return m != nullptr;})) {
    streaming_ = true;
  }
  return return_type::OK;
}

// ------------------------------------------------------------------------------------------
// State

void Rby1SystemInterface::on_state(const StateSample & sample)
{
  // SDK state thread: the back buffer has the same vector sizes, so this copy does not allocate.
  state_buffer_.back() = sample;
  state_buffer_.publish();
}

bool Rby1SystemInterface::copy_state()
{
  if (!state_buffer_.has_data()) {
    return false;
  }
  current_ = state_buffer_.read();  // lock-free; same sizes, no allocation
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto r = joints_[i].robot_index;
    if (r >= current_.position.size()) {
      return false;
    }
    position_[i] = current_.position[r];
    velocity_[i] = current_.velocity[r];
    effort_[i] = current_.torque[r];
  }
  return true;
}

void Rby1SystemInterface::update_status_values()
{
  const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - current_.received).count();
  for (const auto & [field, value] : status_slots_) {
    switch (field) {
      case StatusField::kControlManagerState: *value = static_cast<double>(current_.control_manager); break;
      case StatusField::kControlState: *value = static_cast<double>(current_.control); break;
      case StatusField::kTimeScale: *value = current_.time_scale; break;
      case StatusField::kEmoPressed: *value = current_.emo_pressed ? 1.0 : 0.0; break;
      case StatusField::kStreamActive: *value = stream_ok_ ? 1.0 : 0.0; break;
      case StatusField::kStreamRestarts: *value = static_cast<double>(stream_restarts_.load()); break;
      case StatusField::kClampCount: *value = static_cast<double>(clamp_count_.load()); break;
      case StatusField::kStateAge: *value = age; break;
      case StatusField::kOdometryX: *value = current_.odometry[0]; break;
      case StatusField::kOdometryY: *value = current_.odometry[1]; break;
      case StatusField::kOdometryYaw: *value = current_.odometry[2]; break;
      case StatusField::kFtRight0: case StatusField::kFtRight1: case StatusField::kFtRight2:
      case StatusField::kFtRight3: case StatusField::kFtRight4: case StatusField::kFtRight5:
        *value = current_.ft_right[static_cast<size_t>(field) - static_cast<size_t>(StatusField::kFtRight0)];
        break;
      case StatusField::kFtLeft0: case StatusField::kFtLeft1: case StatusField::kFtLeft2:
      case StatusField::kFtLeft3: case StatusField::kFtLeft4: case StatusField::kFtLeft5:
        *value = current_.ft_left[static_cast<size_t>(field) - static_cast<size_t>(StatusField::kFtLeft0)];
        break;
      case StatusField::kModeTorso: *value = mode_code(active_[index_of(Component::kTorso)]); break;
      case StatusField::kModeRightArm: *value = mode_code(active_[index_of(Component::kRightArm)]); break;
      case StatusField::kModeLeftArm: *value = mode_code(active_[index_of(Component::kLeftArm)]); break;
      case StatusField::kModeHead: *value = mode_code(active_[index_of(Component::kHead)]); break;
      case StatusField::kModeMobility: *value = mode_code(active_[index_of(Component::kMobility)]); break;
      case StatusField::kModeBody: *value = mode_code(active_[index_of(Component::kBody)]); break;
    }
  }
}

return_type Rby1SystemInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!copy_state()) {
    return active_hw_ ? return_type::ERROR : return_type::OK;
  }
  const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - current_.received).count();
  if (active_hw_ && streaming_) {
    if (age > state_timeout_) {
      set_fatal("robot state is " + std::to_string(age) + " s old");
    } else if (current_.control_manager == ControlManagerStatus::kMajorFault ||
      current_.control_manager == ControlManagerStatus::kMinorFault)
    {
      set_fatal("control manager reports a fault");
    } else if (current_.emo_pressed) {
      set_fatal("emergency stop pressed");
    }
  }
  {
    std::unique_lock<std::mutex> lock(mode_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      update_status_values();
    }
  }
  return fatal_ ? return_type::ERROR : return_type::OK;
}

// ------------------------------------------------------------------------------------------
// Commands

return_type Rby1SystemInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_hw_) {
    return return_type::OK;
  }
  if (fatal_) {
    return return_type::ERROR;
  }
  std::unique_lock<std::mutex> mode_lock(mode_mutex_, std::try_to_lock);
  if (!mode_lock.owns_lock()) {
    return return_type::OK;  // a switch is in progress; keep the previous slot
  }
  const StateView state{&position_, &velocity_, &effort_};
  size_t clamps = 0;
  for (const auto component : kAllComponents) {
    const auto i = index_of(component);
    if (active_[i] != nullptr) {
      ready_[i] = active_[i]->condition(commands_, conditioned_, state, clamps);
    }
  }
  if (clamps != 0) {
    clamp_count_ += clamps;
  }
  std::unique_lock<std::mutex> slot_lock(slot_mutex_, std::try_to_lock);
  if (slot_lock.owns_lock()) {
    std::copy(conditioned_.begin(), conditioned_.end(), slot_.values.begin());
    std::copy(position_.begin(), position_.end(), slot_.measured.begin());
    slot_.modes = active_;
    slot_.ready = ready_;
    ++slot_.sequence;
  }
  return return_type::OK;
}

void Rby1SystemInterface::set_fatal(const std::string & message)
{
  if (fatal_.exchange(true)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(fatal_mutex_);
    fatal_message_ = message;
  }
  RCLCPP_ERROR(logger_, "Stopping RB-Y1 command stream: %s", message.c_str());
}

void Rby1SystemInterface::stream_loop()
{
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / command_rate_));
  auto next = std::chrono::steady_clock::now();
  Slot local;
  local.values.assign(slot_.values.size(), kNaN);
  std::vector<double> hold;
  int failures = 0;
  std::array<int, kComponentCount> stream_structure{};
  size_t sends_on_stream = 0;
  uint64_t logged_clamps = 0;
  auto last_clamp_log = std::chrono::steady_clock::now();

  std::array<std::vector<size_t>, kComponentCount> component_joint_ids;
  for (size_t j = 0; j < joints_.size(); ++j) {
    component_joint_ids[index_of(joints_[j].component)].push_back(j);
  }

  while (stream_thread_run_) {
    next += period;
    std::this_thread::sleep_until(next);
    if (std::chrono::steady_clock::now() - next > 5 * period) {
      next = std::chrono::steady_clock::now();  // do not burst after a stall
    }
    if (!streaming_ || fatal_) {
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(slot_mutex_);
      if (slot_.sequence == 0) {
        continue;
      }
      local.values = slot_.values;
      local.modes = slot_.modes;
      local.ready = slot_.ready;
      local.sequence = slot_.sequence;
      hold = hold_position_;
    }

    const auto * body_mode = local.modes[index_of(Component::kBody)];
    if (body_mode != nullptr && body_mode->is_realtime()) {
      if (!backend_->realtime_running() && !fatal_) {
        if (!start_realtime()) {
          set_fatal("cannot start real-time control");
        }
      }
      continue;
    }
    if (backend_->realtime_running()) {
      backend_->stop_realtime();
      std::lock_guard<std::mutex> lock(slot_mutex_);
      hold = slot_.measured;  // continue with a hold where the real-time loop left the robot
      hold_position_ = hold;
      RCLCPP_INFO(logger_, "Real-time control ended; back to the command stream");
    }

    RobotCommandRequest request;
    try {
      const bool body_mode = local.modes[index_of(Component::kBody)] != nullptr;
      for (const auto component : kAllComponents) {
        const auto i = index_of(component);
        const auto * mode = local.modes[i];
        if (mode != nullptr && local.ready[i]) {
          request.components[i] = mode->build(local.values);
          continue;
        }
        const bool holdable = component == Component::kTorso || component == Component::kRightArm ||
          component == Component::kLeftArm || component == Component::kHead;
        const bool covered_by_body = body_mode && component != Component::kHead;
        if (holdable && !covered_by_body && !component_joint_ids[i].empty()) {
          request.components[i] = make_hold_command(joints_, component_joint_ids[i], hold, parameters_);
        }
      }
    } catch (const std::exception & e) {
      set_fatal(std::string("could not build command: ") + e.what());
      continue;
    }
    if (request.empty()) {
      continue;
    }

    // The robot fixes each component's controller type when a stream starts: a change of
    // structure (e.g. joint position -> joint impedance, or the base joining) needs a new stream.
    std::array<int, kComponentCount> structure{};
    for (size_t i = 0; i < kComponentCount; ++i) {
      structure[i] = static_cast<int>(request.components[i].index());
    }
    if (backend_->stream_open() && structure != stream_structure) {
      RCLCPP_INFO(logger_, "Command structure changed; restarting the command stream");
      backend_->close_stream();
    }

    if (!backend_->stream_open()) {
      std::string message;
      if (!backend_->open_stream(stream_priority_, message)) {
        stream_ok_ = false;
        if (++failures >= max_consecutive_failures_) {
          set_fatal("cannot open command stream: " + message);
        }
        continue;
      }
      ++stream_restarts_;
      stream_structure = structure;
      sends_on_stream = 0;
    }
    const auto feedback = backend_->send(request, send_timeout_ms_);
    switch (feedback.status) {
      case StreamStatus::kOk:
        failures = 0;
        ++sends_on_stream;
        stream_ok_ = true;
        break;
      case StreamStatus::kFault:
        stream_ok_ = false;
        set_fatal("robot reported a control fault: " + feedback.message);
        break;
      case StreamStatus::kFinished:
        if (sends_on_stream > 0 && stop_on_preemption_) {
          // The driver renews the hold time every cycle, so a stream that was running only ends
          // when another command takes over (robot manager stop or cancel, a higher-priority
          // client). Do not take the robot back.
          stream_ok_ = false;
          set_fatal("the command stream was ended by another command (stop, cancel or a higher-priority "
            "client); activate the hardware again to resume");
          break;
        }
        [[fallthrough]];
      case StreamStatus::kTimeout:
      case StreamStatus::kError:
        stream_ok_ = false;
        RCLCPP_WARN(logger_, "Command stream: %s", feedback.message.c_str());
        if (++failures >= max_consecutive_failures_) {
          set_fatal("command stream failed " + std::to_string(failures) + " times: " + feedback.message);
        }
        break;
    }

    const auto clamps = clamp_count_.load();
    if (clamps != logged_clamps && std::chrono::steady_clock::now() - last_clamp_log > std::chrono::seconds(2)) {
      RCLCPP_WARN(
        logger_, "Commands were limited by the driver guards %lu times (limits, tracking error, step)",
        static_cast<unsigned long>(clamps - logged_clamps));
      logged_clamps = clamps;
      last_clamp_log = std::chrono::steady_clock::now();
    }
  }
}

bool Rby1SystemInterface::start_realtime()
{
  backend_->close_stream();
  stream_ok_ = false;
  {
    std::lock_guard<std::mutex> lock(slot_mutex_);
    rt_mode_ = slot_.modes[index_of(Component::kBody)];
    rt_values_ = slot_.values;
  }
  rt_entry_of_robot_.assign(description_.joint_names.size(), -1);
  for (size_t j = 0; j < joints_.size(); ++j) {
    rt_entry_of_robot_[joints_[j].robot_index] = static_cast<int>(j);
  }
  rt_claimed_.assign(joints_.size(), false);
  if (rt_mode_ != nullptr) {
    for (const auto id : rt_mode_->joint_ids()) {
      rt_claimed_[id] = true;
    }
  }
  rt_initialized_ = false;
  rt_stale_ = 0;
  rt_last_t_ = -1.0;
  std::string message;
  const bool ok = backend_->start_realtime(
    [this](const RealtimeState & state, RealtimeInput & input) {realtime_tick(state, input);}, rt_priority_, message);
  if (ok) {
    RCLCPP_INFO(logger_, "Real-time control started (%.3f rad/tick, %.2f rad tracking limit)", rt_max_step_,
      rt_max_tracking_error_);
  } else {
    RCLCPP_ERROR(logger_, "Real-time control failed to start: %s", message.c_str());
  }
  return ok;
}

void Rby1SystemInterface::realtime_tick(const RealtimeState & state, RealtimeInput & input)
{
  const size_t n = state.position.size();
  if (!rt_initialized_) {
    rt_sent_ = state.position;
    rt_initialized_ = true;
  }
  if (state.t == rt_last_t_) {
    if (++rt_stale_ >= rt_stale_ticks_) {
      set_fatal("real-time state did not advance for " + std::to_string(rt_stale_) + " ticks");
    }
  } else {
    rt_stale_ = 0;
  }
  rt_last_t_ = state.t;
  {
    std::unique_lock<std::mutex> lock(slot_mutex_, std::try_to_lock);
    if (lock.owns_lock() && slot_.values.size() == rt_values_.size()) {
      std::copy(slot_.values.begin(), slot_.values.end(), rt_values_.begin());
    }
  }
  for (size_t r = 0; r < n; ++r) {
    const int e = r < rt_entry_of_robot_.size() ? rt_entry_of_robot_[r] : -1;
    const bool claimed = e >= 0 && rt_claimed_[static_cast<size_t>(e)];
    const bool wheel = e >= 0 && joints_[static_cast<size_t>(e)].component == Component::kMobility;
    if (wheel || (e < 0 && std::find(description_.component_joints[index_of(Component::kMobility)].begin(),
      description_.component_joints[index_of(Component::kMobility)].end(), r) !=
      description_.component_joints[index_of(Component::kMobility)].end()))
    {
      input.velocity_mode[r] = true;  // wheels stand still
      input.target[r] = 0.0;
      input.feedback_gain[r] = 10;
      input.feedforward_torque[r] = 0.0;
      continue;
    }
    double desired = rt_sent_[r];
    unsigned int gain = 10;
    double ff = 0.0;
    if (claimed) {
      const auto & j = joints_[static_cast<size_t>(e)];
      const double p = rt_values_[j.cmd_position];
      if (std::isfinite(p)) {
        desired = p;
      }
      gain = static_cast<unsigned int>(rt_values_[j.cmd_feedback_gain]);
      ff = rt_values_[j.cmd_effort];
    }
    // Robot rules: the joint drivers refuse large single-tick jumps and the control manager
    // faults when a target is more than 0.2 rad from the position.
    desired = std::clamp(desired, rt_sent_[r] - rt_max_step_, rt_sent_[r] + rt_max_step_);
    desired = std::clamp(desired, state.position[r] - rt_max_tracking_error_, state.position[r] + rt_max_tracking_error_);
    rt_sent_[r] = desired;
    input.velocity_mode[r] = false;
    input.target[r] = desired;
    input.feedback_gain[r] = std::min(10u, gain);
    input.feedforward_torque[r] = std::isfinite(ff) ? ff : 0.0;
  }
  input.finish = fatal_.load() || !stream_thread_run_.load();
}

void Rby1SystemInterface::stop_streaming() noexcept
{
  const bool was_running = stream_thread_run_.exchange(false);
  if (stream_thread_.joinable()) {
    stream_thread_.join();
  }
  if (!backend_ || !was_running) {
    return;
  }
  if (backend_->realtime_running()) {
    backend_->stop_realtime();  // the robot holds the last position when the loop finishes
    streaming_ = false;
    return;
  }
  try {
    if (backend_->stream_open() && !fatal_) {
      // Leave torso, arms and head in position control where they are before closing.
      RobotCommandRequest request;
      std::array<std::vector<size_t>, kComponentCount> ids;
      for (size_t j = 0; j < joints_.size(); ++j) {
        ids[index_of(joints_[j].component)].push_back(j);
      }
      for (const auto component : {Component::kTorso, Component::kRightArm, Component::kLeftArm, Component::kHead}) {
        if (!ids[index_of(component)].empty()) {
          request.at(component) = make_hold_command(joints_, ids[index_of(component)], position_, parameters_);
        }
      }
      if (!request.empty()) {
        // A different structure needs its own stream (see stream_loop).
        backend_->close_stream();
        std::string message;
        if (backend_->open_stream(stream_priority_, message)) {
          backend_->send(request, send_timeout_ms_);
        }
      }
    }
    backend_->close_stream();
  } catch (...) {
  }
  streaming_ = false;
  stream_ok_ = false;
}

}  // namespace rby1_hardware_interface

PLUGINLIB_EXPORT_CLASS(rby1_hardware_interface::Rby1SystemInterface, hardware_interface::SystemInterface)
