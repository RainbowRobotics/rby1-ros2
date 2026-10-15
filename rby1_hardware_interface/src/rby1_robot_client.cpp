#include "rby1_hardware_interface/rby1_robot_client.hpp"

#include <algorithm>
#include <cctype>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/log.h"
#include "rby1-sdk/robot_command_builder.h"

namespace rby1_hardware_interface
{
namespace
{

constexpr int32_t kStateNone = 0;
constexpr int32_t kStateIdle = 1;
constexpr int32_t kStateEnable = 2;
constexpr int32_t kStateExecuting = 3;
constexpr int32_t kStateMajorFault = 4;
constexpr int32_t kStateMinorFault = 5;

template <typename StateT>
bool has_recent_update(const StateT & state)
{
  return state.time_since_last_update.tv_sec != 0 || state.time_since_last_update.tv_nsec != 0;
}

template <typename Vector3T>
std::array<double, 3> to_array3(const Vector3T & value)
{
  return {static_cast<double>(value[0]), static_cast<double>(value[1]), static_cast<double>(value[2])};
}

std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

CommandResult result(bool success, const std::string & message)
{
  return CommandResult{success, message};
}

int32_t map_control_manager_state(const rb::ControlManagerState & control_manager_state)
{
  if (control_manager_state.state == rb::ControlManagerState::State::kMinorFault) {
    return kStateMinorFault;
  }
  if (control_manager_state.state == rb::ControlManagerState::State::kMajorFault) {
    return kStateMajorFault;
  }
  if (control_manager_state.state == rb::ControlManagerState::State::kIdle) {
    return kStateIdle;
  }
  if (control_manager_state.state == rb::ControlManagerState::State::kEnabled) {
    if (control_manager_state.control_state == rb::ControlManagerState::ControlState::kExecuting) {
      return kStateExecuting;
    }
    return kStateEnable;
  }
  return kStateNone;
}

template <typename ToolFlangeT, typename FtSensorT>
ToolFlangeStatus make_tool_flange_status(const ToolFlangeT & tool_flange, const FtSensorT & ft_sensor)
{
  ToolFlangeStatus status;
  const bool flange_valid = has_recent_update(tool_flange);
  const bool ft_valid = has_recent_update(ft_sensor);
  status.connected = flange_valid;

  if (ft_valid) {
    status.ft_force = to_array3(ft_sensor.force);
    status.ft_torque = to_array3(ft_sensor.torque);
  }
  if (flange_valid) {
    status.gyro = to_array3(tool_flange.gyro);
    status.acceleration = to_array3(tool_flange.acceleration);
    status.switch_a = tool_flange.switch_A;
    status.output_voltage = tool_flange.output_voltage;
    status.digital_input_a = tool_flange.digital_input_A;
    status.digital_input_b = tool_flange.digital_input_B;
    status.digital_output_a = tool_flange.digital_output_A;
    status.digital_output_b = tool_flange.digital_output_B;
  }
  return status;
}

}  // namespace

class Rby1RobotClient::ImplBase
{
public:
  virtual ~ImplBase() = default;
  virtual void connect() = 0;
  virtual void disconnect() noexcept = 0;
  virtual bool is_connected() const = 0;
  virtual std::vector<std::string> joint_names() const = 0;
  virtual RobotStateSnapshot read_state() = 0;
  virtual CommandResult set_power(const std::string & selector, bool enabled) = 0;
  virtual CommandResult set_servo(const std::string & selector, bool enabled) = 0;
  virtual CommandResult set_brake(const std::string & selector, bool engaged) = 0;
  virtual CommandResult set_tool_flange_output_voltage(int voltage) = 0;
  virtual CommandResult set_gravity_compensation(const std::string & part_name, bool enabled) = 0;
  virtual CommandResult set_control_manager(ControlManagerCommand command) = 0;
  virtual CommandResult stop(int priority) = 0;
  virtual CommandResult cancel_control() = 0;
  virtual CommandResult jog(
    const std::string & joint_name, JogMode mode, double value, double velocity_limit,
    double acceleration_limit, double timeout_sec, int priority, int32_t & finish_code) = 0;
  virtual CommandResult set_time_scale(double scale, double & applied) = 0;
  virtual CommandResult set_led(uint8_t r, uint8_t g, uint8_t b, double duration, double transition_time,
    bool blinking, double blinking_frequency) = 0;
  virtual CommandResult set_tool_flange_digital_output(const std::string & side, unsigned int channel, bool state) = 0;
  virtual CommandResult get_parameter(const std::string & name, std::string & value) = 0;
  virtual CommandResult set_parameter(const std::string & name, const std::string & value, bool write_db) = 0;
  virtual CommandResult reset_odometry(double x, double y, double yaw) = 0;
  virtual CommandResult get_logs(
    unsigned int count, std::vector<std::string> & fault_logs, std::vector<std::string> & entries) = 0;
  virtual CommandResult get_position_pid_gain(const std::string & device, PidGains & gains) = 0;
  virtual CommandResult set_position_pid_gain(const std::string & device, const PidGains & gains) = 0;
  virtual CommandResult home_offset_reset(const std::string & joint_name) = 0;
  virtual CommandResult set_preset_position(const std::string & joint_name) = 0;
};

template <typename ModelType>
class Rby1RobotClientImpl final : public Rby1RobotClient::ImplBase
{
public:
  explicit Rby1RobotClientImpl(RobotClientOptions options)
  : options_(std::move(options))
  {
  }

  void connect() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      robot_ = rb::Robot<ModelType>::Create(options_.robot_ip);
      if (!robot_ || !robot_->Connect()) {
        robot_.reset();
        connected_ = false;
        throw std::runtime_error("failed to connect to RBY1 robot at " + options_.robot_ip);
      }
      info_ = robot_->GetRobotInfo();
      connected_ = true;
    } catch (const std::exception &) {
      robot_.reset();
      connected_ = false;
      throw;
    } catch (...) {
      robot_.reset();
      connected_ = false;
      throw std::runtime_error(
        "RBY1 SDK threw a non-standard exception while connecting to " + options_.robot_ip);
    }
  }

  void disconnect() noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_.reset();
    connected_ = false;
  }

  bool is_connected() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return connected_ && static_cast<bool>(robot_);
  }

  std::vector<std::string> joint_names() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(info_.joint_infos.size());
    for (const auto & joint_info : info_.joint_infos) {
      names.push_back(joint_info.name);
    }
    return names;
  }

  RobotStateSnapshot read_state() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();

    const auto state = robot_->GetState();
    const auto control_manager_state = robot_->GetControlManagerState();

    RobotStateSnapshot snapshot;
    snapshot.control_manager_state = map_control_manager_state(control_manager_state);
    snapshot.control_state = static_cast<int32_t>(control_manager_state.control_state);
    snapshot.time_scale = control_manager_state.time_scale;
    snapshot.unlimited_mode_enabled = control_manager_state.unlimited_mode_enabled;
    snapshot.joint_names.reserve(info_.joint_infos.size());
    snapshot.joints.reserve(info_.joint_infos.size());

    for (size_t index = 0; index < info_.joint_infos.size(); ++index) {
      const auto & joint_info = info_.joint_infos[index];
      JointSample sample;
      sample.position = state.position[index];
      sample.velocity = state.velocity[index];
      sample.effort = state.torque[index];
      sample.has_brake = joint_info.has_brake;
      sample.is_ready = state.joint_states[index].is_ready;
      snapshot.joint_names.push_back(joint_info.name);
      snapshot.joints.emplace(joint_info.name, sample);
    }

    fill_brakes(snapshot.brakes, state);

    snapshot.left_tool_flange = make_tool_flange_status(state.tool_flange_left, state.ft_sensor_left);
    snapshot.right_tool_flange = make_tool_flange_status(state.tool_flange_right, state.ft_sensor_right);
    snapshot.tool_flange_connected = {
      snapshot.left_tool_flange.connected,
      snapshot.right_tool_flange.connected
    };

    snapshot.emo_state = !state.emo_states.empty() &&
      state.emo_states[0].state == rb::EMOState::State::kPressed;
    snapshot.center_of_mass = to_array3(state.center_of_mass);

    snapshot.battery.available = true;
    snapshot.battery.voltage = state.battery_state.voltage;
    snapshot.battery.current = state.battery_state.current;
    snapshot.battery.percentage = state.battery_state.level_percent / 100.0;

    return snapshot;
  }

  CommandResult set_power(const std::string & selector, bool enabled) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const bool success = enabled ? robot_->PowerOn(selector) : robot_->PowerOff(selector);
    return result(success, success ? "power command succeeded" : "power command failed");
  }

  CommandResult set_servo(const std::string & selector, bool enabled) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const bool success = enabled ? robot_->ServoOn(selector) : robot_->ServoOff(selector);
    return result(success, success ? "servo command succeeded" : "servo command failed");
  }

  CommandResult set_brake(const std::string & selector, bool engaged) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const auto control_manager_state = robot_->GetControlManagerState();
    if (control_manager_state.state != rb::ControlManagerState::State::kIdle) {
      return result(false, "brakes can only be changed while the control manager is idle");
    }
    const bool success = engaged ? robot_->BrakeEngage(selector) : robot_->BrakeRelease(selector);
    return result(success, success ? "brake command succeeded" : "brake command failed");
  }

  CommandResult set_tool_flange_output_voltage(int voltage) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    if (voltage != 0 && voltage != 12 && voltage != 24) {
      return result(false, "tool flange voltage must be 0, 12, or 24");
    }
    const bool right_success = robot_->SetToolFlangeOutputVoltage("right", voltage);
    const bool left_success = robot_->SetToolFlangeOutputVoltage("left", voltage);
    if (right_success && left_success) {
      return result(true, "tool flange voltage command succeeded");
    }
    std::string failed;
    if (!right_success) failed += "right";
    if (!left_success) {
      if (!failed.empty()) failed += ", ";
      failed += "left";
    }
    return result(false, "tool flange voltage command failed for: " + failed);
  }

  CommandResult set_gravity_compensation(const std::string & part_name, bool enabled) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();

    if (part_name != "torso" && part_name != "right_arm" && part_name != "left_arm") {
      return result(false, "part_name must be torso, right_arm, or left_arm");
    }

    const auto control_manager_state = robot_->GetControlManagerState();
    if (control_manager_state.control_state == rb::ControlManagerState::ControlState::kExecuting ||
      control_manager_state.control_state == rb::ControlManagerState::ControlState::kSwitching)
    {
      return result(false, "gravity compensation cannot be changed while control is executing or switching");
    }

    auto gravity_builder = rb::GravityCompensationCommandBuilder();
    gravity_builder.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(10.0));
    gravity_builder.SetOn(enabled);

    auto body_component_builder = rb::BodyComponentBasedCommandBuilder();
    if (part_name == "torso") {
      body_component_builder.SetTorsoCommand(rb::TorsoCommandBuilder(gravity_builder));
    } else if (part_name == "right_arm") {
      body_component_builder.SetRightArmCommand(rb::ArmCommandBuilder(gravity_builder));
    } else {
      body_component_builder.SetLeftArmCommand(rb::ArmCommandBuilder(gravity_builder));
    }

    auto component_builder = rb::ComponentBasedCommandBuilder();
    component_builder.SetBodyCommand(rb::BodyCommandBuilder(body_component_builder));
    (void)robot_->SendCommand(rb::RobotCommandBuilder().SetCommand(component_builder));

    return result(true, "gravity compensation command sent");
  }

  CommandResult set_control_manager(ControlManagerCommand command) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();

    bool success = false;
    std::string command_name;
    switch (command) {
      case ControlManagerCommand::kEnable:
        command_name = "enable";
        success = robot_->EnableControlManager();
        break;
      case ControlManagerCommand::kDisable:
        command_name = "disable";
        success = robot_->DisableControlManager();
        break;
      case ControlManagerCommand::kResetFault:
        command_name = "reset fault";
        success = robot_->ResetFaultControlManager();
        break;
      case ControlManagerCommand::kEnableUnlimited:
        command_name = "enable (unlimited)";
        success = robot_->EnableControlManager(true);
        break;
    }
    return result(success, success ? "control manager " + command_name + " succeeded" : "control manager " + command_name + " failed");
  }

  CommandResult stop(int priority) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    rb::StopCommandBuilder stop_builder;
    rb::WholeBodyCommandBuilder whole_body(stop_builder);
    rb::RobotCommandBuilder command(whole_body);
    auto handler = robot_->SendCommand(command, priority);
    if (!handler || !handler->WaitFor(2000)) {
      return result(false, "stop command did not finish within 2 s");
    }
    return result(true, "stopped");
  }

  CommandResult cancel_control() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const bool ok = robot_->CancelControl();
    return result(ok, ok ? "control cancelled" : "CancelControl failed");
  }

  CommandResult jog(
    const std::string & joint_name, JogMode mode, double value, double velocity_limit,
    double acceleration_limit, double timeout_sec, int priority, int32_t & finish_code) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const auto cm = robot_->GetControlManagerState();
    if (cm.control_state == rb::ControlManagerState::ControlState::kExecuting) {
      return result(false, "another command is executing (deactivate the ros2_control controllers first)");
    }
    rb::JogCommandBuilder jog_builder;
    jog_builder.SetJointName(joint_name);
    if (velocity_limit > 0.0) {jog_builder.SetVelocityLimit(velocity_limit);}
    if (acceleration_limit > 0.0) {jog_builder.SetAccelerationLimit(acceleration_limit);}
    switch (mode) {
      case JogMode::kAbsolute: jog_builder.SetCommand(rb::JogCommandBuilder::AbsolutePosition(value)); break;
      case JogMode::kRelative: jog_builder.SetCommand(rb::JogCommandBuilder::RelativePosition(value)); break;
      case JogMode::kOneStep: jog_builder.SetCommand(rb::JogCommandBuilder::OneStep(value > 0.0)); break;
    }
    rb::RobotCommandBuilder command(jog_builder);
    auto handler = robot_->SendCommand(command, priority);
    if (!handler) {
      return result(false, "jog command was not accepted");
    }
    if (timeout_sec > 0.0) {
      if (!handler->WaitFor(static_cast<int>(timeout_sec * 1000.0))) {
        return result(false, "jog did not finish in time");
      }
      const auto feedback = handler->Get();
      finish_code = static_cast<int32_t>(feedback.finish_code());
      const bool ok = feedback.finish_code() == rb::RobotCommandFeedback::FinishCode::kOk;
      return result(ok, ok ? "jog finished" : "jog finished with code " + std::to_string(finish_code));
    }
    return result(true, "jog sent");
  }

  CommandResult set_time_scale(double scale, double & applied) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    if (!(scale > 0.0 && scale <= 1.0)) {
      return result(false, "time scale must be in (0, 1]");
    }
    applied = robot_->SetTimeScale(scale);
    return result(true, "time scale " + std::to_string(applied));
  }

  CommandResult set_led(uint8_t r, uint8_t g, uint8_t b, double duration, double transition_time, bool blinking,
    double blinking_frequency) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const bool ok = robot_->SetLEDColor(rb::Color(r, g, b), duration, transition_time, blinking,
        blinking_frequency > 0.0 ? blinking_frequency : 1.0);
    return result(ok, ok ? "LED set" : "SetLEDColor failed");
  }

  CommandResult set_tool_flange_digital_output(const std::string & side, unsigned int channel, bool state) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    if (side != "right" && side != "left") {
      return result(false, "side must be right or left");
    }
    const bool ok = robot_->SetToolFlangeDigitalOutput(side, channel, state);
    return result(ok, ok ? "digital output set" : "SetToolFlangeDigitalOutput failed");
  }

  CommandResult get_parameter(const std::string & name, std::string & value) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    value = robot_->GetParameter(name);
    return result(true, "ok");
  }

  CommandResult set_parameter(const std::string & name, const std::string & value, bool write_db) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const bool ok = robot_->SetParameter(name, value, write_db);
    return result(ok, ok ? (write_db ? "parameter set and saved" : "parameter set") : "SetParameter failed");
  }

  CommandResult reset_odometry(double x, double y, double yaw) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const bool ok = robot_->ResetOdometry(yaw, Eigen::Vector2d(x, y));
    return result(ok, ok ? "odometry reset" : "ResetOdometry failed");
  }

  CommandResult get_logs(
    unsigned int count, std::vector<std::string> & fault_logs, std::vector<std::string> & entries) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    fault_logs = robot_->GetFaultLogList();
    entries.clear();
    if (count > 0) {
      for (const auto & log : robot_->GetLastLog(count)) {
        entries.push_back("[" + rb::to_string(log.level) + "] " + log.message);
      }
    }
    return result(true, "ok");
  }

  CommandResult get_position_pid_gain(const std::string & device, PidGains & gains) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const auto g = robot_->GetPositionPIDGain(device);
    gains = {g.p_gain, g.i_gain, g.d_gain};
    return result(true, "ok");
  }

  CommandResult set_position_pid_gain(const std::string & device, const PidGains & gains) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const bool ok = robot_->SetPositionPIDGain(device, gains.p, gains.i, gains.d);
    return result(ok, ok ? "PID gain set" : "SetPositionPIDGain failed");
  }

  CommandResult home_offset_reset(const std::string & joint_name) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const bool ok = robot_->HomeOffsetReset(joint_name);
    return result(ok, ok ? "home offset reset" : "HomeOffsetReset failed");
  }

  CommandResult set_preset_position(const std::string & joint_name) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_connected();
    const bool ok = robot_->SetPresetPosition(joint_name);
    return result(ok, ok ? "preset position set" : "SetPresetPosition failed");
  }

private:
  template <typename StateT>
  void fill_brakes(BrakeGroups & brakes, const StateT & state) const
  {
    fill_brake_group(brakes.torso, info_.torso_joint_idx, state);
    fill_brake_group(brakes.right_arm, info_.right_arm_joint_idx, state);
    fill_brake_group(brakes.left_arm, info_.left_arm_joint_idx, state);
    fill_brake_group(brakes.head, info_.head_joint_idx, state);
  }

  template <typename StateT>
  void fill_brake_group(
    std::vector<bool> & brake_group,
    const std::vector<unsigned int> & joint_indexes,
    const StateT & state) const
  {
    brake_group.resize(joint_indexes.size());
    for (size_t index = 0; index < joint_indexes.size(); ++index) {
      const auto joint_index = joint_indexes[index];
      const auto & joint_info = info_.joint_infos[joint_index];
      brake_group[index] = joint_info.has_brake && !state.joint_states[joint_index].is_ready;
    }
  }

  void require_connected() const
  {
    if (!connected_ || !robot_) {
      throw std::runtime_error("RBY1 robot client is not connected");
    }
  }

  RobotClientOptions options_;
  mutable std::mutex mutex_;
  std::shared_ptr<rb::Robot<ModelType>> robot_;
  rb::RobotInfo info_;
  bool connected_{false};
};

std::string normalize_model_name(const std::string & model)
{
  const auto normalized = to_lower(model);
  if (normalized == "a" || normalized == "rby1a") {
    return "a";
  }
  if (normalized == "m" || normalized == "rby1m") {
    return "m";
  }
  if (normalized == "ub" || normalized == "rby1ub") {
    return "ub";
  }
  throw std::invalid_argument("unsupported RBY1 model: " + model);
}

Rby1RobotClient::Rby1RobotClient(RobotClientOptions options)
: options_(std::move(options))
{
  options_.model = normalize_model_name(options_.model);
  if (options_.model == "a") {
    impl_ = std::make_unique<Rby1RobotClientImpl<rb::y1_model::A>>(options_);
  } else if (options_.model == "m") {
    impl_ = std::make_unique<Rby1RobotClientImpl<rb::y1_model::M>>(options_);
  } else {
    impl_ = std::make_unique<Rby1RobotClientImpl<rb::y1_model::UB>>(options_);
  }
}

Rby1RobotClient::~Rby1RobotClient() = default;
Rby1RobotClient::Rby1RobotClient(Rby1RobotClient &&) noexcept = default;
Rby1RobotClient & Rby1RobotClient::operator=(Rby1RobotClient &&) noexcept = default;

void Rby1RobotClient::connect()
{
  impl_->connect();
}

void Rby1RobotClient::disconnect() noexcept
{
  impl_->disconnect();
}

bool Rby1RobotClient::is_connected() const
{
  return impl_->is_connected();
}

const RobotClientOptions & Rby1RobotClient::options() const
{
  return options_;
}

std::vector<std::string> Rby1RobotClient::joint_names() const
{
  return impl_->joint_names();
}

RobotStateSnapshot Rby1RobotClient::read_state()
{
  return impl_->read_state();
}

CommandResult Rby1RobotClient::set_power(const std::string & selector, bool enabled)
{
  return impl_->set_power(selector, enabled);
}

CommandResult Rby1RobotClient::set_servo(const std::string & selector, bool enabled)
{
  return impl_->set_servo(selector, enabled);
}

CommandResult Rby1RobotClient::set_brake(const std::string & selector, bool engaged)
{
  return impl_->set_brake(selector, engaged);
}

CommandResult Rby1RobotClient::set_tool_flange_output_voltage(int voltage)
{
  return impl_->set_tool_flange_output_voltage(voltage);
}

CommandResult Rby1RobotClient::set_gravity_compensation(const std::string & part_name, bool enabled)
{
  return impl_->set_gravity_compensation(part_name, enabled);
}

CommandResult Rby1RobotClient::set_control_manager(ControlManagerCommand command)
{
  return impl_->set_control_manager(command);
}

CommandResult Rby1RobotClient::stop(int priority) {return impl_->stop(priority);}
CommandResult Rby1RobotClient::cancel_control() {return impl_->cancel_control();}
CommandResult Rby1RobotClient::jog(
  const std::string & joint_name, JogMode mode, double value, double velocity_limit,
  double acceleration_limit, double timeout_sec, int priority, int32_t & finish_code)
{
  return impl_->jog(joint_name, mode, value, velocity_limit, acceleration_limit, timeout_sec, priority, finish_code);
}
CommandResult Rby1RobotClient::set_time_scale(double scale, double & applied)
{
  return impl_->set_time_scale(scale, applied);
}
CommandResult Rby1RobotClient::set_led(
  uint8_t r, uint8_t g, uint8_t b, double duration, double transition_time, bool blinking, double blinking_frequency)
{
  return impl_->set_led(r, g, b, duration, transition_time, blinking, blinking_frequency);
}
CommandResult Rby1RobotClient::set_tool_flange_digital_output(const std::string & side, unsigned int channel, bool state)
{
  return impl_->set_tool_flange_digital_output(side, channel, state);
}
CommandResult Rby1RobotClient::get_parameter(const std::string & name, std::string & value)
{
  return impl_->get_parameter(name, value);
}
CommandResult Rby1RobotClient::set_parameter(const std::string & name, const std::string & value, bool write_db)
{
  return impl_->set_parameter(name, value, write_db);
}
CommandResult Rby1RobotClient::reset_odometry(double x, double y, double yaw) {return impl_->reset_odometry(x, y, yaw);}
CommandResult Rby1RobotClient::get_logs(
  unsigned int count, std::vector<std::string> & fault_logs, std::vector<std::string> & entries)
{
  return impl_->get_logs(count, fault_logs, entries);
}
CommandResult Rby1RobotClient::get_position_pid_gain(const std::string & device, PidGains & gains)
{
  return impl_->get_position_pid_gain(device, gains);
}
CommandResult Rby1RobotClient::set_position_pid_gain(const std::string & device, const PidGains & gains)
{
  return impl_->set_position_pid_gain(device, gains);
}
CommandResult Rby1RobotClient::home_offset_reset(const std::string & joint_name) {return impl_->home_offset_reset(joint_name);}
CommandResult Rby1RobotClient::set_preset_position(const std::string & joint_name)
{
  return impl_->set_preset_position(joint_name);
}

}  // namespace rby1_hardware_interface
