// rby1-sdk implementation of RobotBackend. This is the only translation unit of the
// hardware interface that includes SDK headers.

#include <Eigen/Geometry>

#include <cmath>
#include <functional>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

#include "rby1_hardware_interface/robot_backend.hpp"

namespace rby1_hardware_interface
{
namespace
{

Eigen::VectorXd to_eigen(const std::vector<double> & values)
{
  return Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
}

Eigen::Matrix4d to_matrix(const Pose & pose)
{
  const Eigen::Quaterniond q(pose.orientation[3], pose.orientation[0], pose.orientation[1], pose.orientation[2]);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T(0, 3) = pose.position[0];
  T(1, 3) = pose.position[1];
  T(2, 3) = pose.position[2];
  return T;
}

// The SDK builders are neither copyable nor movable: every builder is filled in place and
// wrapped by its parent while it is alive.

template <typename B>
void set_header(B & b, const CommandTiming & timing)
{
  rb::CommandHeaderBuilder header;
  header.SetControlHoldTime(timing.control_hold_time);
  b.SetCommandHeader(header);
}

template <typename C> struct InnerOf;
template <> struct InnerOf<JointPositionCommand> {using type = rb::JointPositionCommandBuilder;};
template <> struct InnerOf<JointGroupPositionCommand> {using type = rb::JointGroupPositionCommandBuilder;};
template <> struct InnerOf<JointImpedanceCommand> {using type = rb::JointImpedanceControlCommandBuilder;};
template <> struct InnerOf<GravityCompensationCommand> {using type = rb::GravityCompensationCommandBuilder;};
template <> struct InnerOf<JointVelocityCommand> {using type = rb::JointVelocityCommandBuilder;};
template <> struct InnerOf<SE2VelocityCommand> {using type = rb::SE2VelocityCommandBuilder;};
template <> struct InnerOf<CartesianCommand> {using type = rb::CartesianCommandBuilder;};
template <> struct InnerOf<CartesianImpedanceCommand> {using type = rb::CartesianImpedanceControlCommandBuilder;};
template <> struct InnerOf<ImpedanceCommand> {using type = rb::ImpedanceControlCommandBuilder;};
template <> struct InnerOf<OptimalControlCommand> {using type = rb::OptimalControlCommandBuilder;};

void fill(rb::JointPositionCommandBuilder & b, const JointPositionCommand & c)
{
  set_header(b, c.timing);
  b.SetMinimumTime(c.timing.minimum_time).SetPosition(to_eigen(c.position));
  if (!c.velocity_limit.empty()) {b.SetVelocityLimit(to_eigen(c.velocity_limit));}
  if (!c.acceleration_limit.empty()) {b.SetAccelerationLimit(to_eigen(c.acceleration_limit));}
}

void fill(rb::JointGroupPositionCommandBuilder & b, const JointGroupPositionCommand & c)
{
  set_header(b, c.timing);
  b.SetJointNames(c.joint_names).SetMinimumTime(c.timing.minimum_time).SetPosition(to_eigen(c.position));
  if (!c.velocity_limit.empty()) {b.SetVelocityLimit(to_eigen(c.velocity_limit));}
  if (!c.acceleration_limit.empty()) {b.SetAccelerationLimit(to_eigen(c.acceleration_limit));}
}

void fill(rb::JointImpedanceControlCommandBuilder & b, const JointImpedanceCommand & c)
{
  set_header(b, c.timing);
  b.SetMinimumTime(c.timing.minimum_time)
  .SetPosition(to_eigen(c.position))
  .SetStiffness(to_eigen(c.stiffness))
  .SetTorqueLimit(to_eigen(c.torque_limit))
  .SetDampingRatio(c.damping_ratio);
  if (!c.velocity_limit.empty()) {b.SetVelocityLimit(to_eigen(c.velocity_limit));}
  if (!c.acceleration_limit.empty()) {b.SetAccelerationLimit(to_eigen(c.acceleration_limit));}
}

void fill(rb::GravityCompensationCommandBuilder & b, const GravityCompensationCommand & c)
{
  set_header(b, c.timing);
  b.SetOn(c.on);
}

void fill(rb::JointVelocityCommandBuilder & b, const JointVelocityCommand & c)
{
  set_header(b, c.timing);
  b.SetMinimumTime(c.timing.minimum_time).SetVelocity(to_eigen(c.velocity));
  if (!c.acceleration_limit.empty()) {b.SetAccelerationLimit(to_eigen(c.acceleration_limit));}
}

void fill(rb::SE2VelocityCommandBuilder & b, const SE2VelocityCommand & c)
{
  set_header(b, c.timing);
  b.SetMinimumTime(c.timing.minimum_time)
  .SetVelocity(Eigen::Vector2d(c.linear_x, c.linear_y), c.angular_z)
  .SetAccelerationLimit(
    Eigen::Vector2d(c.linear_acceleration_limit_x, c.linear_acceleration_limit_y),
    c.angular_acceleration_limit);
}

void fill(rb::CartesianCommandBuilder & b, const CartesianCommand & c)
{
  set_header(b, c.timing);
  b.SetMinimumTime(c.timing.minimum_time);
  for (const auto & t : c.targets) {
    b.AddTarget(
      t.ref_link, t.link, to_matrix(t.pose), t.linear_velocity_limit, t.angular_velocity_limit,
      t.acceleration_limit_scaling);
  }
  for (const auto & j : c.joint_targets) {
    b.AddJointPositionTarget(j.joint_name, j.position, j.velocity_limit, j.acceleration_limit);
  }
  if (c.stop_position_tracking_error) {b.SetStopPositionTrackingError(*c.stop_position_tracking_error);}
  if (c.stop_orientation_tracking_error) {
    b.SetStopOrientationTrackingError(*c.stop_orientation_tracking_error);
  }
  if (c.stop_joint_position_tracking_error) {
    b.SetStopJointPositionTrackingError(*c.stop_joint_position_tracking_error);
  }
}

void fill(rb::CartesianImpedanceControlCommandBuilder & b, const CartesianImpedanceCommand & c)
{
  set_header(b, c.timing);
  b.SetMinimumTime(c.timing.minimum_time);
  for (const auto & t : c.targets) {
    b.AddTarget(
      t.ref_link, t.link, to_matrix(t.pose), t.linear_velocity_limit, t.angular_velocity_limit,
      t.linear_acceleration_limit, t.angular_acceleration_limit);
  }
  for (const auto & j : c.joint_targets) {
    b.AddJointPositionTarget(j.joint_name, j.position, j.velocity_limit, j.acceleration_limit);
  }
  if (c.nullspace) {
    b.SetNullspaceJointTarget(
      to_eigen(c.nullspace->position), to_eigen(c.nullspace->weight), c.nullspace->k_p,
      c.nullspace->k_d, c.nullspace->cost_weight);
  }
  if (!c.joint_stiffness.empty()) {b.SetJointStiffness(to_eigen(c.joint_stiffness));}
  if (c.joint_damping_ratio) {b.SetJointDampingRatio(*c.joint_damping_ratio);}
  if (!c.joint_torque_limit.empty()) {b.SetJointTorqueLimit(to_eigen(c.joint_torque_limit));}
  for (const auto & limit : c.joint_limits) {
    b.AddJointLimit(limit.joint_name, limit.lower, limit.upper);
  }
  if (c.reset_reference) {b.SetResetReference(*c.reset_reference);}
  if (c.stop_position_tracking_error) {b.SetStopPositionTrackingError(*c.stop_position_tracking_error);}
  if (c.stop_orientation_tracking_error) {
    b.SetStopOrientationTrackingError(*c.stop_orientation_tracking_error);
  }
  if (c.stop_joint_position_tracking_error) {
    b.SetStopJointPositionTrackingError(*c.stop_joint_position_tracking_error);
  }
}

void fill(rb::ImpedanceControlCommandBuilder & b, const ImpedanceCommand & c)
{
  set_header(b, c.timing);
  b.SetReferenceLinkName(c.ref_link)
  .SetLinkName(c.link)
  .SetTransformation(to_matrix(c.pose))
  .SetTranslationWeight(Eigen::Vector3d(c.translation_weight[0], c.translation_weight[1], c.translation_weight[2]))
  .SetRotationWeight(Eigen::Vector3d(c.rotation_weight[0], c.rotation_weight[1], c.rotation_weight[2]))
  .SetDampingRatio(c.damping_ratio);
}

void fill(rb::OptimalControlCommandBuilder & b, const OptimalControlCommand & c)
{
  set_header(b, c.timing);
  for (const auto & t : c.targets) {
    b.AddCartesianTarget(t.ref_link, t.link, to_matrix(t.pose), t.translation_weight, t.rotation_weight);
  }
  if (c.center_of_mass) {
    const auto & com = *c.center_of_mass;
    b.SetCenterOfMassTarget(com.ref_link, Eigen::Vector3d(com.position[0], com.position[1], com.position[2]), com.weight);
  }
  for (const auto & j : c.joint_targets) {
    b.AddJointPositionTarget(j.joint_name, j.position, j.weight);
  }
  if (c.error_scaling) {b.SetErrorScaling(*c.error_scaling);}
  if (c.velocity_limit_scaling) {b.SetVelocityLimitScaling(*c.velocity_limit_scaling);}
  if (c.acceleration_limit_scaling) {b.SetAccelerationLimitScaling(*c.acceleration_limit_scaling);}
  if (c.stop_cost) {b.SetStopCost(*c.stop_cost);}
  if (c.min_delta_cost) {b.SetMinDeltaCost(*c.min_delta_cost);}
  if (c.patience) {b.SetPatience(*c.patience);}
}

// Fill the inner builder of `command`, wrap it into ComponentBuilder and hand that to `sink`.
// Unsupported combinations throw; the mode registry never produces them.
template <typename ComponentBuilder, typename Sink>
void with_component(const ComponentCommand & command, const char * component, Sink && sink)
{
  std::visit(
    [&](const auto & c) {
      using C = std::decay_t<decltype(c)>;
      if constexpr (std::is_same_v<C, std::monostate>) {
        throw std::logic_error(std::string("empty command for ") + component);
      } else {
        using Inner = typename InnerOf<C>::type;
        if constexpr (std::is_constructible_v<ComponentBuilder, const Inner &>) {
          Inner inner;
          fill(inner, c);
          ComponentBuilder wrapped(inner);
          sink(wrapped);
        } else {
          throw std::invalid_argument(std::string("command type not supported by component ") + component);
        }
      }
    }, command);
}

ControlManagerStatus to_status(rb::ControlManagerState::State state)
{
  switch (state) {
    case rb::ControlManagerState::State::kIdle: return ControlManagerStatus::kIdle;
    case rb::ControlManagerState::State::kEnabled: return ControlManagerStatus::kEnabled;
    case rb::ControlManagerState::State::kMinorFault: return ControlManagerStatus::kMinorFault;
    case rb::ControlManagerState::State::kMajorFault: return ControlManagerStatus::kMajorFault;
    default: return ControlManagerStatus::kUnknown;
  }
}

ControlStatus to_status(rb::ControlManagerState::ControlState state)
{
  switch (state) {
    case rb::ControlManagerState::ControlState::kIdle: return ControlStatus::kIdle;
    case rb::ControlManagerState::ControlState::kExecuting: return ControlStatus::kExecuting;
    case rb::ControlManagerState::ControlState::kSwitching: return ControlStatus::kSwitching;
    default: return ControlStatus::kUnknown;
  }
}

template <typename Model>
class SdkRobotBackend final : public RobotBackend
{
public:
  SdkRobotBackend(std::string model, std::string address)
  : model_(std::move(model)), address_(std::move(address))
  {
  }

  ~SdkRobotBackend() override
  {
    stop_realtime();
    disconnect();
  }

  RobotDescription connect(double timeout_sec) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_ = rb::Robot<Model>::Create(address_);
    const int timeout_ms = std::max(100, static_cast<int>(timeout_sec * 1000.0));
    if (!robot_ || !robot_->Connect(3, timeout_ms)) {
      robot_.reset();
      throw std::runtime_error("failed to connect to RB-Y1 at " + address_);
    }
    const auto info = robot_->GetRobotInfo();
    RobotDescription d;
    d.model = model_;
    d.model_name = info.robot_model_name;
    d.model_version = info.robot_model_version;
    d.robot_version = info.robot_version;
    d.sdk_version = info.sdk_version;
    for (const auto & joint : info.joint_infos) {
      d.joint_names.push_back(joint.name);
      d.has_brake.push_back(joint.has_brake);
    }
    auto copy = [](const std::vector<unsigned int> & idx) {
        return std::vector<size_t>(idx.begin(), idx.end());
      };
    d.component_joints[index_of(Component::kTorso)] = copy(info.torso_joint_idx);
    d.component_joints[index_of(Component::kRightArm)] = copy(info.right_arm_joint_idx);
    d.component_joints[index_of(Component::kLeftArm)] = copy(info.left_arm_joint_idx);
    d.component_joints[index_of(Component::kHead)] = copy(info.head_joint_idx);
    d.component_joints[index_of(Component::kMobility)] = copy(info.mobility_joint_idx);
    d.component_joints[index_of(Component::kBody)] = copy(info.body_joint_idx);
    if (d.joint_names.size() != static_cast<size_t>(Model::kRobotDOF)) {
      throw std::runtime_error(
              "robot reports " + std::to_string(d.joint_names.size()) + " joints but model '" + model_ +
              "' has " + std::to_string(Model::kRobotDOF) + "; check the model parameter");
    }
    return d;
  }

  void disconnect() noexcept override
  {
    stop_state_stream();
    close_stream();
    std::lock_guard<std::mutex> lock(mutex_);
    if (robot_) {
      try {
        robot_->Disconnect();
      } catch (...) {
      }
    }
    robot_.reset();
  }

  bool is_connected() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return robot_ && robot_->IsConnected();
  }

  void start_state_stream(double rate_hz, StateCallback callback) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_robot();
    robot_->StartStateUpdate(
      [cb = std::move(callback)](const rb::RobotState<Model> & s, const rb::ControlManagerState & cm) {
        StateSample out;
        out.received = std::chrono::steady_clock::now();
        out.robot_time = static_cast<double>(s.timestamp.tv_sec) + 1e-9 * static_cast<double>(s.timestamp.tv_nsec);
        constexpr auto n = static_cast<size_t>(Model::kRobotDOF);
        out.position.assign(s.position.data(), s.position.data() + n);
        out.velocity.assign(s.velocity.data(), s.velocity.data() + n);
        out.torque.assign(s.torque.data(), s.torque.data() + n);
        out.current.assign(s.current.data(), s.current.data() + n);
        out.is_ready.resize(n);
        for (size_t i = 0; i < n; ++i) {
          out.is_ready[i] = s.is_ready[static_cast<Eigen::Index>(i)];
        }
        out.control_manager = to_status(cm.state);
        out.control = to_status(cm.control_state);
        out.time_scale = cm.time_scale;
        for (const auto & emo : s.emo_states) {
          out.emo_pressed = out.emo_pressed || emo.state == rb::EMOState::State::kPressed;
        }
        out.odometry = {s.odometry(0, 2), s.odometry(1, 2), std::atan2(s.odometry(1, 0), s.odometry(0, 0))};
        for (int i = 0; i < 3; ++i) {
          out.ft_right[i] = s.ft_sensor_right.force[i];
          out.ft_right[i + 3] = s.ft_sensor_right.torque[i];
          out.ft_left[i] = s.ft_sensor_left.force[i];
          out.ft_left[i + 3] = s.ft_sensor_left.torque[i];
        }
        cb(out);
      }, rate_hz);
    state_stream_running_ = true;
  }

  void stop_state_stream() noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (robot_ && state_stream_running_) {
      try {
        robot_->StopStateUpdate();
      } catch (...) {
      }
    }
    state_stream_running_ = false;
  }

  bool ensure_ready(const ReadyOptions & options, std::string & message) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_robot();
    // PowerOn/ServoOn return when the request is accepted; the state follows later (48 V
    // ramp-up on the robot, simulation ticks in the simulator), so wait for it.
    auto wait_for = [](const std::function<bool()> & done, double seconds) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (!done()) {
          if (std::chrono::steady_clock::now() > end) {
            return false;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
      };
    if (!robot_->IsPowerOn(options.power_selector)) {
      // The control manager faults if power changes while it is enabled: power up first.
      if (options.auto_enable &&
        robot_->GetControlManagerState().state == rb::ControlManagerState::State::kEnabled)
      {
        robot_->DisableControlManager();
      }
      if (!options.auto_enable || !robot_->PowerOn(options.power_selector) ||
        !wait_for([&]() {return robot_->IsPowerOn(options.power_selector);}, options.enable_timeout))
      {
        message = "power '" + options.power_selector + "' is off";
        return false;
      }
    }
    if (!robot_->IsServoOn(options.servo_selector)) {
      if (!options.auto_enable || !robot_->ServoOn(options.servo_selector) ||
        !wait_for([&]() {return robot_->IsServoOn(options.servo_selector);}, options.enable_timeout))
      {
        message = "servo '" + options.servo_selector + "' is off";
        return false;
      }
    }
    auto cm = robot_->GetControlManagerState();
    if (cm.state == rb::ControlManagerState::State::kMajorFault) {
      message = "control manager has a major fault; reset it from the robot manager after checking the robot";
      return false;
    }
    if (cm.state == rb::ControlManagerState::State::kMinorFault) {
      if (!options.reset_minor_fault || !robot_->ResetFaultControlManager()) {
        message = "control manager has a minor fault";
        return false;
      }
      cm = robot_->GetControlManagerState();
    }
    if (cm.state != rb::ControlManagerState::State::kEnabled) {
      if (!options.auto_enable || !robot_->EnableControlManager(false)) {
        message = "control manager is not enabled";
        return false;
      }
    }
    if (!robot_->WaitForControlReady(options.control_ready_timeout_ms)) {
      message = "control manager did not become ready";
      return false;
    }
    message = "ready";
    return true;
  }

  bool set_parameter(const std::string & name, const std::string & value) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_robot();
    return robot_->SetParameter(name, value, /*write_db=*/false);
  }

  bool open_stream(int priority, std::string & message) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_robot();
    try {
      sends_on_stream_ = 0;
      stream_ = robot_->CreateCommandStream(priority);
    } catch (const std::exception & e) {
      message = e.what();
      stream_.reset();
    }
    if (!stream_) {
      if (message.empty()) {message = "CreateCommandStream returned null";}
      return false;
    }
    return true;
  }

  bool stream_open() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<bool>(stream_);
  }

  StreamFeedback send(const RobotCommandRequest & request, int timeout_ms) override
  {
    StreamFeedback result;
    rb::ComponentBasedCommandBuilder component_based;
    try {
      fill_request(component_based, request);
    } catch (const std::exception & e) {
      result.status = StreamStatus::kError;
      result.message = std::string("invalid command: ") + e.what();
      return result;
    }
    rb::RobotCommandBuilder command(component_based);

    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_) {
      result.status = StreamStatus::kFinished;
      result.message = "stream is not open";
      return result;
    }
    try {
      const auto feedback = stream_->SendCommand(command, timeout_ms);
      const bool first = sends_on_stream_++ == 0;
      result.finish_code = static_cast<int32_t>(feedback.finish_code());
      if (feedback.status() == rb::RobotCommandFeedback::Status::kFinished) {
        if (feedback.finish_code() == rb::RobotCommandFeedback::FinishCode::kControlManagerFault) {
          result.status = StreamStatus::kFault;
          result.message = "control manager fault";
        } else {
          result.status = StreamStatus::kFinished;
          result.message = "stream finished (finish code " + std::to_string(result.finish_code) + ")";
          if (first && feedback.finish_code() == rb::RobotCommandFeedback::FinishCode::kCanceled) {
            result.message += ": rejected; another control with the same or a higher priority is active "
              "(raise stream_priority or cancel it with the robot manager)";
          }
        }
        stream_.reset();
      }
    } catch (const std::exception & e) {
      const std::string what = e.what();
      result.message = what;
      if (what.find("expired") != std::string::npos) {
        result.status = StreamStatus::kFinished;
        stream_.reset();
      } else if (what.find("Timeout") != std::string::npos || what.find("timeout") != std::string::npos) {
        result.status = StreamStatus::kTimeout;
      } else if (what.find("does not finish") != std::string::npos) {
        result.status = StreamStatus::kTimeout;
      } else {
        result.status = StreamStatus::kError;
        stream_.reset();
      }
    }
    return result;
  }

  void close_stream() noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_) {
      try {
        stream_->Cancel();
        stream_->WaitFor(1000);
      } catch (...) {
      }
      stream_.reset();
      // The control manager ends the stream's control asynchronously; a new stream with the
      // same priority is rejected while it is still executing.
      try {
        for (int i = 0; i < 50 && robot_; ++i) {
          if (robot_->GetControlManagerState().control_state != rb::ControlManagerState::ControlState::kExecuting) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      } catch (...) {
      }
    }
  }

  bool start_realtime(RealtimeCallback callback, int priority, std::string & message) override
  {
    stop_realtime();
    std::shared_ptr<rb::Robot<Model>> robot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!robot_) {
        message = "not connected";
        return false;
      }
      if (stream_) {
        message = "close the command stream before starting real-time control";
        return false;
      }
      robot = robot_;
    }
    rt_stop_ = false;
    rt_running_ = true;
    rt_thread_ = std::thread([this, robot, cb = std::move(callback), priority]() {
        constexpr auto n = static_cast<size_t>(Model::kRobotDOF);
        RealtimeState state;
        state.position.resize(n);
        state.velocity.resize(n);
        state.torque.resize(n);
        state.is_ready.resize(n);
        RealtimeInput input;
        input.velocity_mode.assign(n, false);
        input.target.assign(n, 0.0);
        input.feedback_gain.assign(n, 10);
        input.feedforward_torque.assign(n, 0.0);
        try {
          robot->Control(
            [&](const rb::ControlState<Model> & s) {
              state.t = s.t;
              for (size_t i = 0; i < n; ++i) {
                const auto k = static_cast<Eigen::Index>(i);
                state.position[i] = s.position[k];
                state.velocity[i] = s.velocity[k];
                state.torque[i] = s.torque[k];
                state.is_ready[i] = s.is_ready[k];
                input.target[i] = s.position[k];  // safe default if the callback leaves it
              }
              input.finish = false;
              cb(state, input);
              rb::ControlInput<Model> out;
              for (size_t i = 0; i < n; ++i) {
                const auto k = static_cast<Eigen::Index>(i);
                out.mode[k] = input.velocity_mode[i];
                out.target[k] = input.target[i];
                out.feedback_gain[k] = std::min(10u, input.feedback_gain[i]);
                out.feedforward_torque[k] = input.feedforward_torque[i];
              }
              out.finish = input.finish || rt_stop_;
              return out;
            }, 0, priority);
        } catch (...) {
        }
        rt_running_ = false;
      });
    return true;
  }

  void stop_realtime() noexcept override
  {
    rt_stop_ = true;
    if (rt_thread_.joinable()) {
      rt_thread_.join();
    }
    rt_running_ = false;
  }

  bool realtime_running() const override
  {
    return rt_running_;
  }

  bool cancel_control() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!robot_) {
      return false;
    }
    try {
      return robot_->CancelControl();
    } catch (...) {
      return false;
    }
  }

private:
  void require_robot() const
  {
    if (!robot_) {
      throw std::runtime_error("RB-Y1 backend is not connected");
    }
  }

  static void fill_request(rb::ComponentBasedCommandBuilder & component_based, const RobotCommandRequest & request)
  {
    bool any = false;
    const auto & body = request.at(Component::kBody);
    const bool torso = !std::holds_alternative<std::monostate>(request.at(Component::kTorso));
    const bool right = !std::holds_alternative<std::monostate>(request.at(Component::kRightArm));
    const bool left = !std::holds_alternative<std::monostate>(request.at(Component::kLeftArm));
    if (!std::holds_alternative<std::monostate>(body)) {
      if (torso || right || left) {
        throw std::invalid_argument("body command cannot be combined with torso/arm commands");
      }
      with_component<rb::BodyCommandBuilder>(body, "body", [&](const rb::BodyCommandBuilder & b) {
          component_based.SetBodyCommand(b);
        });
      any = true;
    } else if (torso || right || left) {
      rb::BodyComponentBasedCommandBuilder parts;
      if (torso) {
        with_component<rb::TorsoCommandBuilder>(request.at(Component::kTorso), "torso",
          [&](const rb::TorsoCommandBuilder & b) {parts.SetTorsoCommand(b);});
      }
      if (right) {
        with_component<rb::ArmCommandBuilder>(request.at(Component::kRightArm), "right_arm",
          [&](const rb::ArmCommandBuilder & b) {parts.SetRightArmCommand(b);});
      }
      if (left) {
        with_component<rb::ArmCommandBuilder>(request.at(Component::kLeftArm), "left_arm",
          [&](const rb::ArmCommandBuilder & b) {parts.SetLeftArmCommand(b);});
      }
      rb::BodyCommandBuilder body_builder(parts);
      component_based.SetBodyCommand(body_builder);
      any = true;
    }
    if (!std::holds_alternative<std::monostate>(request.at(Component::kHead))) {
      with_component<rb::HeadCommandBuilder>(request.at(Component::kHead), "head",
        [&](const rb::HeadCommandBuilder & b) {component_based.SetHeadCommand(b);});
      any = true;
    }
    if (!std::holds_alternative<std::monostate>(request.at(Component::kMobility))) {
      with_component<rb::MobilityCommandBuilder>(request.at(Component::kMobility), "mobility",
        [&](const rb::MobilityCommandBuilder & b) {component_based.SetMobilityCommand(b);});
      any = true;
    }
    if (!any) {
      throw std::invalid_argument("empty command request");
    }
  }

  const std::string model_;
  const std::string address_;
  mutable std::mutex mutex_;
  std::shared_ptr<rb::Robot<Model>> robot_;
  std::unique_ptr<rb::RobotCommandStreamHandler<Model>> stream_;
  size_t sends_on_stream_{0};
  bool state_stream_running_{false};
  std::thread rt_thread_;
  std::atomic<bool> rt_stop_{false};
  std::atomic<bool> rt_running_{false};
};

}  // namespace

std::unique_ptr<RobotBackend> make_sdk_backend(const std::string & model, const std::string & address)
{
  if (model == "a") {
    return std::make_unique<SdkRobotBackend<rb::y1_model::A>>(model, address);
  }
  if (model == "m") {
    return std::make_unique<SdkRobotBackend<rb::y1_model::M>>(model, address);
  }
  if (model == "ub") {
    return std::make_unique<SdkRobotBackend<rb::y1_model::UB>>(model, address);
  }
  throw std::invalid_argument("unsupported RB-Y1 model '" + model + "' (expected a, m or ub)");
}

}  // namespace rby1_hardware_interface
