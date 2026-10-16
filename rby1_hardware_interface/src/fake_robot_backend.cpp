#include "rby1_hardware_interface/fake_robot_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace rby1_hardware_interface
{
namespace
{

std::vector<size_t> range(size_t begin, size_t end)
{
  std::vector<size_t> out;
  for (size_t i = begin; i < end; ++i) {
    out.push_back(i);
  }
  return out;
}

std::vector<std::string> body_names(const std::vector<std::string> & torso)
{
  std::vector<std::string> out = torso;
  for (int i = 0; i < 7; ++i) {out.push_back("right_arm_" + std::to_string(i));}
  for (int i = 0; i < 7; ++i) {out.push_back("left_arm_" + std::to_string(i));}
  out.push_back("head_0");
  out.push_back("head_1");
  return out;
}

}  // namespace

RobotDescription FakeRobotBackend::describe(const std::string & model)
{
  RobotDescription d;
  d.model = model;
  d.model_version = "1.2";
  d.robot_version = "fake";
  d.sdk_version = "fake";
  std::vector<std::string> wheels;
  std::vector<std::string> torso = {"torso_0", "torso_1", "torso_2", "torso_3", "torso_4", "torso_5"};
  if (model == "a") {
    d.model_name = "A";
    wheels = {"right_wheel", "left_wheel"};
  } else if (model == "m") {
    d.model_name = "M";
    d.model_version = "1.3";
    wheels = {"wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl"};
  } else if (model == "ub") {
    d.model_name = "UB";
    torso = {"torso_hp", "torso_5"};
  } else {
    throw std::invalid_argument("unsupported RB-Y1 model '" + model + "'");
  }
  d.joint_names = wheels;
  for (const auto & n : body_names(torso)) {
    d.joint_names.push_back(n);
  }
  d.has_brake.assign(d.joint_names.size(), false);
  const size_t w = wheels.size();
  const size_t t = torso.size();
  d.component_joints[index_of(Component::kMobility)] = range(0, w);
  d.component_joints[index_of(Component::kTorso)] = range(w, w + t);
  d.component_joints[index_of(Component::kRightArm)] = range(w + t, w + t + 7);
  d.component_joints[index_of(Component::kLeftArm)] = range(w + t + 7, w + t + 14);
  d.component_joints[index_of(Component::kHead)] = range(w + t + 14, w + t + 16);
  d.component_joints[index_of(Component::kBody)] = range(w, w + t + 14);
  return d;
}

FakeRobotBackend::FakeRobotBackend(std::string model)
: model_(std::move(model)), description_(describe(model_))
{
  const size_t n = description_.joint_names.size();
  position_.assign(n, 0.0);
  velocity_.assign(n, 0.0);
  target_.assign(n, 0.0);
  target_velocity_.assign(n, 0.0);
  velocity_limit_.assign(n, 1.0);
  velocity_mode_.assign(n, false);
}

FakeRobotBackend::~FakeRobotBackend()
{
  stop_realtime();
  stop_state_stream();
}

RobotDescription FakeRobotBackend::connect(double)
{
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = true;
  return description_;
}

void FakeRobotBackend::disconnect() noexcept
{
  stop_state_stream();
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = false;
  stream_open_ = false;
}

bool FakeRobotBackend::is_connected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

void FakeRobotBackend::start_state_stream(double rate_hz, StateCallback callback)
{
  stop_state_stream();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = std::move(callback);
  }
  running_ = true;
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz));
  thread_ = std::thread([this, period]() {
        auto next = std::chrono::steady_clock::now();
        while (running_) {
          next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
          StateSample sample;
          StateCallback cb;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            step(period.count());
            sample.received = std::chrono::steady_clock::now();
            sample.robot_time = std::chrono::duration<double>(sample.received.time_since_epoch()).count();
            sample.position = position_;
            sample.velocity = velocity_;
            sample.torque.assign(position_.size(), 0.0);
            sample.current.assign(position_.size(), 0.0);
            sample.is_ready.assign(position_.size(), ready_);
            sample.control_manager = control_manager_;
            sample.control = stream_open_ ? ControlStatus::kExecuting : ControlStatus::kIdle;
            sample.emo_pressed = emo_;
            cb = callback_;
          }
          if (cb) {
            cb(sample);
          }
          std::this_thread::sleep_until(next);
        }
      });
}

void FakeRobotBackend::stop_state_stream() noexcept
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool FakeRobotBackend::ensure_ready(const ReadyOptions &, std::string & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (control_manager_ == ControlManagerStatus::kMajorFault) {
    message = "control manager has a major fault";
    return false;
  }
  control_manager_ = ControlManagerStatus::kEnabled;
  message = "ready";
  return true;
}

bool FakeRobotBackend::set_parameter(const std::string &, const std::string &)
{
  return true;
}

bool FakeRobotBackend::open_stream(int, std::string & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (control_manager_ != ControlManagerStatus::kEnabled) {
    message = "control manager not enabled";
    return false;
  }
  stream_open_ = true;
  stream_has_structure_ = false;
  ++streams_opened_;
  return true;
}

bool FakeRobotBackend::stream_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stream_open_;
}

StreamFeedback FakeRobotBackend::send(const RobotCommandRequest & request, int)
{
  std::lock_guard<std::mutex> lock(mutex_);
  StreamFeedback fb;
  if (!stream_open_) {
    fb.status = StreamStatus::kFinished;
    fb.message = "stream is not open";
    return fb;
  }
  if (injected_) {
    fb.status = *injected_;
    fb.message = "injected";
    injected_.reset();
    if (fb.status == StreamStatus::kFinished || fb.status == StreamStatus::kError ||
      fb.status == StreamStatus::kFault)
    {
      stream_open_ = false;
    }
    return fb;
  }
  if (control_manager_ != ControlManagerStatus::kEnabled) {
    fb.status = StreamStatus::kFault;
    fb.message = "control manager fault";
    stream_open_ = false;
    return fb;
  }
  // Like the robot: the controller type of each component is fixed when the stream starts.
  std::array<int, kComponentCount> structure{};
  for (size_t i = 0; i < kComponentCount; ++i) {
    structure[i] = static_cast<int>(request.components[i].index());
  }
  if (!stream_has_structure_) {
    stream_structure_ = structure;
    stream_has_structure_ = true;
  } else if (structure != stream_structure_) {
    fb.status = StreamStatus::kFinished;
    fb.message = "command structure changed within a stream";
    stream_open_ = false;
    return fb;
  }
  sent_.push_back(request);
  if (sent_.size() > 2000) {
    sent_.erase(sent_.begin(), sent_.begin() + 1000);
  }
  for (const auto component : kAllComponents) {
    const auto & ids = description_.component_joints[index_of(component)];
    const auto & command = request.at(component);
    auto set_targets = [&](const std::vector<double> & pos, const std::vector<double> & vel) {
        for (size_t k = 0; k < ids.size() && k < pos.size(); ++k) {
          target_[ids[k]] = pos[k];
          velocity_mode_[ids[k]] = false;
          velocity_limit_[ids[k]] = k < vel.size() && vel[k] > 0.0 ? vel[k] : 1.0;
        }
      };
    if (const auto * c = std::get_if<JointPositionCommand>(&command)) {
      set_targets(c->position, c->velocity_limit);
    } else if (const auto * c = std::get_if<JointImpedanceCommand>(&command)) {
      set_targets(c->position, c->velocity_limit);
    } else if (const auto * c = std::get_if<JointGroupPositionCommand>(&command)) {
      for (size_t k = 0; k < c->joint_names.size(); ++k) {
        const auto it = std::find(description_.joint_names.begin(), description_.joint_names.end(), c->joint_names[k]);
        if (it != description_.joint_names.end()) {
          const auto id = static_cast<size_t>(it - description_.joint_names.begin());
          target_[id] = c->position[k];
          velocity_mode_[id] = false;
        }
      }
    } else if (const auto * c = std::get_if<JointVelocityCommand>(&command)) {
      for (size_t k = 0; k < ids.size() && k < c->velocity.size(); ++k) {
        target_velocity_[ids[k]] = c->velocity[k];
        velocity_mode_[ids[k]] = true;
      }
    }
  }
  return fb;
}

void FakeRobotBackend::close_stream() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  stream_open_ = false;
}

bool FakeRobotBackend::cancel_control()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stream_open_ = false;
  return true;
}

bool FakeRobotBackend::start_realtime(RealtimeCallback callback, int, std::string & message)
{
  stop_realtime();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_open_) {
      message = "close the command stream before starting real-time control";
      return false;
    }
  }
  rt_stop_ = false;
  rt_running_ = true;
  rt_thread_ = std::thread([this, cb = std::move(callback)]() {
      const size_t n = description_.joint_names.size();
      RealtimeState state;
      RealtimeInput input;
      input.velocity_mode.assign(n, false);
      input.target.assign(n, 0.0);
      input.feedback_gain.assign(n, 10);
      input.feedforward_torque.assign(n, 0.0);
      auto next = std::chrono::steady_clock::now();
      double t = 0.0;
      while (!rt_stop_) {
        next += std::chrono::milliseconds(2);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          state.t = t;
          state.position = position_;
          state.velocity = velocity_;
          state.torque.assign(n, 0.0);
          state.is_ready.assign(n, true);
        }
        input.target = state.position;
        input.finish = false;
        cb(state, input);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          for (size_t i = 0; i < n; ++i) {
            if (input.velocity_mode[i]) {
              position_[i] += input.target[i] * 0.002;
              velocity_[i] = input.target[i];
            } else {
              velocity_[i] = (input.target[i] - position_[i]) / 0.002;
              position_[i] = input.target[i];
            }
            target_[i] = position_[i];
            velocity_mode_[i] = false;
          }
          ++rt_ticks_;
        }
        if (input.finish) {
          break;
        }
        t += 0.002;
        std::this_thread::sleep_until(next);
      }
      rt_running_ = false;
    });
  return true;
}

void FakeRobotBackend::stop_realtime() noexcept
{
  rt_stop_ = true;
  if (rt_thread_.joinable()) {
    rt_thread_.join();
  }
  rt_running_ = false;
}

bool FakeRobotBackend::realtime_running() const
{
  return rt_running_;
}

void FakeRobotBackend::inject_send_result(StreamStatus status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  injected_ = status;
}

void FakeRobotBackend::set_control_manager(ControlManagerStatus status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  control_manager_ = status;
}

void FakeRobotBackend::set_emo(bool pressed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  emo_ = pressed;
}

void FakeRobotBackend::set_position(size_t robot_index, double position)
{
  std::lock_guard<std::mutex> lock(mutex_);
  position_.at(robot_index) = position;
  target_.at(robot_index) = position;
}

std::vector<RobotCommandRequest> FakeRobotBackend::sent_requests() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sent_;
}

size_t FakeRobotBackend::streams_opened() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return streams_opened_;
}

void FakeRobotBackend::step(double dt)
{
  for (size_t i = 0; i < position_.size(); ++i) {
    double v = 0.0;
    if (velocity_mode_[i]) {
      v = stream_open_ ? target_velocity_[i] : 0.0;
    } else {
      const double error = target_[i] - position_[i];
      v = std::clamp(error / std::max(dt, 0.02), -velocity_limit_[i], velocity_limit_[i]);
    }
    position_[i] += v * dt;
    velocity_[i] = v;
  }
}

}  // namespace rby1_hardware_interface
