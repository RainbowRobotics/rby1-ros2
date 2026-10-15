#pragma once

// Abstract connection to one RB-Y1. The ros2_control system interface only talks to this
// interface; SdkRobotBackend implements it with rby1-sdk, FakeRobotBackend in memory.

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rby1_hardware_interface/command_types.hpp"

namespace rby1_hardware_interface
{

enum class ControlManagerStatus : int32_t
{
  kUnknown = 0,
  kIdle = 1,
  kEnabled = 2,
  kMinorFault = 3,
  kMajorFault = 4,
};

enum class ControlStatus : int32_t
{
  kUnknown = 0,
  kIdle = 1,
  kExecuting = 2,
  kSwitching = 3,
};

struct RobotDescription
{
  std::string model;            // "a", "m" or "ub"
  std::string model_name;       // as reported by the robot
  std::string model_version;
  std::string robot_version;
  std::string sdk_version;
  std::vector<std::string> joint_names;   // robot order
  std::vector<bool> has_brake;
  // Robot-order joint indices per component (kBody: torso + arms, kMobility: wheels).
  std::array<std::vector<size_t>, kComponentCount> component_joints;
};

// One state sample in robot joint order.
struct StateSample
{
  std::chrono::steady_clock::time_point received{};
  double robot_time{0.0};  // robot timestamp [s]
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> torque;
  std::vector<double> current;
  std::vector<bool> is_ready;
  ControlManagerStatus control_manager{ControlManagerStatus::kUnknown};
  ControlStatus control{ControlStatus::kUnknown};
  double time_scale{1.0};
  bool emo_pressed{false};
  std::array<double, 3> odometry{0.0, 0.0, 0.0};  // x, y, yaw
  std::array<double, 6> ft_right{};               // force xyz, torque xyz
  std::array<double, 6> ft_left{};
};

struct ReadyOptions
{
  bool auto_enable{false};          // power on, servo on and enable the control manager if needed
  std::string power_selector{".*"};
  std::string servo_selector{".*"};
  bool reset_minor_fault{true};
  int control_ready_timeout_ms{2000};
};

class RobotBackend
{
public:
  using StateCallback = std::function<void(const StateSample &)>;

  virtual ~RobotBackend() = default;

  // Connect and read the robot description. Throws std::runtime_error on failure.
  virtual RobotDescription connect(double timeout_sec) = 0;
  virtual void disconnect() noexcept = 0;
  virtual bool is_connected() const = 0;

  // Start the state stream; the callback runs on a backend thread.
  virtual void start_state_stream(double rate_hz, StateCallback callback) = 0;
  virtual void stop_state_stream() noexcept = 0;

  // Make power, servo and control manager ready, or report why they are not.
  virtual bool ensure_ready(const ReadyOptions & options, std::string & message) = 0;

  // Robot parameter (never written to the robot's database).
  virtual bool set_parameter(const std::string & name, const std::string & value) = 0;

  // Command stream. send() blocks until the robot answers or timeout_ms elapses.
  virtual bool open_stream(int priority, std::string & message) = 0;
  virtual bool stream_open() const = 0;
  virtual StreamFeedback send(const RobotCommandRequest & request, int timeout_ms) = 0;
  virtual void close_stream() noexcept = 0;

  // Cancel any running control on the robot.
  virtual bool cancel_control() = 0;
};

std::unique_ptr<RobotBackend> make_sdk_backend(const std::string & model, const std::string & address);

}  // namespace rby1_hardware_interface
