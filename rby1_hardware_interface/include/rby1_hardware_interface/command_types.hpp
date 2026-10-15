#pragma once

// SDK-independent description of one robot command. Command modes produce these from
// ros2_control command interfaces; a RobotBackend translates them into rby1-sdk builders.
// Keeping this layer free of SDK types lets the mode logic be unit-tested without a robot.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace rby1_hardware_interface
{

// Command components, following the rby1-sdk command hierarchy:
// ComponentBased{Mobility, Body{Torso, RightArm, LeftArm} | Body, Head}.
// kBody is a virtual component used by body-level controllers (OptimalControl,
// multi-link Cartesian); it is exclusive with kTorso, kRightArm and kLeftArm.
enum class Component : uint8_t
{
  kTorso = 0,
  kRightArm,
  kLeftArm,
  kHead,
  kMobility,
  kBody,
};
constexpr size_t kComponentCount = 6;
constexpr std::array<Component, kComponentCount> kAllComponents{
  Component::kTorso, Component::kRightArm, Component::kLeftArm,
  Component::kHead, Component::kMobility, Component::kBody};

const char * to_string(Component component);
std::optional<Component> component_from_string(const std::string & name);
constexpr size_t index_of(Component component) {return static_cast<size_t>(component);}

// Common header of every streamed command.
struct CommandTiming
{
  double minimum_time{0.1};
  double control_hold_time{1.0};
};

struct JointPositionCommand
{
  CommandTiming timing;
  std::vector<double> position;
  std::vector<double> velocity_limit;
  std::vector<double> acceleration_limit;
};

// JointGroupPosition (torso only): a subset of the component's joints, by name.
struct JointGroupPositionCommand
{
  CommandTiming timing;
  std::vector<std::string> joint_names;
  std::vector<double> position;
  std::vector<double> velocity_limit;
  std::vector<double> acceleration_limit;
};

struct JointImpedanceCommand
{
  CommandTiming timing;
  std::vector<double> position;
  std::vector<double> velocity_limit;
  std::vector<double> acceleration_limit;
  std::vector<double> stiffness;
  std::vector<double> torque_limit;
  double damping_ratio{1.0};
};

struct GravityCompensationCommand
{
  CommandTiming timing;
  bool on{true};
};

struct JointVelocityCommand
{
  CommandTiming timing;
  std::vector<double> velocity;
  std::vector<double> acceleration_limit;
};

struct SE2VelocityCommand
{
  CommandTiming timing;
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  double linear_acceleration_limit_x{1.0};
  double linear_acceleration_limit_y{1.0};
  double angular_acceleration_limit{1.0};
};

// Pose as position + unit quaternion (x, y, z, w), in ref_link.
struct Pose
{
  std::array<double, 3> position{0.0, 0.0, 0.0};
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
};

struct CartesianTarget
{
  std::string ref_link;
  std::string link;
  Pose pose;
  double linear_velocity_limit{0.5};
  double angular_velocity_limit{1.0};
  double acceleration_limit_scaling{1.0};
  // Only used by CartesianImpedanceControl.
  std::optional<double> linear_acceleration_limit;
  std::optional<double> angular_acceleration_limit;
};

struct JointTarget
{
  std::string joint_name;
  double position{0.0};
  std::optional<double> velocity_limit;
  std::optional<double> acceleration_limit;
  double weight{1.0};  // OptimalControl only
};

struct CartesianCommand
{
  CommandTiming timing;
  std::vector<CartesianTarget> targets;
  std::vector<JointTarget> joint_targets;
  std::optional<double> stop_position_tracking_error;
  std::optional<double> stop_orientation_tracking_error;
  std::optional<double> stop_joint_position_tracking_error;
};

struct JointLimit
{
  std::string joint_name;
  double lower{0.0};
  double upper{0.0};
};

struct NullspaceTarget
{
  std::vector<double> position;
  std::vector<double> weight;
  std::optional<double> k_p;
  std::optional<double> k_d;
  std::optional<double> cost_weight;
};

struct CartesianImpedanceCommand
{
  CommandTiming timing;
  std::vector<CartesianTarget> targets;
  std::vector<JointTarget> joint_targets;
  std::optional<NullspaceTarget> nullspace;
  std::vector<double> joint_stiffness;
  std::optional<double> joint_damping_ratio;
  std::vector<double> joint_torque_limit;
  std::vector<JointLimit> joint_limits;
  std::optional<bool> reset_reference;
  std::optional<double> stop_position_tracking_error;
  std::optional<double> stop_orientation_tracking_error;
  std::optional<double> stop_joint_position_tracking_error;
};

// Cartesian impedance of a single link (SDK ImpedanceControlCommand).
struct ImpedanceCommand
{
  CommandTiming timing;
  std::string ref_link;
  std::string link;
  Pose pose;
  std::array<double, 3> translation_weight{1000.0, 1000.0, 1000.0};
  std::array<double, 3> rotation_weight{50.0, 50.0, 50.0};
  double damping_ratio{1.0};
};

struct OptimalControlTarget
{
  std::string ref_link;
  std::string link;
  Pose pose;
  double translation_weight{1.0};
  double rotation_weight{1.0};
};

struct CenterOfMassTarget
{
  std::string ref_link;
  std::array<double, 3> position{0.0, 0.0, 0.0};
  double weight{1.0};
};

struct OptimalControlCommand
{
  CommandTiming timing;
  std::vector<OptimalControlTarget> targets;
  std::optional<CenterOfMassTarget> center_of_mass;
  std::vector<JointTarget> joint_targets;
  std::optional<double> error_scaling;
  std::optional<double> velocity_limit_scaling;
  std::optional<double> acceleration_limit_scaling;
  std::optional<double> stop_cost;
  std::optional<double> min_delta_cost;
  std::optional<int> patience;
};

using ComponentCommand = std::variant<
  std::monostate,
  JointPositionCommand,
  JointGroupPositionCommand,
  JointImpedanceCommand,
  GravityCompensationCommand,
  JointVelocityCommand,
  SE2VelocityCommand,
  CartesianCommand,
  CartesianImpedanceCommand,
  ImpedanceCommand,
  OptimalControlCommand>;

// One message on the command stream. Components holding std::monostate are omitted.
struct RobotCommandRequest
{
  std::array<ComponentCommand, kComponentCount> components;

  bool empty() const;
  ComponentCommand & at(Component component) {return components[index_of(component)];}
  const ComponentCommand & at(Component component) const {return components[index_of(component)];}
};

// Outcome of one SendCommand on the stream.
enum class StreamStatus : uint8_t
{
  kOk = 0,           // command accepted, stream still running
  kFinished,         // stream finished (hold time elapsed, cancelled or preempted); recreate it
  kFault,            // control manager fault reported; do not retry
  kTimeout,          // no feedback in time
  kError,            // transport or SDK error
};

struct StreamFeedback
{
  StreamStatus status{StreamStatus::kOk};
  int32_t finish_code{0};  // rb::RobotCommandFeedback::FinishCode as int
  std::string message;
};

}  // namespace rby1_hardware_interface
