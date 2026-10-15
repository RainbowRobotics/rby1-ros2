#pragma once

// Command modes: one class per rby1-sdk controller. A mode decides from the set of
// claimed ros2_control interfaces whether it is the active controller of a component,
// conditions the command values in the real-time loop, and builds the SDK-independent
// component command for the stream thread.
//
// Adding an SDK controller = one CommandMode subclass + one line in make_modes() +
// one translation in SdkRobotBackend. Nothing else in the hardware interface changes.

#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rby1_hardware_interface/command_types.hpp"

namespace rby1_hardware_interface
{

constexpr size_t kNoInterface = std::numeric_limits<size_t>::max();

// Interface names (joint-level; standard names come from hardware_interface).
inline constexpr const char * kStiffness = "stiffness";
inline constexpr const char * kDampingRatio = "damping_ratio";
inline constexpr const char * kTorqueLimit = "torque_limit";
inline constexpr const char * kFeedbackGain = "feedback_gain";

// String parameters with typed getters ("<mode>.<key>" lookups fall back to "<key>").
class ParameterTable
{
public:
  ParameterTable() = default;
  explicit ParameterTable(std::map<std::string, std::string> values);

  bool has(const std::string & key) const;
  std::string get_string(const std::string & key, const std::string & fallback) const;
  double get_double(const std::string & key, double fallback) const;
  bool get_bool(const std::string & key, bool fallback) const;
  std::vector<double> get_doubles(const std::string & key) const;      // comma separated
  std::vector<std::string> get_strings(const std::string & key) const;  // comma separated
  // "<scope>.<key>" if present, else "<key>", else fallback.
  double scoped_double(const std::string & scope, const std::string & key, double fallback) const;
  std::string scoped_string(const std::string & scope, const std::string & key, const std::string & fallback) const;

private:
  std::map<std::string, std::string> values_;
};

struct JointEntry
{
  std::string name;
  Component component{Component::kTorso};
  size_t robot_index{0};
  bool has_position_limits{false};
  double lower{-std::numeric_limits<double>::infinity()};
  double upper{std::numeric_limits<double>::infinity()};
  double velocity_limit{0.0};  // 0: unknown
  double effort_limit{0.0};    // 0: unknown
  // Command interface slots in the command value vector (kNoInterface if not exported).
  size_t cmd_position{kNoInterface};
  size_t cmd_velocity{kNoInterface};
  size_t cmd_effort{kNoInterface};
  size_t cmd_stiffness{kNoInterface};
  size_t cmd_damping_ratio{kNoInterface};
  size_t cmd_torque_limit{kNoInterface};
  size_t cmd_feedback_gain{kNoInterface};
};

struct GpioEntry
{
  std::string name;          // ros2_control gpio name, e.g. "right_arm_cartesian"
  std::string kind;          // "gravity_compensation", "se2_velocity", "cartesian", ...
  Component component{Component::kTorso};
  ParameterTable parameters;
  std::map<std::string, size_t> command_slots;  // interface name (without gpio prefix) -> slot
};

// Read-only view of the latest measured state, in JointEntry order.
struct StateView
{
  const std::vector<double> * position{nullptr};
  const std::vector<double> * velocity{nullptr};
  const std::vector<double> * effort{nullptr};
};

// Everything a mode instance is bound to.
struct ModeBinding
{
  Component component{Component::kTorso};
  const std::vector<JointEntry> * joints{nullptr};
  std::vector<size_t> joint_ids;    // indices into *joints that belong to the component
  const GpioEntry * gpio{nullptr};  // for GPIO-driven modes
  const ParameterTable * hardware_parameters{nullptr};
};

class CommandMode
{
public:
  explicit CommandMode(ModeBinding binding) : binding_(std::move(binding)) {}
  virtual ~CommandMode() = default;

  virtual const char * name() const = 0;
  Component component() const {return binding_.component;}

  // Full interface names ("<joint>/<if>" or "<gpio>/<if>") this mode needs / may also take.
  virtual std::set<std::string> required_interfaces() const = 0;
  virtual std::set<std::string> optional_interfaces() const {return {};}
  // Whether this mode is selected by the interfaces claimed for its component.
  virtual bool matches(const std::set<std::string> & claimed) const;

  // Called when the mode becomes active: seed the command values (e.g. position = measured).
  virtual void on_activate(std::vector<double> & commands, const StateView & state) const;

  // Real-time: copy this mode's command slots from `commands` to `conditioned`, applying
  // guards against the previous conditioned values and the measured state. Returns false
  // while the command is not usable yet (component is then held). No allocation.
  virtual bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView & state, size_t & clamp_count) const = 0;

  // Stream thread: build the component command from conditioned values.
  virtual ComponentCommand build(const std::vector<double> & conditioned) const = 0;

  // Real-time modes are not streamed: the hardware runs the SDK real-time loop instead and
  // reads the conditioned values every control period.
  virtual bool is_realtime() const {return false;}
  const std::vector<size_t> & joint_ids() const {return binding_.joint_ids;}

protected:
  const JointEntry & joint(size_t k) const {return (*binding_.joints)[binding_.joint_ids[k]];}
  size_t joint_count() const {return binding_.joint_ids.size();}
  double param(const std::string & key, double fallback) const;
  std::string param_string(const std::string & key, const std::string & fallback) const;
  CommandTiming timing() const;
  std::vector<double> velocity_limits() const;
  std::vector<double> acceleration_limits() const;
  size_t gpio_slot(const std::string & interface_name) const;
  std::string gpio_interface(const std::string & interface_name) const;

  // Shared joint-position guard: finite, limits, tracking error, step.
  bool condition_joint_positions(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView & state, size_t & clamp_count) const;

  ModeBinding binding_;
};

// Create every mode instance applicable to the bound joints and GPIOs.
std::vector<std::unique_ptr<CommandMode>> make_modes(
  const std::vector<JointEntry> & joints, const std::vector<GpioEntry> & gpios,
  const ParameterTable & hardware_parameters);

// Joint-position hold command for a component (used for unclaimed components).
JointPositionCommand make_hold_command(
  const std::vector<JointEntry> & joints, const std::vector<size_t> & joint_ids,
  const std::vector<double> & hold_position, const ParameterTable & hardware_parameters);

}  // namespace rby1_hardware_interface
