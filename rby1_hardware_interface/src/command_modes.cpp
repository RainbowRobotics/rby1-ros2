#include "rby1_hardware_interface/command_modes.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rby1_hardware_interface
{

// ---------------------------------------------------------------------------------------------
// Component names

const char * to_string(Component component)
{
  switch (component) {
    case Component::kTorso: return "torso";
    case Component::kRightArm: return "right_arm";
    case Component::kLeftArm: return "left_arm";
    case Component::kHead: return "head";
    case Component::kMobility: return "mobility";
    case Component::kBody: return "body";
  }
  return "unknown";
}

std::optional<Component> component_from_string(const std::string & name)
{
  for (const auto component : kAllComponents) {
    if (name == to_string(component)) {
      return component;
    }
  }
  return std::nullopt;
}

bool RobotCommandRequest::empty() const
{
  return std::all_of(components.begin(), components.end(), [](const ComponentCommand & c) {
             return std::holds_alternative<std::monostate>(c);
           });
}

// ---------------------------------------------------------------------------------------------
// ParameterTable

namespace
{

std::vector<std::string> split(const std::string & text, char separator)
{
  std::vector<std::string> out;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, separator)) {
    const auto begin = item.find_first_not_of(" \t\n");
    const auto end = item.find_last_not_of(" \t\n");
    if (begin != std::string::npos) {
      out.push_back(item.substr(begin, end - begin + 1));
    }
  }
  return out;
}

bool finite(double value) {return std::isfinite(value);}

}  // namespace

ParameterTable::ParameterTable(std::map<std::string, std::string> values)
: values_(std::move(values))
{
}

bool ParameterTable::has(const std::string & key) const
{
  return values_.count(key) != 0;
}

std::string ParameterTable::get_string(const std::string & key, const std::string & fallback) const
{
  const auto it = values_.find(key);
  return it == values_.end() ? fallback : it->second;
}

double ParameterTable::get_double(const std::string & key, double fallback) const
{
  const auto it = values_.find(key);
  if (it == values_.end()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    throw std::invalid_argument("parameter '" + key + "' is not a number: '" + it->second + "'");
  }
}

bool ParameterTable::get_bool(const std::string & key, bool fallback) const
{
  const auto it = values_.find(key);
  if (it == values_.end()) {
    return fallback;
  }
  const auto & v = it->second;
  if (v == "true" || v == "True" || v == "1") {return true;}
  if (v == "false" || v == "False" || v == "0") {return false;}
  throw std::invalid_argument("parameter '" + key + "' is not a boolean: '" + v + "'");
}

std::vector<double> ParameterTable::get_doubles(const std::string & key) const
{
  std::vector<double> out;
  for (const auto & item : get_strings(key)) {
    try {
      out.push_back(std::stod(item));
    } catch (const std::exception &) {
      throw std::invalid_argument("parameter '" + key + "' has a non-number element '" + item + "'");
    }
  }
  return out;
}

std::vector<std::string> ParameterTable::get_strings(const std::string & key) const
{
  const auto it = values_.find(key);
  return it == values_.end() ? std::vector<std::string>{} : split(it->second, ',');
}

double ParameterTable::scoped_double(const std::string & scope, const std::string & key, double fallback) const
{
  const auto scoped = scope + "." + key;
  return has(scoped) ? get_double(scoped, fallback) : get_double(key, fallback);
}

std::string ParameterTable::scoped_string(
  const std::string & scope, const std::string & key, const std::string & fallback) const
{
  const auto scoped = scope + "." + key;
  return has(scoped) ? get_string(scoped, fallback) : get_string(key, fallback);
}

// ---------------------------------------------------------------------------------------------
// CommandMode base

bool CommandMode::matches(const std::set<std::string> & claimed) const
{
  const auto required = required_interfaces();
  if (required.empty()) {
    return false;
  }
  for (const auto & name : required) {
    if (claimed.count(name) == 0) {
      return false;
    }
  }
  const auto optional = optional_interfaces();
  for (const auto & name : claimed) {
    if (required.count(name) == 0 && optional.count(name) == 0) {
      return false;
    }
  }
  return true;
}

void CommandMode::on_activate(std::vector<double> & commands, const StateView & state) const
{
  // Joint modes start at the measured position so that a controller that has not
  // written yet does not move the robot.
  for (size_t k = 0; k < joint_count(); ++k) {
    const auto & j = joint(k);
    if (j.cmd_position != kNoInterface) {
      commands[j.cmd_position] = (*state.position)[binding_.joint_ids[k]];
    }
  }
}

double CommandMode::param(const std::string & key, double fallback) const
{
  return binding_.hardware_parameters->scoped_double(name(), key, fallback);
}

std::string CommandMode::param_string(const std::string & key, const std::string & fallback) const
{
  return binding_.hardware_parameters->scoped_string(name(), key, fallback);
}

CommandTiming CommandMode::timing() const
{
  CommandTiming t;
  t.minimum_time = param("minimum_time", 0.1);
  t.control_hold_time = param("control_hold_time", 1.0);
  return t;
}

std::vector<double> CommandMode::velocity_limits() const
{
  // Explicit "<mode>.velocity_limit" (rad/s, all joints) wins; otherwise URDF limit * scale.
  const double scale = param("velocity_limit_scale", 0.5);
  const bool has_uniform = binding_.hardware_parameters->has(std::string(name()) + ".velocity_limit") ||
    binding_.hardware_parameters->has("velocity_limit");
  const double uniform = param("velocity_limit", 0.5);
  std::vector<double> out;
  out.reserve(joint_count());
  for (size_t k = 0; k < joint_count(); ++k) {
    const double urdf = joint(k).velocity_limit;
    out.push_back(has_uniform || urdf <= 0.0 ? uniform : urdf * scale);
  }
  return out;
}

std::vector<double> CommandMode::acceleration_limits() const
{
  return std::vector<double>(joint_count(), param("acceleration_limit", 2.0));
}

size_t CommandMode::gpio_slot(const std::string & interface_name) const
{
  if (binding_.gpio == nullptr) {
    return kNoInterface;
  }
  const auto it = binding_.gpio->command_slots.find(interface_name);
  return it == binding_.gpio->command_slots.end() ? kNoInterface : it->second;
}

std::string CommandMode::gpio_interface(const std::string & interface_name) const
{
  return binding_.gpio->name + "/" + interface_name;
}

bool CommandMode::condition_joint_positions(
  const std::vector<double> & commands, std::vector<double> & conditioned,
  const StateView & state, size_t & clamp_count) const
{
  const double max_tracking_error = param("max_tracking_error", 0.15);
  const double max_step = param("max_step", 0.05);
  for (size_t k = 0; k < joint_count(); ++k) {
    const auto & j = joint(k);
    const size_t slot = j.cmd_position;
    const double measured = (*state.position)[binding_.joint_ids[k]];
    const double previous = conditioned[slot];
    double target = commands[slot];
    if (!finite(target)) {
      target = finite(previous) ? previous : measured;
      ++clamp_count;
    }
    if (j.has_position_limits) {
      const double clamped = std::clamp(target, j.lower, j.upper);
      clamp_count += clamped != target;
      target = clamped;
    }
    if (max_tracking_error > 0.0 && finite(measured)) {
      const double clamped = std::clamp(target, measured - max_tracking_error, measured + max_tracking_error);
      clamp_count += clamped != target;
      target = clamped;
    }
    if (max_step > 0.0 && finite(previous)) {
      const double clamped = std::clamp(target, previous - max_step, previous + max_step);
      clamp_count += clamped != target;
      target = clamped;
    }
    conditioned[slot] = target;
  }
  return true;
}

namespace
{

// ---------------------------------------------------------------------------------------------
// Joint-space modes

class JointPositionMode : public CommandMode
{
public:
  using CommandMode::CommandMode;
  const char * name() const override {return "joint_position";}

  std::set<std::string> required_interfaces() const override
  {
    std::set<std::string> out;
    for (size_t k = 0; k < joint_count(); ++k) {
      out.insert(joint(k).name + "/position");
    }
    return out;
  }

  bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView & state, size_t & clamp_count) const override
  {
    return condition_joint_positions(commands, conditioned, state, clamp_count);
  }

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    JointPositionCommand c;
    c.timing = timing();
    for (size_t k = 0; k < joint_count(); ++k) {
      c.position.push_back(conditioned[joint(k).cmd_position]);
    }
    c.velocity_limit = velocity_limits();
    c.acceleration_limit = acceleration_limits();
    return c;
  }
};

// Torso only: position of a strict, non-empty subset of the torso joints.
class JointGroupPositionMode : public CommandMode
{
public:
  using CommandMode::CommandMode;
  const char * name() const override {return "joint_group_position";}

  std::set<std::string> required_interfaces() const override {return {};}

  bool matches(const std::set<std::string> & claimed) const override
  {
    if (claimed.empty() || claimed.size() >= joint_count()) {
      return false;
    }
    for (const auto & name : claimed) {
      bool found = false;
      for (size_t k = 0; k < joint_count(); ++k) {
        found = found || name == joint(k).name + "/position";
      }
      if (!found) {
        return false;
      }
    }
    selected_.clear();
    for (size_t k = 0; k < joint_count(); ++k) {
      if (claimed.count(joint(k).name + "/position") != 0) {
        selected_.push_back(k);
      }
    }
    return true;
  }

  bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView & state, size_t & clamp_count) const override
  {
    // Guard all torso joints; the unselected ones keep their seeded (measured) values.
    return condition_joint_positions(commands, conditioned, state, clamp_count);
  }

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    JointGroupPositionCommand c;
    c.timing = timing();
    const auto vel = velocity_limits();
    const auto acc = acceleration_limits();
    for (const auto k : selected_) {
      c.joint_names.push_back(joint(k).name);
      c.position.push_back(conditioned[joint(k).cmd_position]);
      c.velocity_limit.push_back(vel[k]);
      c.acceleration_limit.push_back(acc[k]);
    }
    return c;
  }

private:
  mutable std::vector<size_t> selected_;
};

class JointImpedanceMode : public CommandMode
{
public:
  using CommandMode::CommandMode;
  const char * name() const override {return "joint_impedance";}

  std::set<std::string> required_interfaces() const override
  {
    std::set<std::string> out;
    for (size_t k = 0; k < joint_count(); ++k) {
      const auto & n = joint(k).name;
      out.insert(n + "/position");
      out.insert(n + "/" + kStiffness);
      out.insert(n + "/" + kDampingRatio);
    }
    return out;
  }

  std::set<std::string> optional_interfaces() const override
  {
    std::set<std::string> out;
    for (size_t k = 0; k < joint_count(); ++k) {
      out.insert(joint(k).name + "/" + kTorqueLimit);
    }
    return out;
  }

  void on_activate(std::vector<double> & commands, const StateView & state) const override
  {
    CommandMode::on_activate(commands, state);
    for (size_t k = 0; k < joint_count(); ++k) {
      const auto & j = joint(k);
      commands[j.cmd_stiffness] = std::numeric_limits<double>::quiet_NaN();
      commands[j.cmd_damping_ratio] = std::numeric_limits<double>::quiet_NaN();
      if (j.cmd_torque_limit != kNoInterface) {
        commands[j.cmd_torque_limit] = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView & state, size_t & clamp_count) const override
  {
    condition_joint_positions(commands, conditioned, state, clamp_count);
    const double max_stiffness = param("max_stiffness", 1.0e5);
    for (size_t k = 0; k < joint_count(); ++k) {
      const auto & j = joint(k);
      double stiffness = commands[j.cmd_stiffness];
      if (!finite(stiffness) || stiffness < 0.0) {
        stiffness = default_stiffness(k);
      }
      conditioned[j.cmd_stiffness] = std::min(stiffness, max_stiffness);
      const double damping = commands[j.cmd_damping_ratio];
      conditioned[j.cmd_damping_ratio] = finite(damping) && damping > 0.0 ? damping : param("damping_ratio", 1.0);
      if (j.cmd_torque_limit != kNoInterface) {
        const double limit = commands[j.cmd_torque_limit];
        conditioned[j.cmd_torque_limit] = finite(limit) && limit > 0.0 ? limit : default_torque_limit(k);
      }
    }
    return true;
  }

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    JointImpedanceCommand c;
    c.timing = timing();
    c.velocity_limit = velocity_limits();
    c.acceleration_limit = acceleration_limits();
    double damping_sum = 0.0;
    for (size_t k = 0; k < joint_count(); ++k) {
      const auto & j = joint(k);
      c.position.push_back(conditioned[j.cmd_position]);
      c.stiffness.push_back(conditioned[j.cmd_stiffness]);
      c.torque_limit.push_back(
        j.cmd_torque_limit != kNoInterface ? conditioned[j.cmd_torque_limit] : default_torque_limit(k));
      damping_sum += conditioned[j.cmd_damping_ratio];
    }
    // The SDK takes one damping ratio per command.
    c.damping_ratio = joint_count() > 0 ? damping_sum / static_cast<double>(joint_count()) : 1.0;
    return c;
  }

private:
  double default_stiffness(size_t k) const
  {
    const auto list = binding_.hardware_parameters->get_doubles(
      std::string(name()) + "." + to_string(component()) + ".stiffness");
    if (k < list.size()) {
      return list[k];
    }
    return param("stiffness", 100.0);
  }

  double default_torque_limit(size_t k) const
  {
    const auto list = binding_.hardware_parameters->get_doubles(
      std::string(name()) + "." + to_string(component()) + ".torque_limit");
    if (k < list.size()) {
      return list[k];
    }
    const double urdf = joint(k).effort_limit;
    return param("torque_limit", urdf > 0.0 ? urdf : 30.0);
  }
};

class JointVelocityMode : public CommandMode
{
public:
  using CommandMode::CommandMode;
  const char * name() const override {return "joint_velocity";}

  std::set<std::string> required_interfaces() const override
  {
    std::set<std::string> out;
    for (size_t k = 0; k < joint_count(); ++k) {
      out.insert(joint(k).name + "/velocity");
    }
    return out;
  }

  void on_activate(std::vector<double> & commands, const StateView &) const override
  {
    for (size_t k = 0; k < joint_count(); ++k) {
      commands[joint(k).cmd_velocity] = 0.0;
    }
  }

  bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView &, size_t & clamp_count) const override
  {
    const double max_velocity = param("max_velocity", 0.0);
    for (size_t k = 0; k < joint_count(); ++k) {
      const auto & j = joint(k);
      double v = commands[j.cmd_velocity];
      if (!finite(v)) {
        v = 0.0;
        ++clamp_count;
      }
      const double limit = max_velocity > 0.0 ? max_velocity : j.velocity_limit;
      if (limit > 0.0) {
        const double clamped = std::clamp(v, -limit, limit);
        clamp_count += clamped != v;
        v = clamped;
      }
      conditioned[j.cmd_velocity] = v;
    }
    return true;
  }

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    JointVelocityCommand c;
    c.timing = timing();
    for (size_t k = 0; k < joint_count(); ++k) {
      c.velocity.push_back(conditioned[joint(k).cmd_velocity]);
    }
    c.acceleration_limit = acceleration_limits();
    return c;
  }
};

// ---------------------------------------------------------------------------------------------
// GPIO-driven modes

class GpioMode : public CommandMode
{
public:
  using CommandMode::CommandMode;

protected:
  double gpio_param(const std::string & key, double fallback) const
  {
    return binding_.gpio->parameters.get_double(key, param(key, fallback));
  }

  std::string gpio_param_string(const std::string & key, const std::string & fallback) const
  {
    return binding_.gpio->parameters.get_string(key, fallback);
  }

  CommandTiming gpio_timing() const
  {
    CommandTiming t = timing();
    t.minimum_time = gpio_param("minimum_time", t.minimum_time);
    t.control_hold_time = gpio_param("control_hold_time", t.control_hold_time);
    return t;
  }

  double value(const std::vector<double> & v, const std::string & interface_name) const
  {
    const auto slot = gpio_slot(interface_name);
    return slot == kNoInterface ? std::numeric_limits<double>::quiet_NaN() : v[slot];
  }

  std::optional<double> optional_value(const std::vector<double> & v, const std::string & interface_name) const
  {
    const double x = value(v, interface_name);
    return finite(x) ? std::optional<double>(x) : std::nullopt;
  }

  // All command slots of the gpio are passed through unchanged except NaN -> previous.
  void pass_through(const std::vector<double> & commands, std::vector<double> & conditioned) const
  {
    for (const auto & entry : binding_.gpio->command_slots) {
      const double x = commands[entry.second];
      if (finite(x)) {
        conditioned[entry.second] = x;
      }
    }
  }

  void reset_slots(std::vector<double> & commands) const
  {
    for (const auto & entry : binding_.gpio->command_slots) {
      commands[entry.second] = std::numeric_limits<double>::quiet_NaN();
    }
  }
};

class GravityCompensationMode : public GpioMode
{
public:
  using GpioMode::GpioMode;
  const char * name() const override {return "gravity_compensation";}

  std::set<std::string> required_interfaces() const override {return {gpio_interface("enable")};}

  void on_activate(std::vector<double> & commands, const StateView &) const override
  {
    commands[gpio_slot("enable")] = 1.0;
  }

  bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView &, size_t &) const override
  {
    const double on = commands[gpio_slot("enable")];
    conditioned[gpio_slot("enable")] = finite(on) && on > 0.5 ? 1.0 : 0.0;
    return true;
  }

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    GravityCompensationCommand c;
    c.timing = gpio_timing();
    c.on = conditioned[gpio_slot("enable")] > 0.5;
    return c;
  }
};

class SE2VelocityMode : public GpioMode
{
public:
  using GpioMode::GpioMode;
  const char * name() const override {return "se2_velocity";}

  std::set<std::string> required_interfaces() const override
  {
    return {gpio_interface("linear_x"), gpio_interface("linear_y"), gpio_interface("angular_z")};
  }

  void on_activate(std::vector<double> & commands, const StateView &) const override
  {
    for (const auto * n : {"linear_x", "linear_y", "angular_z"}) {
      commands[gpio_slot(n)] = 0.0;
    }
  }

  bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView &, size_t & clamp_count) const override
  {
    const double max_linear = gpio_param("max_linear_velocity", 1.0);
    const double max_angular = gpio_param("max_angular_velocity", 1.5);
    for (const auto * n : {"linear_x", "linear_y", "angular_z"}) {
      const double limit = std::string(n) == "angular_z" ? max_angular : max_linear;
      double v = commands[gpio_slot(n)];
      if (!finite(v)) {
        v = 0.0;
        ++clamp_count;
      }
      const double clamped = std::clamp(v, -limit, limit);
      clamp_count += clamped != v;
      conditioned[gpio_slot(n)] = clamped;
    }
    return true;
  }

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    SE2VelocityCommand c;
    c.timing = gpio_timing();
    c.linear_x = conditioned[gpio_slot("linear_x")];
    c.linear_y = conditioned[gpio_slot("linear_y")];
    c.angular_z = conditioned[gpio_slot("angular_z")];
    const double linear_acc = gpio_param("linear_acceleration_limit", 1.0);
    c.linear_acceleration_limit_x = linear_acc;
    c.linear_acceleration_limit_y = linear_acc;
    c.angular_acceleration_limit = gpio_param("angular_acceleration_limit", 1.0);
    return c;
  }
};

// Shared parsing of Cartesian target slots: gpio parameter "targets" lists
// "ref_link:link" pairs; slot i exposes t<i>.{x,y,z,qx,qy,qz,qw} (+ optional extras).
class CartesianGpioMode : public GpioMode
{
public:
  CartesianGpioMode(ModeBinding binding, std::vector<std::string> extra_target_fields)
  : GpioMode(std::move(binding)), extra_target_fields_(std::move(extra_target_fields))
  {
    for (const auto & pair : binding_.gpio->parameters.get_strings("targets")) {
      const auto colon = pair.find(':');
      if (colon == std::string::npos) {
        throw std::invalid_argument(
                "gpio '" + binding_.gpio->name + "': target '" + pair + "' must be 'ref_link:link'");
      }
      links_.emplace_back(pair.substr(0, colon), pair.substr(colon + 1));
    }
    joint_targets_ = binding_.gpio->parameters.get_strings("joint_targets");
  }

  std::set<std::string> required_interfaces() const override
  {
    std::set<std::string> out;
    for (size_t i = 0; i < links_.size(); ++i) {
      for (const auto * f : {"x", "y", "z", "qx", "qy", "qz", "qw"}) {
        out.insert(gpio_interface(field(i, f)));
      }
    }
    for (size_t k = 0; k < joint_targets_.size(); ++k) {
      out.insert(gpio_interface("j" + std::to_string(k) + ".position"));
    }
    return out;
  }

  std::set<std::string> optional_interfaces() const override
  {
    std::set<std::string> out;
    for (const auto & entry : binding_.gpio->command_slots) {
      out.insert(gpio_interface(entry.first));
    }
    return out;
  }

  void on_activate(std::vector<double> & commands, const StateView &) const override
  {
    reset_slots(commands);  // targets must be written by the controller before anything is sent
  }

  bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView &, size_t & clamp_count) const override
  {
    const double max_linear_step = gpio_param("max_linear_step", 0.05);
    bool ready = true;
    for (size_t i = 0; i < links_.size(); ++i) {
      double p[3];
      double q[4];
      bool valid = true;
      const char * pf[3] = {"x", "y", "z"};
      const char * qf[4] = {"qx", "qy", "qz", "qw"};
      for (int a = 0; a < 3; ++a) {
        p[a] = commands[gpio_slot(field(i, pf[a]))];
        valid = valid && finite(p[a]);
      }
      double norm = 0.0;
      for (int a = 0; a < 4; ++a) {
        q[a] = commands[gpio_slot(field(i, qf[a]))];
        valid = valid && finite(q[a]);
        norm += q[a] * q[a];
      }
      norm = std::sqrt(norm);
      if (!valid || norm < 1e-6) {
        // keep the previous conditioned target if there is one
        ready = ready && finite(conditioned[gpio_slot(field(i, "x"))]);
        ++clamp_count;
        continue;
      }
      for (int a = 0; a < 3; ++a) {
        const size_t slot = gpio_slot(field(i, pf[a]));
        const double previous = conditioned[slot];
        double v = p[a];
        if (max_linear_step > 0.0 && finite(previous)) {
          const double clamped = std::clamp(v, previous - max_linear_step, previous + max_linear_step);
          clamp_count += clamped != v;
          v = clamped;
        }
        conditioned[slot] = v;
      }
      for (int a = 0; a < 4; ++a) {
        conditioned[gpio_slot(field(i, qf[a]))] = q[a] / norm;
      }
      for (const auto & extra : extra_target_fields_) {
        const auto slot = gpio_slot(field(i, extra.c_str()));
        if (slot != kNoInterface && finite(commands[slot])) {
          conditioned[slot] = commands[slot];
        }
      }
    }
    for (size_t k = 0; k < joint_targets_.size(); ++k) {
      const auto prefix = "j" + std::to_string(k) + ".";
      for (const auto * f : {"position", "velocity_limit", "acceleration_limit", "weight"}) {
        const auto slot = gpio_slot(prefix + f);
        if (slot == kNoInterface) {
          continue;
        }
        if (finite(commands[slot])) {
          conditioned[slot] = commands[slot];
        } else if (std::string(f) == "position" && !finite(conditioned[slot])) {
          ready = false;
        }
      }
    }
    return ready;
  }

protected:
  static std::string field(size_t i, const char * f) {return "t" + std::to_string(i) + "." + f;}

  Pose pose(const std::vector<double> & v, size_t i) const
  {
    Pose out;
    out.position = {value(v, field(i, "x")), value(v, field(i, "y")), value(v, field(i, "z"))};
    out.orientation = {value(v, field(i, "qx")), value(v, field(i, "qy")), value(v, field(i, "qz")),
      value(v, field(i, "qw"))};
    return out;
  }

  CartesianTarget target(const std::vector<double> & v, size_t i) const
  {
    CartesianTarget t;
    t.ref_link = links_[i].first;
    t.link = links_[i].second;
    t.pose = pose(v, i);
    t.linear_velocity_limit = optional_value(v, field(i, "linear_velocity_limit"))
      .value_or(gpio_param("linear_velocity_limit", 0.3));
    t.angular_velocity_limit = optional_value(v, field(i, "angular_velocity_limit"))
      .value_or(gpio_param("angular_velocity_limit", 0.8));
    t.acceleration_limit_scaling = optional_value(v, field(i, "acceleration_limit_scaling"))
      .value_or(gpio_param("acceleration_limit_scaling", 1.0));
    return t;
  }

  std::vector<JointTarget> joint_targets(const std::vector<double> & v) const
  {
    std::vector<JointTarget> out;
    for (size_t k = 0; k < joint_targets_.size(); ++k) {
      const auto prefix = "j" + std::to_string(k) + ".";
      JointTarget j;
      j.joint_name = joint_targets_[k];
      j.position = value(v, prefix + "position");
      j.velocity_limit = optional_value(v, prefix + "velocity_limit");
      j.acceleration_limit = optional_value(v, prefix + "acceleration_limit");
      j.weight = optional_value(v, prefix + "weight").value_or(1.0);
      out.push_back(j);
    }
    return out;
  }

  std::optional<double> optional_param(const std::string & key) const
  {
    if (binding_.gpio->parameters.has(key)) {
      return binding_.gpio->parameters.get_double(key, 0.0);
    }
    return std::nullopt;
  }

  std::vector<std::pair<std::string, std::string>> links_;
  std::vector<std::string> joint_targets_;
  std::vector<std::string> extra_target_fields_;
};

class CartesianMode : public CartesianGpioMode
{
public:
  explicit CartesianMode(ModeBinding binding)
  : CartesianGpioMode(std::move(binding), {"linear_velocity_limit", "angular_velocity_limit", "acceleration_limit_scaling"})
  {
  }
  const char * name() const override {return "cartesian";}

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    CartesianCommand c;
    c.timing = gpio_timing();
    for (size_t i = 0; i < links_.size(); ++i) {
      c.targets.push_back(target(conditioned, i));
    }
    c.joint_targets = joint_targets(conditioned);
    c.stop_position_tracking_error = optional_param("stop_position_tracking_error");
    c.stop_orientation_tracking_error = optional_param("stop_orientation_tracking_error");
    c.stop_joint_position_tracking_error = optional_param("stop_joint_position_tracking_error");
    return c;
  }
};

class CartesianImpedanceMode : public CartesianGpioMode
{
public:
  explicit CartesianImpedanceMode(ModeBinding binding)
  : CartesianGpioMode(
      std::move(binding),
      {"linear_velocity_limit", "angular_velocity_limit", "linear_acceleration_limit", "angular_acceleration_limit"})
  {
  }
  const char * name() const override {return "cartesian_impedance";}

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    const auto & p = binding_.gpio->parameters;
    CartesianImpedanceCommand c;
    c.timing = gpio_timing();
    for (size_t i = 0; i < links_.size(); ++i) {
      auto t = target(conditioned, i);
      t.linear_acceleration_limit = optional_value(conditioned, field(i, "linear_acceleration_limit"));
      t.angular_acceleration_limit = optional_value(conditioned, field(i, "angular_acceleration_limit"));
      c.targets.push_back(t);
    }
    c.joint_targets = joint_targets(conditioned);
    c.joint_stiffness = p.get_doubles("joint_stiffness");
    c.joint_damping_ratio = optional_param("joint_damping_ratio");
    c.joint_torque_limit = p.get_doubles("joint_torque_limit");
    for (const auto & item : p.get_strings("joint_limits")) {
      // "joint:lower:upper"
      const auto a = item.find(':');
      const auto b = item.rfind(':');
      if (a == std::string::npos || a == b) {
        throw std::invalid_argument("joint_limits entry '" + item + "' must be 'joint:lower:upper'");
      }
      c.joint_limits.push_back({item.substr(0, a), std::stod(item.substr(a + 1, b - a - 1)), std::stod(item.substr(b + 1))});
    }
    if (p.has("nullspace_position")) {
      NullspaceTarget n;
      n.position = p.get_doubles("nullspace_position");
      n.weight = p.get_doubles("nullspace_weight");
      if (n.weight.empty()) {
        n.weight.assign(n.position.size(), 1.0);
      }
      n.k_p = optional_param("nullspace_k_p");
      n.k_d = optional_param("nullspace_k_d");
      n.cost_weight = optional_param("nullspace_cost_weight");
      c.nullspace = n;
    }
    if (p.has("reset_reference")) {
      c.reset_reference = p.get_bool("reset_reference", false);
    }
    c.stop_position_tracking_error = optional_param("stop_position_tracking_error");
    c.stop_orientation_tracking_error = optional_param("stop_orientation_tracking_error");
    c.stop_joint_position_tracking_error = optional_param("stop_joint_position_tracking_error");
    return c;
  }
};

class ImpedanceMode : public CartesianGpioMode
{
public:
  explicit ImpedanceMode(ModeBinding binding)
  : CartesianGpioMode(std::move(binding), {})
  {
    if (links_.size() != 1 || !joint_targets_.empty()) {
      throw std::invalid_argument(
              "gpio '" + binding_.gpio->name + "' (impedance) needs exactly one target and no joint targets");
    }
  }
  const char * name() const override {return "impedance";}

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    const auto & p = binding_.gpio->parameters;
    ImpedanceCommand c;
    c.timing = gpio_timing();
    c.ref_link = links_[0].first;
    c.link = links_[0].second;
    c.pose = pose(conditioned, 0);
    const auto tw = p.get_doubles("translation_weight");
    const auto rw = p.get_doubles("rotation_weight");
    if (tw.size() == 3) {c.translation_weight = {tw[0], tw[1], tw[2]};}
    if (rw.size() == 3) {c.rotation_weight = {rw[0], rw[1], rw[2]};}
    c.damping_ratio = p.get_double("damping_ratio", 1.0);
    return c;
  }
};

class OptimalControlMode : public CartesianGpioMode
{
public:
  explicit OptimalControlMode(ModeBinding binding)
  : CartesianGpioMode(std::move(binding), {"translation_weight", "rotation_weight"})
  {
    com_ref_link_ = binding_.gpio->parameters.get_string("com_ref_link", "");
  }
  const char * name() const override {return "optimal_control";}

  std::set<std::string> required_interfaces() const override
  {
    auto out = CartesianGpioMode::required_interfaces();
    if (!com_ref_link_.empty()) {
      for (const auto * f : {"com.x", "com.y", "com.z"}) {
        out.insert(gpio_interface(f));
      }
    }
    return out;
  }

  bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView & state, size_t & clamp_count) const override
  {
    bool ready = CartesianGpioMode::condition(commands, conditioned, state, clamp_count);
    if (!com_ref_link_.empty()) {
      for (const auto * f : {"com.x", "com.y", "com.z", "com.weight"}) {
        const auto slot = gpio_slot(f);
        if (slot == kNoInterface) {
          continue;
        }
        if (finite(commands[slot])) {
          conditioned[slot] = commands[slot];
        } else if (std::string(f) != "com.weight" && !finite(conditioned[slot])) {
          ready = false;
        }
      }
    }
    return ready;
  }

  ComponentCommand build(const std::vector<double> & conditioned) const override
  {
    OptimalControlCommand c;
    c.timing = gpio_timing();
    for (size_t i = 0; i < links_.size(); ++i) {
      OptimalControlTarget t;
      t.ref_link = links_[i].first;
      t.link = links_[i].second;
      t.pose = pose(conditioned, i);
      t.translation_weight = optional_value(conditioned, field(i, "translation_weight"))
        .value_or(gpio_param("translation_weight", 1.0));
      t.rotation_weight = optional_value(conditioned, field(i, "rotation_weight"))
        .value_or(gpio_param("rotation_weight", 1.0));
      c.targets.push_back(t);
    }
    if (!com_ref_link_.empty()) {
      CenterOfMassTarget com;
      com.ref_link = com_ref_link_;
      com.position = {value(conditioned, "com.x"), value(conditioned, "com.y"), value(conditioned, "com.z")};
      com.weight = optional_value(conditioned, "com.weight").value_or(gpio_param("com_weight", 1.0));
      c.center_of_mass = com;
    }
    c.joint_targets = joint_targets(conditioned);
    c.error_scaling = optional_param("error_scaling");
    c.velocity_limit_scaling = optional_param("velocity_limit_scaling");
    c.acceleration_limit_scaling = optional_param("acceleration_limit_scaling");
    c.stop_cost = optional_param("stop_cost");
    c.min_delta_cost = optional_param("min_delta_cost");
    if (binding_.gpio->parameters.has("patience")) {
      c.patience = static_cast<int>(binding_.gpio->parameters.get_double("patience", 10));
    }
    return c;
  }

private:
  std::string com_ref_link_;
};

// Whole-body real-time control (SDK RealTimeControlCommand / Robot::Control): position
// target, feedforward torque and feedback gain (0-10) per joint, sent every 2 ms by the
// hardware's real-time loop. Claims every torso and arm joint (head optional).
class RealtimeMode : public CommandMode
{
public:
  using CommandMode::CommandMode;
  const char * name() const override {return "realtime";}
  bool is_realtime() const override {return true;}

  std::set<std::string> required_interfaces() const override
  {
    std::set<std::string> out;
    for (size_t k = 0; k < joint_count(); ++k) {
      if (joint(k).component != Component::kHead) {
        add(out, joint(k).name);
      }
    }
    return out;
  }

  std::set<std::string> optional_interfaces() const override
  {
    std::set<std::string> out;
    for (size_t k = 0; k < joint_count(); ++k) {
      if (joint(k).component == Component::kHead) {
        add(out, joint(k).name);
      }
    }
    return out;
  }

  void on_activate(std::vector<double> & commands, const StateView & state) const override
  {
    CommandMode::on_activate(commands, state);
    for (size_t k = 0; k < joint_count(); ++k) {
      commands[joint(k).cmd_effort] = 0.0;
      commands[joint(k).cmd_feedback_gain] = 10.0;
    }
  }

  bool condition(
    const std::vector<double> & commands, std::vector<double> & conditioned,
    const StateView & state, size_t & clamp_count) const override
  {
    condition_joint_positions(commands, conditioned, state, clamp_count);
    const double max_ff_scale = param("max_feedforward_scale", 1.0);
    for (size_t k = 0; k < joint_count(); ++k) {
      const auto & j = joint(k);
      double ff = commands[j.cmd_effort];
      if (!finite(ff)) {
        ff = 0.0;
      }
      const double limit = (j.effort_limit > 0.0 ? j.effort_limit : 50.0) * max_ff_scale;
      const double clamped = std::clamp(ff, -limit, limit);
      clamp_count += clamped != ff;
      conditioned[j.cmd_effort] = clamped;
      const double gain = commands[j.cmd_feedback_gain];
      conditioned[j.cmd_feedback_gain] = finite(gain) ? std::clamp(std::round(gain), 0.0, 10.0) : 10.0;
    }
    return true;
  }

  ComponentCommand build(const std::vector<double> &) const override {return std::monostate{};}

private:
  static void add(std::set<std::string> & out, const std::string & joint)
  {
    out.insert(joint + "/position");
    out.insert(joint + "/" + kEffortCommand);
    out.insert(joint + "/" + kFeedbackGain);
  }
  static constexpr const char * kEffortCommand = "effort";
};

bool has_all(const std::vector<JointEntry> & joints, const std::vector<size_t> & ids, size_t JointEntry::* slot)
{
  return !ids.empty() && std::all_of(ids.begin(), ids.end(), [&](size_t id) {
             return joints[id].*slot != kNoInterface;
           });
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Registry

std::vector<std::unique_ptr<CommandMode>> make_modes(
  const std::vector<JointEntry> & joints, const std::vector<GpioEntry> & gpios,
  const ParameterTable & hardware_parameters)
{
  std::vector<std::unique_ptr<CommandMode>> modes;

  auto binding_for = [&](Component component, const GpioEntry * gpio) {
      ModeBinding b;
      b.component = component;
      b.joints = &joints;
      b.gpio = gpio;
      b.hardware_parameters = &hardware_parameters;
      for (size_t id = 0; id < joints.size(); ++id) {
        if (joints[id].component == component) {
          b.joint_ids.push_back(id);
        }
      }
      return b;
    };

  // Joint-space modes: created when every joint of the component exports the interfaces.
  for (const auto component : {Component::kTorso, Component::kRightArm, Component::kLeftArm, Component::kHead}) {
    auto b = binding_for(component, nullptr);
    if (!has_all(joints, b.joint_ids, &JointEntry::cmd_position)) {
      continue;
    }
    modes.push_back(std::make_unique<JointPositionMode>(b));
    if (component == Component::kTorso) {
      modes.push_back(std::make_unique<JointGroupPositionMode>(b));
    }
    if (component != Component::kHead && has_all(joints, b.joint_ids, &JointEntry::cmd_stiffness) &&
      has_all(joints, b.joint_ids, &JointEntry::cmd_damping_ratio))
    {
      modes.push_back(std::make_unique<JointImpedanceMode>(b));
    }
  }
  {
    auto b = binding_for(Component::kMobility, nullptr);
    if (has_all(joints, b.joint_ids, &JointEntry::cmd_velocity)) {
      modes.push_back(std::make_unique<JointVelocityMode>(b));
    }
  }
  {
    // Real-time control spans torso, arms and head: bound to the virtual body component.
    ModeBinding b = binding_for(Component::kBody, nullptr);
    for (size_t id = 0; id < joints.size(); ++id) {
      const auto c = joints[id].component;
      if (c == Component::kTorso || c == Component::kRightArm || c == Component::kLeftArm || c == Component::kHead) {
        b.joint_ids.push_back(id);
      }
    }
    const bool all = !b.joint_ids.empty() && std::all_of(b.joint_ids.begin(), b.joint_ids.end(), [&](size_t id) {
          return joints[id].cmd_position != kNoInterface && joints[id].cmd_effort != kNoInterface &&
          joints[id].cmd_feedback_gain != kNoInterface;
        });
    if (all) {
      modes.push_back(std::make_unique<RealtimeMode>(b));
    }
  }

  // GPIO-driven modes: one per declared gpio block, by its "kind".
  for (const auto & gpio : gpios) {
    auto b = binding_for(gpio.component, &gpio);
    const auto allowed = [&](std::initializer_list<Component> components) {
        if (std::find(components.begin(), components.end(), gpio.component) == components.end()) {
          throw std::invalid_argument(
                  "gpio '" + gpio.name + "': kind '" + gpio.kind + "' is not supported on component '" +
                  to_string(gpio.component) + "'");
        }
      };
    if (gpio.kind == "gravity_compensation") {
      allowed({Component::kTorso, Component::kRightArm, Component::kLeftArm, Component::kBody});
      modes.push_back(std::make_unique<GravityCompensationMode>(b));
    } else if (gpio.kind == "se2_velocity") {
      allowed({Component::kMobility});
      modes.push_back(std::make_unique<SE2VelocityMode>(b));
    } else if (gpio.kind == "cartesian") {
      allowed({Component::kTorso, Component::kRightArm, Component::kLeftArm, Component::kBody});
      modes.push_back(std::make_unique<CartesianMode>(b));
    } else if (gpio.kind == "cartesian_impedance") {
      allowed({Component::kTorso, Component::kRightArm, Component::kLeftArm, Component::kBody});
      modes.push_back(std::make_unique<CartesianImpedanceMode>(b));
    } else if (gpio.kind == "impedance") {
      allowed({Component::kTorso, Component::kRightArm, Component::kLeftArm});
      modes.push_back(std::make_unique<ImpedanceMode>(b));
    } else if (gpio.kind == "optimal_control") {
      allowed({Component::kTorso, Component::kBody});
      modes.push_back(std::make_unique<OptimalControlMode>(b));
    } else if (gpio.kind == "status") {
      // state-only gpio, no mode
    } else {
      throw std::invalid_argument("gpio '" + gpio.name + "': unknown kind '" + gpio.kind + "'");
    }
    // Every required interface must exist.
    if (!modes.empty() && modes.back()->component() == gpio.component && gpio.kind != "status") {
      for (const auto & name : modes.back()->required_interfaces()) {
        const auto local = name.substr(gpio.name.size() + 1);
        if (gpio.command_slots.count(local) == 0) {
          throw std::invalid_argument("gpio '" + gpio.name + "' is missing command interface '" + local + "'");
        }
      }
    }
  }
  return modes;
}

JointPositionCommand make_hold_command(
  const std::vector<JointEntry> & joints, const std::vector<size_t> & joint_ids,
  const std::vector<double> & hold_position, const ParameterTable & hardware_parameters)
{
  JointPositionCommand c;
  c.timing.minimum_time = hardware_parameters.scoped_double("hold", "minimum_time", 0.1);
  c.timing.control_hold_time = hardware_parameters.scoped_double("hold", "control_hold_time", 1.0);
  const double velocity = hardware_parameters.scoped_double("hold", "velocity_limit", 0.5);
  const double acceleration = hardware_parameters.scoped_double("hold", "acceleration_limit", 2.0);
  for (const auto id : joint_ids) {
    c.position.push_back(hold_position[id]);
    c.velocity_limit.push_back(velocity);
    c.acceleration_limit.push_back(acceleration);
    (void)joints;
  }
  return c;
}

}  // namespace rby1_hardware_interface
