#include "rby1_controllers/joint_stream_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"

namespace rby1_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

controller_interface::CallbackReturn JointStreamController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("mode", "impedance");
    auto_declare<std::vector<double>>("stiffness", {100.0});
    auto_declare<double>("damping_ratio", 1.0);
    auto_declare<std::vector<double>>("torque_limit", {});
    auto_declare<double>("command_timeout", 0.5);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration JointStreamController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/position");
    if (impedance_) {
      config.names.push_back(joint + "/stiffness");
      config.names.push_back(joint + "/damping_ratio");
      if (use_torque_limit_) {
        config.names.push_back(joint + "/torque_limit");
      }
    }
  }
  return config;
}

InterfaceConfiguration JointStreamController::state_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/position");
  }
  return config;
}

bool JointStreamController::load_gains(Gains & gains) const
{
  const auto n = joints_.size();
  auto expand = [n](std::vector<double> values) {
      if (values.size() == 1 && n > 1) {
        values.assign(n, values.front());
      }
      return values;
    };
  gains.stiffness = expand(get_node()->get_parameter("stiffness").as_double_array());
  gains.torque_limit = expand(get_node()->get_parameter("torque_limit").as_double_array());
  gains.damping_ratio = get_node()->get_parameter("damping_ratio").as_double();
  if (impedance_ && gains.stiffness.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "'stiffness' needs 1 or %zu values", n);
    return false;
  }
  if (use_torque_limit_ && gains.torque_limit.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "'torque_limit' needs 1 or %zu values", n);
    return false;
  }
  for (const double k : gains.stiffness) {
    if (!std::isfinite(k) || k < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "'stiffness' values must be >= 0");
      return false;
    }
  }
  if (!(gains.damping_ratio > 0.0)) {
    RCLCPP_ERROR(get_node()->get_logger(), "'damping_ratio' must be > 0");
    return false;
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult JointStreamController::on_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool gains_changed = false;
  for (const auto & p : parameters) {
    if (p.get_name() == "joints" || p.get_name() == "mode") {
      result.successful = false;
      result.reason = "'" + p.get_name() + "' can only change while the controller is unconfigured";
      return result;
    }
    gains_changed = gains_changed || p.get_name() == "stiffness" || p.get_name() == "damping_ratio" ||
      p.get_name() == "torque_limit";
  }
  if (gains_changed) {
    // Validate after the parameters are applied (the callback runs before they are set).
    const auto current = *gains_buffer_.readFromNonRT();
    if (!current) {
      return result;  // not configured yet; on_configure reads the parameters
    }
    auto pending = std::make_shared<Gains>(*current);
    const auto n = joints_.size();
    for (const auto & p : parameters) {
      if (p.get_name() == "stiffness") {
        pending->stiffness = p.as_double_array();
        if (pending->stiffness.size() == 1) {pending->stiffness.assign(n, pending->stiffness.front());}
      } else if (p.get_name() == "torque_limit") {
        pending->torque_limit = p.as_double_array();
        if (pending->torque_limit.size() == 1) {pending->torque_limit.assign(n, pending->torque_limit.front());}
      } else if (p.get_name() == "damping_ratio") {
        pending->damping_ratio = p.as_double();
      }
    }
    const bool valid = (!impedance_ || pending->stiffness.size() == n) &&
      (!use_torque_limit_ || pending->torque_limit.size() == n) && pending->damping_ratio > 0.0 &&
      std::all_of(pending->stiffness.begin(), pending->stiffness.end(), [](double k) {return k >= 0.0;});
    if (!valid) {
      result.successful = false;
      result.reason = "invalid gains";
      return result;
    }
    gains_buffer_.writeFromNonRT(pending);
  }
  return result;
}

CallbackReturn JointStreamController::on_configure(const rclcpp_lifecycle::State &)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  const auto mode = get_node()->get_parameter("mode").as_string();
  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' is empty");
    return CallbackReturn::ERROR;
  }
  if (mode != "position" && mode != "impedance") {
    RCLCPP_ERROR(get_node()->get_logger(), "'mode' must be 'position' or 'impedance'");
    return CallbackReturn::ERROR;
  }
  impedance_ = mode == "impedance";
  use_torque_limit_ = impedance_ && !get_node()->get_parameter("torque_limit").as_double_array().empty();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();

  auto gains = std::make_shared<Gains>();
  if (!load_gains(*gains)) {
    return CallbackReturn::ERROR;
  }
  gains_buffer_.writeFromNonRT(gains);
  target_buffer_.writeFromNonRT(nullptr);

  std::unordered_map<std::string, size_t> index;
  for (size_t i = 0; i < joints_.size(); ++i) {
    index[joints_[i]] = i;
  }
  const auto n = joints_.size();
  auto now_ns = [this]() {return get_node()->get_clock()->now().nanoseconds();};

  joint_state_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "~/joint_commands", rclcpp::SystemDefaultsQoS(),
    [this, index, n, now_ns](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (msg->position.size() != msg->name.size()) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "joint_commands: name and position sizes differ; dropped");
        return;
      }
      auto target = std::make_shared<std::vector<double>>(n, std::numeric_limits<double>::quiet_NaN());
      size_t matched = 0;
      for (size_t k = 0; k < msg->name.size(); ++k) {
        const auto it = index.find(msg->name[k]);
        if (it != index.end()) {
          (*target)[it->second] = msg->position[k];
          ++matched;
        }
      }
      if (matched != n) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "joint_commands: %zu of %zu joints matched; dropped", matched, n);
        return;
      }
      target_buffer_.writeFromNonRT(target);
      last_command_ns_ = now_ns();
    });
  array_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this, n, now_ns](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() != n) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "commands: expected %zu values, got %zu; dropped", n, msg->data.size());
        return;
      }
      target_buffer_.writeFromNonRT(std::make_shared<std::vector<double>>(msg->data));
      last_command_ns_ = now_ns();
    });
  parameter_handle_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) {return on_parameters(p);});

  RCLCPP_INFO(get_node()->get_logger(), "%zu joints, %s mode", n, impedance_ ? "impedance" : "position");
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointStreamController::on_activate(const rclcpp_lifecycle::State &)
{
  const auto n = joints_.size();
  position_cmd_.assign(n, -1);
  stiffness_cmd_.assign(n, -1);
  damping_cmd_.assign(n, -1);
  torque_cmd_.assign(n, -1);
  position_state_.assign(n, -1);
  target_.assign(n, std::numeric_limits<double>::quiet_NaN());
  for (size_t i = 0; i < n; ++i) {
    position_cmd_[i] = find_interface(command_interfaces_, joints_[i] + "/position");
    position_state_[i] = find_interface(state_interfaces_, joints_[i] + "/position");
    if (impedance_) {
      stiffness_cmd_[i] = find_interface(command_interfaces_, joints_[i] + "/stiffness");
      damping_cmd_[i] = find_interface(command_interfaces_, joints_[i] + "/damping_ratio");
      if (use_torque_limit_) {
        torque_cmd_[i] = find_interface(command_interfaces_, joints_[i] + "/torque_limit");
      }
    }
    if (position_cmd_[i] < 0 || position_state_[i] < 0 || (impedance_ && (stiffness_cmd_[i] < 0 || damping_cmd_[i] < 0))) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing interfaces for joint '%s'", joints_[i].c_str());
      return CallbackReturn::ERROR;
    }
    target_[i] = read_value(state_interfaces_[position_state_[i]]);
  }
  target_buffer_.writeFromNonRT(nullptr);
  last_command_ns_ = 0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointStreamController::on_deactivate(const rclcpp_lifecycle::State &)
{
  target_buffer_.writeFromNonRT(nullptr);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointStreamController::update(const rclcpp::Time & time, const rclcpp::Duration &)
{
  const auto latest = *target_buffer_.readFromRT();
  if (latest) {
    for (size_t i = 0; i < target_.size(); ++i) {
      if (std::isfinite((*latest)[i])) {
        target_[i] = (*latest)[i];
      }
    }
  }
  if (command_timeout_ > 0.0 && last_command_ns_ != 0 &&
    (time.nanoseconds() - last_command_ns_) * 1e-9 > command_timeout_)
  {
    // The last target is held; the hardware keeps commanding it.
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "No joint command for more than %.2f s; holding the last target", command_timeout_);
  }
  const auto gains = *gains_buffer_.readFromRT();
  for (size_t i = 0; i < target_.size(); ++i) {
    write_value(command_interfaces_[position_cmd_[i]], target_[i]);
    if (impedance_) {
      write_value(command_interfaces_[stiffness_cmd_[i]], gains->stiffness[i]);
      write_value(command_interfaces_[damping_cmd_[i]], gains->damping_ratio);
      if (use_torque_limit_) {
        write_value(command_interfaces_[torque_cmd_[i]], gains->torque_limit[i]);
      }
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace rby1_controllers

PLUGINLIB_EXPORT_CLASS(rby1_controllers::JointStreamController, controller_interface::ControllerInterface)
