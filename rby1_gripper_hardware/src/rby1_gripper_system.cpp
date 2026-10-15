#include "rby1_gripper_hardware/rby1_gripper_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rby1_gripper_hardware
{
namespace
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

std::string param(const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

bool truthy(const std::string & value)
{
  return value == "true" || value == "True" || value == "1";
}

}  // namespace

Rby1GripperSystem::~Rby1GripperSystem()
{
  run_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

#ifdef RBY1_GRIPPER_PARAMS_INIT
CallbackReturn Rby1GripperSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return initialize(params.hardware_info);
}
#else
CallbackReturn Rby1GripperSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return initialize(info);
}
#endif

CallbackReturn Rby1GripperSystem::initialize(const hardware_interface::HardwareInfo & info)
{
  logger_ = rclcpp::get_logger("rby1_gripper_system." + info.name);
  try {
    backend_ = param(info, "backend", "rb_dynamixel");
    device_ = param(info, "device", "/dev/rby1_gripper");
    address_ = param(info, "robot_ip", "192.168.30.1:50051");
    model_ = param(info, "model", "a");
    tool_flange_voltage_ = std::stoi(param(info, "tool_flange_voltage", "12"));
    calibrate_ = truthy(param(info, "calibrate", "true"));
    calibration_current_ = std::stod(param(info, "calibration_current", "0.5"));
    calibration_time_ = std::stod(param(info, "calibration_time", "3.0"));
    min_stroke_ = std::stod(param(info, "min_stroke", "3.0"));
    current_limit_ = std::stod(param(info, "current_limit", "0.5"));
    open_position_ = std::stod(param(info, "open_position", "-0.05"));
    closed_position_ = std::stod(param(info, "closed_position", "0.0"));
    closed_at_min_ = param(info, "closed_at", "min") == "min";
    rate_ = std::stod(param(info, "rate", "100"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Invalid gripper parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (backend_ == "robotiq_flange") {
    RCLCPP_ERROR(
      logger_, "backend robotiq_flange needs the robot's GripperCommandService, which rby1-sdk does not "
      "expose yet (see docs/SDK_COVERAGE.md)");
    return CallbackReturn::ERROR;
  }
  if (backend_ != "rb_dynamixel" && backend_ != "fake") {
    RCLCPP_ERROR(logger_, "backend must be rb_dynamixel, robotiq_flange or fake");
    return CallbackReturn::ERROR;
  }

  const int right_id = std::stoi(param(info, "right_id", "0"));
  const int left_id = std::stoi(param(info, "left_id", "1"));
  fingers_.clear();
  motor_ids_.clear();
  motor_finger_.clear();
  for (const auto & joint : info.joints) {
    Finger f;
    f.name = joint.name;
    fingers_.push_back(f);
  }
  for (size_t i = 0; i < info.joints.size(); ++i) {
    const auto & joint = info.joints[i];
    const auto mirror = joint.parameters.find("mirror_of");
    if (mirror != joint.parameters.end()) {
      const auto it = std::find_if(fingers_.begin(), fingers_.end(), [&](const Finger & f) {return f.name == mirror->second;});
      if (it == fingers_.end()) {
        RCLCPP_ERROR(logger_, "Joint '%s' mirrors unknown joint '%s'", joint.name.c_str(), mirror->second.c_str());
        return CallbackReturn::ERROR;
      }
      fingers_[i].mirror = static_cast<int>(it - fingers_.begin());
      continue;
    }
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(logger_, "Gripper joint '%s' needs exactly one position command interface", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    const auto motor_param = joint.parameters.find("motor_id");
    int id = -1;
    if (motor_param != joint.parameters.end()) {
      id = std::stoi(motor_param->second);
    } else if (joint.name.find("_r") != std::string::npos) {
      id = right_id;
    } else if (joint.name.find("_l") != std::string::npos) {
      id = left_id;
    } else {
      RCLCPP_ERROR(logger_, "Cannot tell the motor of '%s'; set its motor_id parameter", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    fingers_[i].motor = static_cast<int>(motor_ids_.size());
    motor_ids_.push_back(id);
    motor_finger_.push_back(i);
  }
  samples_.assign(motor_ids_.size(), MotorSample{});
  goals_.assign(motor_ids_.size(), std::numeric_limits<double>::quiet_NaN());
  stroke_min_.assign(motor_ids_.size(), 0.0);
  stroke_max_.assign(motor_ids_.size(), 0.0);
  RCLCPP_INFO(logger_, "%zu grippers on %s (%s)", motor_ids_.size(), device_.c_str(), backend_.c_str());
  return CallbackReturn::SUCCESS;
}

double Rby1GripperSystem::to_finger(size_t motor, double motor_angle) const
{
  const double span = stroke_max_[motor] - stroke_min_[motor];
  if (span <= 0.0) {
    return closed_position_;
  }
  double s = std::clamp((motor_angle - stroke_min_[motor]) / span, 0.0, 1.0);  // 0 at min
  if (!closed_at_min_) {
    s = 1.0 - s;
  }
  // s = 0: closed, s = 1: open
  return closed_position_ + s * (open_position_ - closed_position_);
}

double Rby1GripperSystem::to_motor(size_t motor, double finger_position) const
{
  const double range = open_position_ - closed_position_;
  double s = range != 0.0 ? std::clamp((finger_position - closed_position_) / range, 0.0, 1.0) : 0.0;
  if (!closed_at_min_) {
    s = 1.0 - s;
  }
  return stroke_min_[motor] + s * (stroke_max_[motor] - stroke_min_[motor]);
}

CallbackReturn Rby1GripperSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    if (!bus_) {
      bus_ = backend_ == "fake" ? make_fake_bus() : make_dynamixel_bus(device_);
    }
    if (backend_ == "rb_dynamixel" && tool_flange_voltage_ > 0) {
      std::string error;
      if (!set_tool_flange_voltage(model_, address_, tool_flange_voltage_, error)) {
        RCLCPP_ERROR(logger_, "Cannot power the tool flanges: %s", error.c_str());
        return CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    bus_->open();
    for (size_t m = 0; m < motor_ids_.size(); ++m) {
      if (calibrate_) {
        RCLCPP_INFO(logger_, "Calibrating gripper motor %d (the gripper opens and closes)", motor_ids_[m]);
        const auto ends = bus_->find_stroke(motor_ids_[m], calibration_current_, calibration_time_);
        stroke_min_[m] = std::min(ends[0], ends[1]);
        stroke_max_[m] = std::max(ends[0], ends[1]);
        if (stroke_max_[m] - stroke_min_[m] < min_stroke_) {
          RCLCPP_ERROR(
            logger_, "Gripper motor %d moved only %.2f rad during calibration (expected >= %.2f)", motor_ids_[m],
            stroke_max_[m] - stroke_min_[m], min_stroke_);
          return CallbackReturn::ERROR;
        }
      } else {
        stroke_min_[m] = std::stod(info_.hardware_parameters.at("stroke_min_" + std::to_string(motor_ids_[m])));
        stroke_max_[m] = std::stod(info_.hardware_parameters.at("stroke_max_" + std::to_string(motor_ids_[m])));
      }
      RCLCPP_INFO(logger_, "Gripper motor %d stroke [%.3f, %.3f] rad", motor_ids_[m], stroke_min_[m], stroke_max_[m]);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Gripper configuration failed: %s", e.what());
    if (bus_) {
      bus_->close();
    }
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rby1GripperSystem::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    for (size_t m = 0; m < motor_ids_.size(); ++m) {
      bus_->start_position_control(motor_ids_[m], current_limit_);
    }
    std::vector<MotorSample> now;
    if (!bus_->read(motor_ids_, now)) {
      RCLCPP_ERROR(logger_, "Cannot read the gripper motors");
      return CallbackReturn::ERROR;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    samples_ = now;
    have_samples_ = true;
    for (size_t m = 0; m < motor_ids_.size(); ++m) {
      goals_[m] = now[m].position;  // hold where it is
      fingers_[motor_finger_[m]].command = to_finger(m, now[m].position);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Gripper activation failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  read_failures_ = 0;
  run_ = true;
  thread_ = std::thread([this]() {loop();});
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rby1GripperSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  run_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rby1GripperSystem::on_cleanup(const rclcpp_lifecycle::State & state)
{
  on_deactivate(state);
  if (bus_) {
    try {
      bus_->torque_off(motor_ids_);
    } catch (...) {
    }
    bus_->close();
  }
  return CallbackReturn::SUCCESS;
}

void Rby1GripperSystem::loop()
{
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / rate_));
  auto next = std::chrono::steady_clock::now();
  std::vector<MotorSample> samples;
  std::vector<double> goals;
  while (run_) {
    next += period;
    std::this_thread::sleep_until(next);
    try {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        goals = goals_;
      }
      bus_->write(motor_ids_, goals);
      if (bus_->read(motor_ids_, samples)) {
        std::lock_guard<std::mutex> lock(mutex_);
        samples_ = samples;
        read_failures_ = 0;
      } else {
        ++read_failures_;
      }
    } catch (const std::exception & e) {
      ++read_failures_;
      RCLCPP_WARN(logger_, "Gripper bus error: %s", e.what());
    }
  }
}

std::vector<hardware_interface::StateInterface> Rby1GripperSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (auto & f : fingers_) {
    out.emplace_back(f.name, hardware_interface::HW_IF_POSITION, &f.position);
    out.emplace_back(f.name, hardware_interface::HW_IF_VELOCITY, &f.velocity);
    out.emplace_back(f.name, hardware_interface::HW_IF_EFFORT, &f.effort);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> Rby1GripperSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & f : fingers_) {
    if (f.motor >= 0) {
      out.emplace_back(f.name, hardware_interface::HW_IF_POSITION, &f.command);
    }
  }
  return out;
}

return_type Rby1GripperSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (read_failures_ > static_cast<int>(rate_)) {  // about one second without data
    RCLCPP_ERROR(logger_, "Gripper motors stopped answering");
    return return_type::ERROR;
  }
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !have_samples_) {
    return return_type::OK;
  }
  for (size_t m = 0; m < motor_ids_.size(); ++m) {
    auto & f = fingers_[motor_finger_[m]];
    const double span = stroke_max_[m] - stroke_min_[m];
    const double scale = span > 0.0 ? (open_position_ - closed_position_) / span * (closed_at_min_ ? 1.0 : -1.0) : 0.0;
    f.position = to_finger(m, samples_[m].position);
    f.velocity = samples_[m].velocity * scale;
    f.effort = samples_[m].current;
  }
  for (auto & f : fingers_) {
    if (f.mirror >= 0) {
      f.position = fingers_[f.mirror].position;
      f.velocity = fingers_[f.mirror].velocity;
      f.effort = fingers_[f.mirror].effort;
    }
  }
  return return_type::OK;
}

return_type Rby1GripperSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return return_type::OK;
  }
  for (size_t m = 0; m < motor_ids_.size(); ++m) {
    const double command = fingers_[motor_finger_[m]].command;
    if (std::isfinite(command)) {
      goals_[m] = to_motor(m, command);
    }
  }
  return return_type::OK;
}

}  // namespace rby1_gripper_hardware

PLUGINLIB_EXPORT_CLASS(rby1_gripper_hardware::Rby1GripperSystem, hardware_interface::SystemInterface)
