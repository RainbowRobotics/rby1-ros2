#pragma once

// ros2_control system interface for the RB-Y1 grippers.
//
// backend rb_dynamixel: the RB gripper, one Dynamixel per gripper on the UPC's serial port
//   (/dev/rby1_gripper, protocol 2.0, 2 Mbps). On configure the tool flanges are powered (12 V via
//   the robot), each motor is driven against both ends in current mode to find its stroke, and is
//   then run in current-based position mode with `current_limit` as the grip current.
// backend fake: in-memory grippers for tests.
// backend robotiq_flange: Robotiq on the tool flange through the robot's GripperCommandService
//   (not exposed by the rby1-sdk API; see docs/SDK_COVERAGE.md).
//
// Finger joints: gripper_finger_<r|l>1 take a position command in the URDF finger range
// [closed_position, open_position]; joints with a "mirror_of" parameter copy the state of
// another finger.

#include <array>
#include <limits>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"

#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#define RBY1_GRIPPER_PARAMS_INIT 1
#endif

namespace rby1_gripper_hardware
{

struct MotorSample
{
  double position{0.0};  // motor angle [rad]
  double velocity{0.0};  // [rad/s]
  double current{0.0};   // [A]
};

// Access to the gripper motors (one per gripper).
class GripperBus
{
public:
  virtual ~GripperBus() = default;
  // Returns a warning (empty if none), throws if the bus cannot be used.
  virtual std::string open() = 0;
  virtual void close() noexcept = 0;
  // Drive the motor against both ends with `current` [A] for `seconds` each and return
  // the motor angles at the two ends (unordered).
  virtual std::array<double, 2> find_stroke(int id, double current, double seconds) = 0;
  // Switch to position control with `current_limit` [A].
  virtual void start_position_control(int id, double current_limit) = 0;
  virtual bool read(const std::vector<int> & ids, std::vector<MotorSample> & out) = 0;
  virtual void write(const std::vector<int> & ids, const std::vector<double> & goal) = 0;
  virtual void torque_off(const std::vector<int> & ids) = 0;
};

std::unique_ptr<GripperBus> make_dynamixel_bus(const std::string & device);
std::unique_ptr<GripperBus> make_fake_bus();
bool set_tool_flange_voltage(const std::string & model, const std::string & address, int voltage, std::string & error);

class Rby1GripperSystem : public hardware_interface::SystemInterface
{
public:
  ~Rby1GripperSystem() override;

#ifdef RBY1_GRIPPER_PARAMS_INIT
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
#endif
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void set_bus_for_testing(std::unique_ptr<GripperBus> bus) {bus_ = std::move(bus);}

private:
  struct Finger
  {
    std::string name;
    int motor{-1};          // index into motor_ids_, or -1 for mirrors
    int mirror{-1};         // finger index this one mirrors
    double position{0.0}, velocity{0.0}, effort{0.0};
    double command{std::numeric_limits<double>::quiet_NaN()};
  };

  hardware_interface::CallbackReturn initialize(const hardware_interface::HardwareInfo & info);
  void loop();
  double to_finger(size_t motor, double motor_angle) const;
  double to_motor(size_t motor, double finger_position) const;

  rclcpp::Logger logger_{rclcpp::get_logger("rby1_gripper_system")};
  std::string backend_;
  std::string device_;
  std::string address_;
  std::string model_;
  int tool_flange_voltage_{12};
  bool calibrate_{true};
  double calibration_current_{0.5};
  double calibration_time_{3.0};
  double min_stroke_{3.0};       // [rad], below this the calibration failed
  double current_limit_{0.5};
  double open_position_{-0.05};
  double closed_position_{0.0};
  bool closed_at_min_{true};
  double rate_{100.0};
  std::vector<double> stroke_min_, stroke_max_;  // per motor [rad]

  std::vector<Finger> fingers_;
  std::vector<int> motor_ids_;
  std::vector<size_t> motor_finger_;  // driving finger per motor

  std::unique_ptr<GripperBus> bus_;
  std::mutex mutex_;
  std::vector<MotorSample> samples_;
  std::vector<double> goals_;  // motor angles
  bool have_samples_{false};
  std::atomic<bool> run_{false};
  std::atomic<int> read_failures_{0};
  std::thread thread_;
};

}  // namespace rby1_gripper_hardware
