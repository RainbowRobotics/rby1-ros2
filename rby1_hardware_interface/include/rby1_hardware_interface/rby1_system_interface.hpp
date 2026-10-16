#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"

#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#define RBY1_HARDWARE_PARAMS_INIT 1
#endif

#include "rby1_hardware_interface/command_modes.hpp"
#include "rby1_hardware_interface/robot_backend.hpp"
#include "rby1_hardware_interface/triple_buffer.hpp"

namespace rby1_hardware_interface
{

// ros2_control system interface for one RB-Y1 (arms, torso, head, mobile base).
//
// - State comes from the SDK state stream (StartStateUpdate); read() only copies it.
// - Each component (torso, right_arm, left_arm, head, mobility, body) is driven by the
//   CommandMode selected from the claimed command interfaces (see command_modes.hpp).
// - A stream thread sends the conditioned commands on one SDK command stream at
//   command_rate; unclaimed torso/arm/head joints are held where they were.
class Rby1SystemInterface : public hardware_interface::SystemInterface
{
public:
  enum class StatusField : uint8_t;  // values of the rby1_status gpio

  Rby1SystemInterface();
  ~Rby1SystemInterface() override;

#ifdef RBY1_HARDWARE_PARAMS_INIT
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
#endif
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // For tests: use this backend instead of creating one from the "backend" parameter.
  void set_backend_for_testing(std::shared_ptr<RobotBackend> backend);
  std::string active_mode(Component component) const;

private:
  hardware_interface::CallbackReturn initialize(const hardware_interface::HardwareInfo & info);
  using ModeArray = std::array<const CommandMode *, kComponentCount>;

  bool parse_joints(const hardware_interface::HardwareInfo & info);
  bool parse_gpios(const hardware_interface::HardwareInfo & info);
  void load_urdf_limits(const std::string & urdf_xml);
  std::optional<Component> component_of_interface(const std::string & full_name) const;
  bool resolve_modes(const std::set<std::string> & claimed, ModeArray & modes, std::string & error) const;
  void on_state(const StateSample & sample);
  bool copy_state();
  void stream_loop();
  void realtime_tick(const RealtimeState & state, RealtimeInput & input);
  bool start_realtime();
  void stop_streaming() noexcept;
  void set_fatal(const std::string & message);
  void update_status_values();

  rclcpp::Logger logger_;
  ParameterTable parameters_;
  std::string model_;
  std::string address_;
  std::string backend_type_;
  double state_rate_{200.0};
  double command_rate_{100.0};
  double state_timeout_{0.1};
  int stream_priority_{1};
  int send_timeout_ms_{200};
  int max_consecutive_failures_{10};
  bool stop_on_preemption_{true};
  ReadyOptions ready_options_;

  // Joints and gpios in hardware-info order.
  std::vector<JointEntry> joints_;
  std::vector<GpioEntry> gpios_;
  std::vector<std::string> command_names_;              // full interface name per slot
  std::unordered_map<std::string, size_t> command_slot_;
  std::vector<Component> slot_component_;               // owning component per slot
  std::vector<double> commands_;     // exported command interfaces
  std::vector<double> conditioned_;  // RT-owned conditioned copy
  std::vector<double> position_, velocity_, effort_;  // exported state, JointEntry order
  // Status gpio states: exported storage (stable addresses) and, per exported state, the value
  // it shows, resolved once in on_init so read() only assigns.
  std::map<std::string, double> status_values_;
  std::vector<std::pair<std::string, std::string>> status_interfaces_;  // (gpio, interface)
  std::vector<std::pair<StatusField, double *>> status_slots_;
  std::vector<std::unique_ptr<CommandMode>> modes_;

  // Mode selection (guarded by mode_mutex_ because Humble switches from a non-RT thread).
  mutable std::mutex mode_mutex_;
  std::set<std::string> claimed_;
  ModeArray active_{};
  ModeArray pending_{};
  std::array<bool, kComponentCount> ready_{};

  // Backend and state stream.
  std::shared_ptr<RobotBackend> backend_;
  bool backend_injected_{false};
  RobotDescription description_;
  TripleBuffer<StateSample> state_buffer_;  // SDK state thread -> read()
  StateSample current_;                     // RT copy

  // Stream thread exchange.
  struct Slot
  {
    std::vector<double> values;
    ModeArray modes{};
    std::array<bool, kComponentCount> ready{};
    std::vector<double> measured;  // JointEntry order
    uint64_t sequence{0};
  };
  std::mutex slot_mutex_;
  Slot slot_;
  std::vector<double> hold_position_;  // JointEntry order; guarded by slot_mutex_
  std::atomic<bool> streaming_{false};
  std::atomic<bool> stream_thread_run_{false};
  std::atomic<bool> fatal_{false};
  std::atomic<uint64_t> clamp_count_{0};
  std::atomic<uint64_t> stream_restarts_{0};
  std::atomic<bool> stream_ok_{false};
  std::string fatal_message_;
  std::mutex fatal_mutex_;
  std::thread stream_thread_;
  bool active_hw_{false};

  // Real-time loop (backend thread at the robot's control period).
  std::vector<double> rt_values_;            // copy of the conditioned slot
  std::vector<double> rt_sent_;              // last sent position target, robot order
  std::vector<int> rt_entry_of_robot_;       // JointEntry index per robot joint, -1 if none
  const CommandMode * rt_mode_{nullptr};
  std::vector<bool> rt_claimed_;             // JointEntry claimed by the real-time mode
  double rt_last_t_{-1.0};
  int rt_stale_{0};
  bool rt_initialized_{false};
  double rt_max_step_{0.004};
  double rt_max_tracking_error_{0.15};
  int rt_stale_ticks_{25};
  int rt_priority_{1};
};

}  // namespace rby1_hardware_interface
