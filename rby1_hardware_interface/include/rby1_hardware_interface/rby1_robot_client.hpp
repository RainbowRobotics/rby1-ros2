#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rby1_hardware_interface
{

struct RobotClientOptions
{
  std::string robot_ip{"127.0.0.1:50051"};
  std::string model{"a"};
  bool auto_reconnect{true};
  double connect_timeout_sec{3.0};
  double read_timeout_sec{0.5};
};

struct CommandResult
{
  bool success{false};
  std::string message;
};

struct JointSample
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  bool has_brake{false};
  bool is_ready{false};
};

struct BrakeGroups
{
  std::vector<bool> left_arm;
  std::vector<bool> right_arm;
  std::vector<bool> torso;
  std::vector<bool> head;
};

struct BatteryStatus
{
  bool available{false};
  double voltage{0.0};
  double current{0.0};
  double percentage{0.0};
};

struct ToolFlangeStatus
{
  bool connected{false};
  std::array<double, 3> ft_force{0.0, 0.0, 0.0};
  std::array<double, 3> ft_torque{0.0, 0.0, 0.0};
  std::array<double, 3> gyro{0.0, 0.0, 0.0};
  std::array<double, 3> acceleration{0.0, 0.0, 0.0};
  bool switch_a{false};
  int32_t output_voltage{0};
  bool digital_input_a{false};
  bool digital_input_b{false};
  bool digital_output_a{false};
  bool digital_output_b{false};
};

struct RobotStateSnapshot
{
  std::unordered_map<std::string, JointSample> joints;
  std::vector<std::string> joint_names;
  BrakeGroups brakes;
  int32_t control_manager_state{0};
  int32_t control_state{0};  // 0 unknown, 1 idle, 2 executing, 3 switching
  double time_scale{1.0};
  bool unlimited_mode_enabled{false};
  bool emo_state{false};
  std::array<double, 3> center_of_mass{0.0, 0.0, 0.0};
  std::array<bool, 2> tool_flange_connected{false, false};
  BatteryStatus battery;
  ToolFlangeStatus left_tool_flange;
  ToolFlangeStatus right_tool_flange;
};

enum class ControlManagerCommand
{
  kEnable,
  kDisable,
  kResetFault,
  kEnableUnlimited
};

enum class JogMode
{
  kAbsolute,
  kRelative,
  kOneStep
};

struct PidGains
{
  uint16_t p{0};
  uint16_t i{0};
  uint16_t d{0};
};

// All public methods of Rby1RobotClient are thread-safe: the underlying
// implementation serializes RBY1 SDK access with an internal mutex.
class Rby1RobotClient
{
public:
  explicit Rby1RobotClient(RobotClientOptions options);
  ~Rby1RobotClient();

  Rby1RobotClient(const Rby1RobotClient &) = delete;
  Rby1RobotClient & operator=(const Rby1RobotClient &) = delete;
  Rby1RobotClient(Rby1RobotClient &&) noexcept;
  Rby1RobotClient & operator=(Rby1RobotClient &&) noexcept;

  void connect();
  void disconnect() noexcept;
  bool is_connected() const;
  const RobotClientOptions & options() const;
  std::vector<std::string> joint_names() const;
  RobotStateSnapshot read_state();

  CommandResult set_power(const std::string & selector, bool enabled);
  CommandResult set_servo(const std::string & selector, bool enabled);
  CommandResult set_brake(const std::string & selector, bool engaged);
  CommandResult set_tool_flange_output_voltage(int voltage);
  CommandResult set_gravity_compensation(const std::string & part_name, bool enabled);
  CommandResult set_control_manager(ControlManagerCommand command);

  // Whole-robot commands (they preempt any command stream with a lower priority).
  CommandResult stop(int priority);
  CommandResult cancel_control();
  CommandResult jog(
    const std::string & joint_name, JogMode mode, double value, double velocity_limit,
    double acceleration_limit, double timeout_sec, int priority, int32_t & finish_code);

  CommandResult set_time_scale(double scale, double & applied);
  CommandResult set_led(uint8_t r, uint8_t g, uint8_t b, double duration, double transition_time, bool blinking,
    double blinking_frequency);
  CommandResult set_tool_flange_digital_output(const std::string & side, unsigned int channel, bool state);
  CommandResult get_parameter(const std::string & name, std::string & value);
  CommandResult set_parameter(const std::string & name, const std::string & value, bool write_db);
  CommandResult reset_odometry(double x, double y, double yaw);
  CommandResult get_logs(unsigned int count, std::vector<std::string> & fault_logs, std::vector<std::string> & entries);

  // Maintenance (the robot manager only offers these with enable_maintenance_services).
  CommandResult get_position_pid_gain(const std::string & device, PidGains & gains);
  CommandResult set_position_pid_gain(const std::string & device, const PidGains & gains);
  CommandResult home_offset_reset(const std::string & joint_name);
  CommandResult set_preset_position(const std::string & joint_name);

private:
  class ImplBase;
  template <typename ModelType>
  friend class Rby1RobotClientImpl;

  RobotClientOptions options_;
  std::unique_ptr<ImplBase> impl_;
};

std::string normalize_model_name(const std::string & model);

}  // namespace rby1_hardware_interface
