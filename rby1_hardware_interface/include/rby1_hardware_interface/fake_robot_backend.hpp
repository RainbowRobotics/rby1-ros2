#pragma once

// In-memory RB-Y1 for tests and SDK-free bring-up (hardware parameter backend:=fake).
// Joint commands are tracked with a velocity-limited first-order response; Cartesian
// commands are accepted but do not move the joints.

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "rby1_hardware_interface/robot_backend.hpp"

namespace rby1_hardware_interface
{

class FakeRobotBackend : public RobotBackend
{
public:
  explicit FakeRobotBackend(std::string model);
  ~FakeRobotBackend() override;

  RobotDescription connect(double timeout_sec) override;
  void disconnect() noexcept override;
  bool is_connected() const override;
  void start_state_stream(double rate_hz, StateCallback callback) override;
  void stop_state_stream() noexcept override;
  bool ensure_ready(const ReadyOptions & options, std::string & message) override;
  bool set_parameter(const std::string & name, const std::string & value) override;
  bool open_stream(int priority, std::string & message) override;
  bool stream_open() const override;
  StreamFeedback send(const RobotCommandRequest & request, int timeout_ms) override;
  void close_stream() noexcept override;
  bool cancel_control() override;

  // Test hooks.
  void inject_send_result(StreamStatus status);   // next send() returns this status
  void set_control_manager(ControlManagerStatus status);
  void set_emo(bool pressed);
  void set_position(size_t robot_index, double position);
  std::vector<RobotCommandRequest> sent_requests() const;
  size_t streams_opened() const;
  static RobotDescription describe(const std::string & model);

private:
  void step(double dt);

  const std::string model_;
  RobotDescription description_;
  mutable std::mutex mutex_;
  bool connected_{false};
  bool stream_open_{false};
  bool ready_{true};
  size_t streams_opened_{0};
  ControlManagerStatus control_manager_{ControlManagerStatus::kEnabled};
  bool emo_{false};
  std::optional<StreamStatus> injected_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> target_;
  std::vector<double> target_velocity_;
  std::vector<double> velocity_limit_;
  std::vector<bool> velocity_mode_;
  std::vector<RobotCommandRequest> sent_;
  StateCallback callback_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace rby1_hardware_interface
