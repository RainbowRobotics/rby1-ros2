// Gripper buses: RB gripper Dynamixels through rby1-sdk's DynamixelBus, and an in-memory fake.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "rby1-sdk/base/dynamixel_bus.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/device.h"

#include "rby1_gripper_hardware/rby1_gripper_system.hpp"

namespace rby1_gripper_hardware
{
namespace
{

using namespace std::chrono_literals;

class DynamixelGripperBus final : public GripperBus
{
public:
  explicit DynamixelGripperBus(std::string device) : device_(std::move(device)) {}

  std::string open() override
  {
    std::string warning;
    try {
      // Sets the USB serial latency timer to 1 ms (needs write access to sysfs).
      rb::upc::InitializeDevice(device_);
    } catch (const std::exception & e) {
      warning = std::string("could not set the USB latency timer (") + e.what() +
        "); install the udev rule from docs/DEPLOYMENT.md for 1 ms latency";
    }
    bus_ = std::make_unique<rb::DynamixelBus>(device_);
    if (!bus_->OpenPort()) {
      throw std::runtime_error("cannot open " + device_);
    }
    if (!bus_->SetBaudRate(rb::DynamixelBus::kDefaultBaudrate)) {
      throw std::runtime_error("cannot set the baud rate on " + device_);
    }
    return warning;
  }

  void close() noexcept override
  {
    bus_.reset();
  }

  void set_mode(int id, int mode)
  {
    for (int attempt = 0; attempt < 20; ++attempt) {
      const auto current = bus_->ReadOperatingMode(id);
      if (current && *current == mode) {
        break;
      }
      bus_->SendTorqueEnable(id, rb::DynamixelBus::kTorqueDisable);
      bus_->SendOperatingMode(id, mode);
      std::this_thread::sleep_for(20ms);
    }
    for (int attempt = 0; attempt < 20; ++attempt) {
      const auto torque = bus_->ReadTorqueEnable(id);
      if (torque && *torque == rb::DynamixelBus::kTorqueEnable) {
        return;
      }
      bus_->SendTorqueEnable(id, rb::DynamixelBus::kTorqueEnable);
      std::this_thread::sleep_for(20ms);
    }
    throw std::runtime_error("gripper motor " + std::to_string(id) + " does not enable torque");
  }

  std::array<double, 2> find_stroke(int id, double current, double seconds) override
  {
    if (!bus_->Ping(id)) {
      throw std::runtime_error("gripper motor " + std::to_string(id) + " does not answer on " + device_);
    }
    set_mode(id, rb::DynamixelBus::kCurrentControlMode);
    std::array<double, 2> ends{};
    for (int side = 0; side < 2; ++side) {
      bus_->SendCurrent(id, side == 0 ? current : -current);
      std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
      const auto q = bus_->ReadEncoder(id);
      if (!q) {
        throw std::runtime_error("cannot read gripper motor " + std::to_string(id));
      }
      ends[side] = *q;
    }
    bus_->SendCurrent(id, 0.0);
    return ends;
  }

  void start_position_control(int id, double current_limit) override
  {
    set_mode(id, rb::DynamixelBus::kCurrentBasedPositionControlMode);
    // In current-based position mode the goal current is the current limit.
    bus_->SendCurrent(id, current_limit);
  }

  bool read(const std::vector<int> & ids, std::vector<MotorSample> & out) override
  {
    const auto states = bus_->GetMotorStates(ids);
    if (!states || states->size() != ids.size()) {
      return false;
    }
    out.resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      for (const auto & [id, s] : *states) {
        if (id == ids[i]) {
          out[i].position = s.position;
          out[i].velocity = s.velocity;
          out[i].current = s.current;
        }
      }
    }
    return true;
  }

  void write(const std::vector<int> & ids, const std::vector<double> & goal) override
  {
    std::vector<std::pair<int, double>> targets;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (std::isfinite(goal[i])) {
        targets.emplace_back(ids[i], goal[i]);
      }
    }
    bus_->GroupSyncWriteSendPosition(targets);
  }

  void torque_off(const std::vector<int> & ids) override
  {
    if (bus_) {
      bus_->GroupSyncWriteTorqueEnable(ids, rb::DynamixelBus::kTorqueDisable);
    }
  }

private:
  std::string device_;
  std::unique_ptr<rb::DynamixelBus> bus_;
};

class FakeGripperBus final : public GripperBus
{
public:
  std::string open() override {return {};}
  void close() noexcept override {}
  std::array<double, 2> find_stroke(int id, double, double) override
  {
    state_[id];
    return {0.0, 4.0};
  }
  void start_position_control(int, double) override {}
  bool read(const std::vector<int> & ids, std::vector<MotorSample> & out) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    out.resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      auto & s = state_[ids[i]];
      const double error = s.goal - s.sample.position;
      s.sample.velocity = std::clamp(error / 0.05, -4.0, 4.0);
      s.sample.position += s.sample.velocity * 0.01;
      out[i] = s.sample;
    }
    return true;
  }
  void write(const std::vector<int> & ids, const std::vector<double> & goal) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (std::isfinite(goal[i])) {
        state_[ids[i]].goal = goal[i];
      }
    }
  }
  void torque_off(const std::vector<int> &) override {}

private:
  struct Motor
  {
    MotorSample sample;
    double goal{0.0};
  };
  std::mutex mutex_;
  std::map<int, Motor> state_;
};

template <typename Model>
bool set_voltage(const std::string & address, int voltage, std::string & error)
{
  auto robot = rb::Robot<Model>::Create(address);
  if (!robot || !robot->Connect(3, 1000)) {
    error = "cannot connect to the robot at " + address;
    return false;
  }
  const bool ok = robot->SetToolFlangeOutputVoltage("right", voltage) &&
    robot->SetToolFlangeOutputVoltage("left", voltage);
  if (!ok) {
    error = "SetToolFlangeOutputVoltage failed (is 48 V on?)";
  }
  robot->Disconnect();
  return ok;
}

}  // namespace

std::unique_ptr<GripperBus> make_dynamixel_bus(const std::string & device)
{
  return std::make_unique<DynamixelGripperBus>(device);
}

std::unique_ptr<GripperBus> make_fake_bus()
{
  return std::make_unique<FakeGripperBus>();
}

bool set_tool_flange_voltage(const std::string & model, const std::string & address, int voltage, std::string & error)
{
  try {
    if (model == "a") {return set_voltage<rb::y1_model::A>(address, voltage, error);}
    if (model == "m") {return set_voltage<rb::y1_model::M>(address, voltage, error);}
    if (model == "ub") {return set_voltage<rb::y1_model::UB>(address, voltage, error);}
    error = "unknown model '" + model + "'";
  } catch (const std::exception & e) {
    error = e.what();
  }
  return false;
}

}  // namespace rby1_gripper_hardware
