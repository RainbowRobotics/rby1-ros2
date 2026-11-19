// gripper_driver: the RB-Y1 gripper on the hands, virtual or real (`virtual`). The RB-Y1 driver
// does not drive the grippers; this node takes their commands and reports their fingers, so
// what follows a gripper's state -- the gripper's model on TF and its parts in the planning
// scene (gripper.launch.py) -- does not care which runs. Settings: config/gripper/gripper_driver.yaml.
//
//   ros2 launch rby1_tools gripper.launch.py                   # virtual: nothing moves
//   ros2 launch rby1_tools gripper.launch.py virtual:=false    # the gripper itself (on the UPC)
//
// virtual: true -- nothing moves; the reported fingers move toward each command at `speed`.
// virtual: false -- the gripper itself, through the SDK's Dynamixel bus, one motor per hand on
// `device`, as SDK example 35 drives it: at start each motor is homed, pushed with
// `homing_current` to one end and the other, 3 s each, until its travel spans 207°;
// THE FINGERS MOVE, then go to `initial_position`. Before that the motors' supply, the tool flange
// output, is switched to `tool_flange_voltage` through the RB-Y1 driver's tool_flange_power service
// (as SDK example 35 does); without the driver it warns and goes on. Then current-based position control with
// `hold_current` as the current limit; the state is read back from the encoders. Built with
// the real gripper only where the SDK is (RBY1_SDK_PATH); elsewhere only virtual: true runs.
//
// Per hand in `hands`: <topic_prefix>/<hand>_gripper/grasp (std_msgs/Bool: true closes, false
// opens to `open_position`), /command (std_msgs/Float64, each finger's opening in m: 0 closed ..
// max_opening open; outside that it is refused with a warning) and /state (sensor_msgs/JointState
// at `rate` Hz: gripper_finger_<r|l>1 and _2, both at the opening). Positions in the settings
// are 0 (closed) .. 1000 (max_opening).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#ifdef RBY1_TOOLS_WITH_MSGS
#include <rby1_msgs/srv/state_on_off.hpp>
#endif

#ifdef RBY1_TOOLS_WITH_SDK
#include "rby1-sdk/base/dynamixel_bus.h"
#endif
#include "rby1_tools/gripper.hpp"

class GripperDriver : public rclcpp::Node {
public:
  GripperDriver() : rclcpp::Node("gripper_driver") {
    virtual_ = declare_parameter("virtual", true);
    const auto hands = declare_parameter("hands", std::vector<std::string>{"right", "left"});
    const auto prefix = declare_parameter("topic_prefix", "/rby1");
    max_opening_ = declare_parameter("max_opening", 0.05);
    const auto open_position = declare_parameter("open_position", 1000);
    const auto initial_position = declare_parameter("initial_position", 1000);
    const double rate = declare_parameter("rate", 50.0);
    speed_ = declare_parameter("speed", 0.05);
    const auto device = declare_parameter("device", "/dev/rby1_gripper");
    const int64_t right_id = declare_parameter("right_id", 0);
    const int64_t left_id = declare_parameter("left_id", 1);
    const bool homing = declare_parameter("homing", true);
    homing_current_ = declare_parameter("homing_current", 0.5);
    hold_current_ = declare_parameter("hold_current", 5.0);
    inverted_ = declare_parameter("inverted", false);
    tool_flange_voltage_ = declare_parameter("tool_flange_voltage", 12);
    driver_namespace_ = declare_parameter("driver_namespace", "rby1");   // gripper.launch.py driver_namespace:=
    while (!driver_namespace_.empty() && driver_namespace_.back() == '/') driver_namespace_.pop_back();
    if (!driver_namespace_.empty() && driver_namespace_.front() != '/') driver_namespace_ = "/" + driver_namespace_;
    // The finger joints of the gripper's URDF -- gripper_finger_<r|l>1_joint (0..max_opening)
    // and _2_joint (-max_opening..0) -- for robot_state_publisher: gripper.launch.py's own
    // (/gripper/joint_states), or the robot's /joint_states on an RB-Y1 v1.3, whose URDF
    // carries the gripper. Empty: not published.
    const auto joint_states_topic = declare_parameter("joint_states_topic", "");
    if (hands.empty()) throw std::invalid_argument("hands is empty");
    if (!(max_opening_ > 0.0) || !(speed_ > 0.0) || !(rate > 0.0)) {
      throw std::invalid_argument("max_opening, speed and rate must be positive");
    }
    open_opening_ = rby1_tools::opening_at(open_position, max_opening_, "open_position");
    const double initial = rby1_tools::opening_at(initial_position, max_opening_, "initial_position");
    dt_ = 1.0 / rate;
    if (!joint_states_topic.empty()) {
      joint_states_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);
    }

    for (const auto & name : hands) {
      if (name != "right" && name != "left") {
        throw std::invalid_argument("hands are right and/or left, got '" + name + "'");
      }
      auto hand = std::make_shared<Hand>();
      hand->name = name;
      hand->id = static_cast<int>(name == "right" ? right_id : left_id);
      hand->opening = hand->goal = initial;
      const std::string letter = name == "right" ? "r" : "l";
      hand->joints = {"gripper_finger_" + letter + "1", "gripper_finger_" + letter + "2"};
      const auto base = prefix + "/" + name + "_gripper/";
      hand->state = create_publisher<sensor_msgs::msg::JointState>(base + "state", 10);
      hand->command = create_subscription<std_msgs::msg::Float64>(
        base + "command", 10, [this, hand](const std_msgs::msg::Float64 & message) {
          if (const auto why = rby1_tools::refused(message.data, max_opening_)) {
            RCLCPP_WARN(get_logger(), "%s gripper: refused command %g: %s", hand->name.c_str(), message.data,
                        why->c_str());
            return;
          }
          hand->goal = message.data;
          RCLCPP_INFO(get_logger(), "%s gripper: to %.4f m", hand->name.c_str(), hand->goal);
        });
      hand->grasp = create_subscription<std_msgs::msg::Bool>(
        base + "grasp", 10, [this, hand](const std_msgs::msg::Bool & message) {
          hand->goal = message.data ? 0.0 : open_opening_;
          RCLCPP_INFO(get_logger(), "%s gripper: %s", hand->name.c_str(), message.data ? "closing" : "opening");
        });
      hands_.push_back(hand);
      RCLCPP_INFO(get_logger(), "%s gripper: %sgrasp, %scommand -> %sstate; starts at %ld (%.4f m)%s",
                  name.c_str(), base.c_str(), base.c_str(), base.c_str(), initial_position, initial,
                  virtual_ ? " (virtual, nothing moves)" : "");
    }
    if (!virtual_) {
      power_tool_flange();
      start_real(device, homing);
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { cycle(); });
  }

  ~GripperDriver() override {
#ifdef RBY1_TOOLS_WITH_SDK
    if (!bus_) return;
    try {
      bus_->GroupSyncWriteTorqueEnable(ids(), rb::DynamixelBus::kTorqueDisable);
    } catch (...) {
    }
#endif
  }

private:
  struct Hand {
    std::string name;
    int id = 0;
    std::vector<std::string> joints;
    double min = std::numeric_limits<double>::infinity();    // real: encoder at one end, rad
    double max = -std::numeric_limits<double>::infinity();   // and the other
    double opening = 0.0;   // m, stepped (virtual) or read back (real)
    double goal = 0.0;      // m
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp;
  };

  std::vector<int> ids() const {
    std::vector<int> result;
    for (const auto & hand : hands_) result.push_back(hand->id);
    return result;
  }

  void cycle() {
    std::vector<double> velocity(hands_.size(), 0.0);
    if (virtual_) {
      for (size_t i = 0; i < hands_.size(); ++i) {
        auto & hand = *hands_[i];
        const double before = hand.opening;
        hand.opening = rby1_tools::step_toward(hand.opening, hand.goal, speed_, dt_);
        velocity[i] = (hand.opening - before) / dt_;
      }
    } else if (!read_real()) {
      return;
    }
    const auto stamp = now();
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    for (size_t h = 0; h < hands_.size(); ++h) {
      const auto & hand = *hands_[h];
      sensor_msgs::msg::JointState state;
      state.header.stamp = stamp;
      state.name = hand.joints;
      state.position = {hand.opening, hand.opening};
      if (virtual_) state.velocity = {velocity[h], velocity[h]};
      hand.state->publish(state);
      for (size_t i = 0; i < hand.joints.size(); ++i) {
        joints.name.push_back(hand.joints[i] + "_joint");
        joints.position.push_back((i == 0 ? 1.0 : -1.0) * hand.opening);   // finger 2 moves along -x in the URDF
      }
    }
    if (joint_states_) joint_states_->publish(joints);
  }

#ifdef RBY1_TOOLS_WITH_SDK
  void start_real(const std::string & device, bool homing) {
    bus_ = std::make_unique<rb::DynamixelBus>(device);
    if (!bus_->OpenPort()) throw std::runtime_error("could not open " + device);
    if (!bus_->SetBaudRate(rb::DynamixelBus::kDefaultBaudrate)) {
      throw std::runtime_error("could not set the baud rate of " + device);
    }
    for (const auto & hand : hands_) {
      if (!bus_->Ping(hand->id)) {
        throw std::runtime_error(hand->name + " gripper: Dynamixel id " + std::to_string(hand->id) +
                                 " does not answer on " + device + " -- is the tool flange's 12 V on? (" +
                                 "ros2 service call " + driver_namespace_ + "/tool_flange_power " +
                                 "rby1_msgs/srv/StateOnOff \"{state: true, parameters: '12'}\")");
      }
      RCLCPP_INFO(get_logger(), "%s gripper: Dynamixel id %d on %s", hand->name.c_str(), hand->id, device.c_str());
    }
    const auto all = ids();
    bus_->SetTorqueConstant(std::vector<double>(*std::max_element(all.begin(), all.end()) + 1, 1.0));
    bus_->GroupSyncWriteTorqueEnable(all, rb::DynamixelBus::kTorqueEnable);
    if (homing) home(all);
    for (const auto & hand : hands_) {
      if (!std::isfinite(hand->min) || !std::isfinite(hand->max) || !(hand->max > hand->min)) {
        throw std::runtime_error(hand->name + " gripper: its travel is not known (homing: true learns it)");
      }
    }
    set_mode(all, rb::DynamixelBus::kCurrentBasedPositionControlMode);
    std::vector<std::pair<int, double>> limits;
    for (const auto & hand : hands_) limits.emplace_back(hand->id, hold_current_);
    bus_->GroupSyncWriteSendTorque(limits);
    // The goals stay at initial_position: the first cycle moves the fingers there.
  }

  void set_mode(const std::vector<int> & ids, int mode) {
    bus_->GroupSyncWriteTorqueEnable(ids, rb::DynamixelBus::kTorqueDisable);
    std::vector<std::pair<int, int>> modes;
    for (int id : ids) modes.emplace_back(id, mode);
    bus_->GroupSyncWriteOperatingMode(modes);
    bus_->GroupSyncWriteTorqueEnable(ids, rb::DynamixelBus::kTorqueEnable);
  }

  // SDK example 35's homing (v0.10.1): push every motor one way, then the other, with
  // `homing_current` for 3 s each, until each motor's travel spans kMinTravel; the extremes read
  // on the way are the travel. Short of it after 3 rounds: refused, not run on part of the travel.
  // (The homing before stopped a direction once the encoder stood still, one round, at 0.3 A: on
  // the robot the fingers closed and never opened; this one worked there, 2026-11-09, in the
  // rby1-gripper Muscat plugin's copy.)
  void home(const std::vector<int> & ids) {
    RCLCPP_WARN(get_logger(), "homing: the fingers move to both ends of their travel (%.2f A)", homing_current_);
    set_mode(ids, rb::DynamixelBus::kCurrentControlMode);
    constexpr int kPhases = 6;
    for (int phase = 0; phase < kPhases && rclcpp::ok(); ++phase) {
      std::vector<std::pair<int, double>> push;
      for (int id : ids) push.emplace_back(id, homing_current_ * (phase % 2 == 0 ? 1.0 : -1.0));
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
        bus_->GroupSyncWriteSendTorque(push);
        if (const auto read = bus_->GroupFastSyncReadEncoder(ids)) {
          for (const auto & hand : hands_) {
            for (const auto & [id, encoder] : *read) {
              if (id != hand->id) continue;
              hand->min = std::min(hand->min, encoder);
              hand->max = std::max(hand->max, encoder);
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (phase % 2 == 0) continue;  // both ways pushed: a round
      bool done = true;
      for (const auto & hand : hands_) done = done && rby1_tools::spans_travel(hand->min, hand->max);
      if (done) break;
      RCLCPP_WARN(get_logger(), "homing: not both ends yet, round %d of %d", phase / 2 + 1, kPhases / 2);
    }
    std::vector<std::pair<int, double>> rest;
    for (int id : ids) rest.emplace_back(id, 0.0);
    bus_->GroupSyncWriteSendTorque(rest);
    for (const auto & hand : hands_) {
      const double span = (hand->max - hand->min) * 180.0 / M_PI;
      RCLCPP_INFO(get_logger(), "%s gripper: travel %.3f .. %.3f rad (%.0f°); to %.4f m", hand->name.c_str(),
                  hand->min, hand->max, span, hand->goal);
      if (!rby1_tools::spans_travel(hand->min, hand->max)) {
        throw std::runtime_error(hand->name + " gripper: homing found " + std::to_string(static_cast<int>(span)) +
                                 "° of travel, under " +
                                 std::to_string(static_cast<int>(rby1_tools::kMinTravel * 180.0 / M_PI)) +
                                 "°: the fingers did not reach both ends (homing_current " +
                                 std::to_string(homing_current_) + " A)");
      }
    }
  }

  // Each finger's opening (m) from a motor's encoder, and back. Open is `max` unless
  // `inverted` (example 35's GRIPPER_DIRECTION).
  double opening_of(const Hand & hand, double encoder) const {
    double fraction = (encoder - hand.min) / (hand.max - hand.min);
    if (inverted_) fraction = 1.0 - fraction;
    return std::clamp(fraction, 0.0, 1.0) * max_opening_;
  }

  double encoder_of(const Hand & hand, double opening) const {
    double fraction = std::clamp(opening / max_opening_, 0.0, 1.0);
    if (inverted_) fraction = 1.0 - fraction;
    return hand.min + fraction * (hand.max - hand.min);
  }

  // Sends the goals and reads the openings back; false when the encoders did not answer.
  bool read_real() {
    std::vector<std::pair<int, double>> goals;
    for (const auto & hand : hands_) goals.emplace_back(hand->id, encoder_of(*hand, hand->goal));
    bus_->GroupSyncWriteSendPosition(goals);
    const auto read = bus_->GroupFastSyncReadEncoder(ids());
    if (!read) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "the gripper's encoders did not answer");
      return false;
    }
    for (const auto & hand : hands_) {
      for (const auto & [id, encoder] : *read) {
        if (id == hand->id) hand->opening = opening_of(*hand, encoder);
      }
    }
    return true;
  }

  std::unique_ptr<rb::DynamixelBus> bus_;
#else
  void start_real(const std::string &, bool) {
    throw std::runtime_error("virtual: false needs the real gripper, but this gripper_driver was built without the "
                             "RB-Y1 SDK (set RBY1_SDK_PATH and rebuild rby1_tools), or set virtual: true");
  }

  bool read_real() { return false; }
#endif

  // The motors' supply: switch the tool flange output (both hands) to tool_flange_voltage through the
  // RB-Y1 driver, as SDK example 35 does before it opens the bus, and give the motors 1 s to boot.
  // 0 leaves the flange as it is. Without the driver (or without rby1_msgs in this build) it warns:
  // the flange may be on already, and the Dynamixel ping that follows says so if it is not.
  void power_tool_flange() {
    if (tool_flange_voltage_ == 0) return;
    if (tool_flange_voltage_ != 12 && tool_flange_voltage_ != 24) {
      throw std::invalid_argument("tool_flange_voltage is 0 (leave it), 12 or 24, not " +
                                  std::to_string(tool_flange_voltage_));
    }
#ifdef RBY1_TOOLS_WITH_MSGS
    auto caller = std::make_shared<rclcpp::Node>("gripper_driver_tool_flange");
    auto client = caller->create_client<rby1_msgs::srv::StateOnOff>(driver_namespace_ + "/tool_flange_power");
    if (!client->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_WARN(get_logger(), "%s/tool_flange_power is not there (is the RB-Y1 driver running?): the tool "
                  "flange's %ld V is not switched on here", driver_namespace_.c_str(), tool_flange_voltage_);
      return;
    }
    auto request = std::make_shared<rby1_msgs::srv::StateOnOff::Request>();
    request->state = true;
    request->parameters = std::to_string(tool_flange_voltage_);
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(caller, future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "%s/tool_flange_power did not answer", driver_namespace_.c_str());
      return;
    }
    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(get_logger(), "tool flange %ld V refused: %s", tool_flange_voltage_, response->message.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "tool flange output %ld V on (both hands); waiting 1 s for the motors",
                tool_flange_voltage_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
#else
    RCLCPP_WARN(get_logger(), "built without rby1_msgs: switch the tool flange's %ld V on yourself (%s/"
                "tool_flange_power)", tool_flange_voltage_, driver_namespace_.c_str());
#endif
  }

  bool virtual_ = true;
  int64_t tool_flange_voltage_ = 12;
  std::string driver_namespace_ = "/rby1";   // normalized: "/rby1", or "" for none
  std::vector<std::shared_ptr<Hand>> hands_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
  double max_opening_ = 0.05;
  double open_opening_ = 0.05;
  double speed_ = 0.05;
  double dt_ = 0.02;
  double homing_current_ = 0.5;
  double hold_current_ = 5.0;
  bool inverted_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<GripperDriver>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("gripper_driver"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
