// Drives Rby1SystemInterface through its lifecycle against FakeRobotBackend, the way the
// controller manager would.

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rby1_hardware_interface/fake_robot_backend.hpp"
#include "rby1_hardware_interface/rby1_system_interface.hpp"

namespace rby1_hardware_interface
{
namespace
{

using namespace std::chrono_literals;

std::vector<std::string> joint_names()
{
  std::vector<std::string> out = {"right_wheel", "left_wheel"};
  for (int i = 0; i < 6; ++i) {out.push_back("torso_" + std::to_string(i));}
  for (int i = 0; i < 7; ++i) {out.push_back("right_arm_" + std::to_string(i));}
  for (int i = 0; i < 7; ++i) {out.push_back("left_arm_" + std::to_string(i));}
  out.push_back("head_0");
  out.push_back("head_1");
  return out;
}

std::string make_urdf(bool realtime = false)
{
  std::ostringstream u;
  u << "<?xml version=\"1.0\"?><robot name=\"rby1a\"><link name=\"base\"/>";
  std::string parent = "base";
  for (const auto & j : joint_names()) {
    const bool wheel = j.find("wheel") != std::string::npos;
    u << "<link name=\"l_" << j << "\"/>";
    u << "<joint name=\"" << j << "\" type=\"" << (wheel ? "continuous" : "revolute") << "\">"
      << "<parent link=\"" << parent << "\"/><child link=\"l_" << j << "\"/><axis xyz=\"0 0 1\"/>"
      << "<limit lower=\"-2\" upper=\"2\" velocity=\"2\" effort=\"50\"/></joint>";
    parent = "l_" + j;
  }
  u << "<ros2_control name=\"RBY1System\" type=\"system\"><hardware>"
    << "<plugin>rby1_hardware_interface/Rby1SystemInterface</plugin>"
    << "<param name=\"model\">a</param><param name=\"backend\">fake</param>"
    << "<param name=\"command_rate\">200</param><param name=\"state_rate\">200</param>"
    << "<param name=\"state_timeout\">0.5</param>"
    << "</hardware>";
  for (const auto & j : joint_names()) {
    u << "<joint name=\"" << j << "\">";
    if (j.find("wheel") != std::string::npos) {
      u << "<command_interface name=\"velocity\"/>";
    } else {
      u << "<command_interface name=\"position\"/>";
      if (j.find("arm") != std::string::npos) {
        u << "<command_interface name=\"stiffness\"/><command_interface name=\"damping_ratio\"/>"
          << "<command_interface name=\"torque_limit\"/>";
      }
      if (realtime) {
        u << "<command_interface name=\"effort\"/><command_interface name=\"feedback_gain\"/>";
      }
    }
    u << "<state_interface name=\"position\"/><state_interface name=\"velocity\"/>"
      << "<state_interface name=\"effort\"/></joint>";
  }
  u << "<gpio name=\"right_arm_gravity_compensation\"><param name=\"kind\">gravity_compensation</param>"
    << "<param name=\"component\">right_arm</param><command_interface name=\"enable\"/></gpio>"
    << "<gpio name=\"base_se2\"><param name=\"kind\">se2_velocity</param><param name=\"component\">mobility</param>"
    << "<command_interface name=\"linear_x\"/><command_interface name=\"linear_y\"/>"
    << "<command_interface name=\"angular_z\"/></gpio>"
    << "<gpio name=\"rby1_status\"><param name=\"kind\">status</param>"
    << "<state_interface name=\"control_manager_state\"/><state_interface name=\"mode_right_arm\"/>"
    << "<state_interface name=\"stream_active\"/></gpio>";
  u << "</ros2_control></robot>";
  return u.str();
}

class SystemInterfaceTest : public ::testing::Test
{
protected:
  virtual bool realtime() const {return false;}

  void SetUp() override
  {
    const auto infos = hardware_interface::parse_control_resources_from_urdf(make_urdf(realtime()));
    ASSERT_EQ(infos.size(), 1u);
    info_ = infos[0];
    backend_ = std::make_shared<FakeRobotBackend>("a");
    hw_ = std::make_unique<Rby1SystemInterface>();
    hw_->set_backend_for_testing(backend_);
#ifdef RBY1_HARDWARE_PARAMS_INIT
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = info_;
    ASSERT_EQ(hw_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);
#else
    ASSERT_EQ(hw_->on_init(info_), hardware_interface::CallbackReturn::SUCCESS);
#endif
    states_ = hw_->export_state_interfaces();
    commands_ = hw_->export_command_interfaces();
    const rclcpp_lifecycle::State state;
    ASSERT_EQ(hw_->on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(hw_->on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    const rclcpp_lifecycle::State state;
    hw_->on_deactivate(state);
    hw_->on_cleanup(state);
  }

  hardware_interface::CommandInterface & command(const std::string & name)
  {
    for (auto & c : commands_) {
      if (c.get_name() == name) {
        return c;
      }
    }
    throw std::runtime_error("no command interface " + name);
  }

  double state(const std::string & name)
  {
    for (auto & s : states_) {
      if (s.get_name() == name) {
#ifdef RBY1_HARDWARE_PARAMS_INIT
        return s.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
#else
        return s.get_value();
#endif
      }
    }
    throw std::runtime_error("no state interface " + name);
  }

  std::vector<std::string> arm(const std::string & side, const std::vector<std::string> & ifs)
  {
    std::vector<std::string> out;
    for (int i = 0; i < 7; ++i) {
      for (const auto & f : ifs) {
        out.push_back(side + "_arm_" + std::to_string(i) + "/" + f);
      }
    }
    return out;
  }

  hardware_interface::return_type cycle()
  {
    const rclcpp::Time t(0, 0, RCL_STEADY_TIME);
    const rclcpp::Duration d(0, 10000000);
    auto r = hw_->read(t, d);
    if (r != hardware_interface::return_type::OK) {
      return r;
    }
    return hw_->write(t, d);
  }

  void run_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      cycle();
      std::this_thread::sleep_for(5ms);
    }
  }

  bool switch_to(const std::vector<std::string> & start, const std::vector<std::string> & stop = {})
  {
    if (hw_->prepare_command_mode_switch(start, stop) != hardware_interface::return_type::OK) {
      return false;
    }
    return hw_->perform_command_mode_switch(start, stop) == hardware_interface::return_type::OK;
  }

  hardware_interface::HardwareInfo info_;
  std::shared_ptr<FakeRobotBackend> backend_;
  std::unique_ptr<Rby1SystemInterface> hw_;
  std::vector<hardware_interface::StateInterface> states_;
  std::vector<hardware_interface::CommandInterface> commands_;
};

TEST_F(SystemInterfaceTest, ReadsStateWithoutStreaming)
{
  run_for(100ms);
  EXPECT_EQ(cycle(), hardware_interface::return_type::OK);
  EXPECT_TRUE(backend_->sent_requests().empty());  // nothing is commanded without controllers
  EXPECT_DOUBLE_EQ(state("rby1_status/control_manager_state"), static_cast<double>(ControlManagerStatus::kEnabled));
}

TEST_F(SystemInterfaceTest, JointPositionAndHold)
{
  ASSERT_TRUE(switch_to(arm("right", {"position"})));
  EXPECT_EQ(hw_->active_mode(Component::kRightArm), "joint_position");
  (void)command("right_arm_1/position").set_value(0.1);
  run_for(400ms);
  const auto sent = backend_->sent_requests();
  ASSERT_FALSE(sent.empty());
  const auto & last = sent.back();
  ASSERT_TRUE(std::holds_alternative<JointPositionCommand>(last.at(Component::kRightArm)));
  EXPECT_NEAR(std::get<JointPositionCommand>(last.at(Component::kRightArm)).position[1], 0.1, 1e-9);
  // unclaimed body parts are held, the base is not commanded
  EXPECT_TRUE(std::holds_alternative<JointPositionCommand>(last.at(Component::kTorso)));
  EXPECT_TRUE(std::holds_alternative<JointPositionCommand>(last.at(Component::kLeftArm)));
  EXPECT_TRUE(std::holds_alternative<JointPositionCommand>(last.at(Component::kHead)));
  EXPECT_TRUE(std::holds_alternative<std::monostate>(last.at(Component::kMobility)));
  EXPECT_NEAR(state("right_arm_1/position"), 0.1, 0.02);
  EXPECT_DOUBLE_EQ(state("rby1_status/mode_right_arm"), 1.0);
}

TEST_F(SystemInterfaceTest, SwitchesToImpedanceAndGravityCompensation)
{
  ASSERT_TRUE(switch_to(arm("right", {"position", "stiffness", "damping_ratio"})));
  EXPECT_EQ(hw_->active_mode(Component::kRightArm), "joint_impedance");
  run_for(100ms);
  ASSERT_TRUE(std::holds_alternative<JointImpedanceCommand>(backend_->sent_requests().back().at(Component::kRightArm)));

  ASSERT_TRUE(switch_to({"right_arm_gravity_compensation/enable"}, arm("right", {"position", "stiffness", "damping_ratio"})));
  EXPECT_EQ(hw_->active_mode(Component::kRightArm), "gravity_compensation");
  run_for(100ms);
  ASSERT_TRUE(std::holds_alternative<GravityCompensationCommand>(backend_->sent_requests().back().at(Component::kRightArm)));
}

TEST_F(SystemInterfaceTest, RejectsInvalidClaims)
{
  auto partial = arm("left", {"position"});
  partial.resize(3);
  EXPECT_FALSE(switch_to(partial));
  auto mixed = arm("left", {"position"});
  mixed.push_back("right_arm_gravity_compensation/enable");
  mixed.push_back("right_arm_0/position");
  EXPECT_FALSE(switch_to(mixed));
}

TEST_F(SystemInterfaceTest, BaseVelocityModes)
{
  ASSERT_TRUE(switch_to({"right_wheel/velocity", "left_wheel/velocity"}));
  EXPECT_EQ(hw_->active_mode(Component::kMobility), "joint_velocity");
  (void)command("right_wheel/velocity").set_value(1.0);
  run_for(100ms);
  ASSERT_TRUE(std::holds_alternative<JointVelocityCommand>(backend_->sent_requests().back().at(Component::kMobility)));
  ASSERT_TRUE(switch_to(
      {"base_se2/linear_x", "base_se2/linear_y", "base_se2/angular_z"},
      {"right_wheel/velocity", "left_wheel/velocity"}));
  EXPECT_EQ(hw_->active_mode(Component::kMobility), "se2_velocity");
  run_for(100ms);
  ASSERT_TRUE(std::holds_alternative<SE2VelocityCommand>(backend_->sent_requests().back().at(Component::kMobility)));
}

TEST_F(SystemInterfaceTest, ReopensExpiredStream)
{
  ASSERT_TRUE(switch_to(arm("right", {"position"})));
  run_for(100ms);
  const auto opened = backend_->streams_opened();
  backend_->inject_send_result(StreamStatus::kFinished);
  run_for(200ms);
  EXPECT_GT(backend_->streams_opened(), opened);
  EXPECT_EQ(cycle(), hardware_interface::return_type::OK);
}

TEST_F(SystemInterfaceTest, FaultStopsTheStream)
{
  ASSERT_TRUE(switch_to(arm("right", {"position"})));
  run_for(100ms);
  backend_->set_control_manager(ControlManagerStatus::kMajorFault);
  run_for(200ms);
  EXPECT_EQ(cycle(), hardware_interface::return_type::ERROR);
}

class RealtimeTest : public SystemInterfaceTest
{
protected:
  bool realtime() const override {return true;}

  std::vector<std::string> body(const std::vector<std::string> & ifs)
  {
    std::vector<std::string> out;
    for (const auto & j : joint_names()) {
      if (j.find("wheel") != std::string::npos || j.find("head") != std::string::npos) {
        continue;
      }
      for (const auto & f : ifs) {
        out.push_back(j + "/" + f);
      }
    }
    return out;
  }
};

TEST_F(RealtimeTest, RunsTheRealtimeLoopAndReturnsToTheStream)
{
  const auto claims = body({"position", "effort", "feedback_gain"});
  ASSERT_TRUE(switch_to(claims));
  EXPECT_EQ(hw_->active_mode(Component::kBody), "realtime");
  (void)command("right_arm_2/position").set_value(0.3);
  run_for(500ms);
  EXPECT_TRUE(backend_->realtime_running());
  EXPECT_FALSE(backend_->stream_open());
  EXPECT_GT(backend_->realtime_ticks(), 100u);
  // 0.004 rad per 2 ms tick: 0.3 rad is reached in about 0.15 s
  EXPECT_NEAR(state("right_arm_2/position"), 0.3, 0.01);

  // a far target is approached at no more than 0.004 rad per 2 ms tick (2 rad/s)
  (void)command("right_arm_2/position").set_value(1.5);
  const auto t0 = std::chrono::steady_clock::now();
  run_for(300ms);
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_GT(state("right_arm_2/position"), 0.4);
  EXPECT_LT(state("right_arm_2/position"), 0.3 + 2.0 * elapsed + 0.02);

  // a head position controller joins the real-time loop (head targets are optional claims)
  ASSERT_TRUE(switch_to({"head_0/position", "head_1/position"}));
  EXPECT_EQ(hw_->active_mode(Component::kBody), "realtime");
  // the base cannot run during real-time control
  EXPECT_FALSE(switch_to({"right_wheel/velocity", "left_wheel/velocity"}));
  ASSERT_TRUE(switch_to({}, {"head_0/position", "head_1/position"}));

  // back to joint position on one arm: the loop ends and the stream holds the rest
  ASSERT_TRUE(switch_to(arm("right", {"position"}), claims));
  run_for(300ms);
  EXPECT_FALSE(backend_->realtime_running());
  EXPECT_TRUE(backend_->stream_open());
  EXPECT_EQ(hw_->active_mode(Component::kRightArm), "joint_position");
}

}  // namespace
}  // namespace rby1_hardware_interface

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
