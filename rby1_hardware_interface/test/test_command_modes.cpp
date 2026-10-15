#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rby1_hardware_interface/command_modes.hpp"

namespace rby1_hardware_interface
{
namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Fixture
{
  std::vector<JointEntry> joints;
  std::vector<GpioEntry> gpios;
  ParameterTable params;
  size_t slots{0};

  size_t add_joint(const std::string & name, Component c, bool impedance)
  {
    JointEntry j;
    j.name = name;
    j.component = c;
    j.has_position_limits = true;
    j.lower = -1.0;
    j.upper = 1.0;
    j.velocity_limit = 2.0;
    j.effort_limit = 50.0;
    j.cmd_position = slots++;
    if (impedance) {
      j.cmd_stiffness = slots++;
      j.cmd_damping_ratio = slots++;
      j.cmd_torque_limit = slots++;
    }
    joints.push_back(j);
    return joints.size() - 1;
  }

  void add_arm(const std::string & prefix, Component c, bool impedance)
  {
    for (int i = 0; i < 7; ++i) {
      add_joint(prefix + std::to_string(i), c, impedance);
    }
  }

  GpioEntry & add_gpio(const std::string & name, const std::string & kind, Component c,
    std::map<std::string, std::string> parameters, const std::vector<std::string> & interfaces)
  {
    GpioEntry g;
    g.name = name;
    g.kind = kind;
    g.component = c;
    g.parameters = ParameterTable(std::move(parameters));
    for (const auto & i : interfaces) {
      g.command_slots[i] = slots++;
    }
    gpios.push_back(g);
    return gpios.back();
  }

  std::set<std::string> positions(const std::string & prefix, int n) const
  {
    std::set<std::string> out;
    for (int i = 0; i < n; ++i) {
      out.insert(prefix + std::to_string(i) + "/position");
    }
    return out;
  }
};

const CommandMode * select(const std::vector<std::unique_ptr<CommandMode>> & modes, Component c,
  const std::set<std::string> & claimed)
{
  const CommandMode * found = nullptr;
  int count = 0;
  for (const auto & m : modes) {
    if (m->component() == c && m->matches(claimed)) {
      found = m.get();
      ++count;
    }
  }
  return count == 1 ? found : nullptr;
}

std::vector<std::string> pose_interfaces(int slot)
{
  std::vector<std::string> out;
  for (const auto * f : {"x", "y", "z", "qx", "qy", "qz", "qw"}) {
    out.push_back("t" + std::to_string(slot) + "." + f);
  }
  return out;
}

TEST(CommandModes, SelectsJointModesFromClaims)
{
  Fixture f;
  f.add_arm("right_arm_", Component::kRightArm, true);
  for (int i = 0; i < 6; ++i) {
    f.add_joint("torso_" + std::to_string(i), Component::kTorso, true);
  }
  const auto modes = make_modes(f.joints, f.gpios, f.params);

  auto claims = f.positions("right_arm_", 7);
  const auto * position = select(modes, Component::kRightArm, claims);
  ASSERT_NE(position, nullptr);
  EXPECT_STREQ(position->name(), "joint_position");

  for (int i = 0; i < 7; ++i) {
    claims.insert("right_arm_" + std::to_string(i) + "/stiffness");
    claims.insert("right_arm_" + std::to_string(i) + "/damping_ratio");
  }
  const auto * impedance = select(modes, Component::kRightArm, claims);
  ASSERT_NE(impedance, nullptr);
  EXPECT_STREQ(impedance->name(), "joint_impedance");

  // torque_limit is optional for impedance
  claims.insert("right_arm_0/torque_limit");
  EXPECT_NE(select(modes, Component::kRightArm, claims), nullptr);

  // a partial arm claim is rejected
  auto partial = f.positions("right_arm_", 3);
  EXPECT_EQ(select(modes, Component::kRightArm, partial), nullptr);

  // a partial torso claim is a joint group position command
  const auto * group = select(modes, Component::kTorso, f.positions("torso_", 2));
  ASSERT_NE(group, nullptr);
  EXPECT_STREQ(group->name(), "joint_group_position");
  // torso impedance is allowed
  std::set<std::string> torso_imp = f.positions("torso_", 6);
  for (int i = 0; i < 6; ++i) {
    torso_imp.insert("torso_" + std::to_string(i) + "/stiffness");
    torso_imp.insert("torso_" + std::to_string(i) + "/damping_ratio");
  }
  const auto * torso_impedance = select(modes, Component::kTorso, torso_imp);
  ASSERT_NE(torso_impedance, nullptr);
  EXPECT_STREQ(torso_impedance->name(), "joint_impedance");
}

TEST(CommandModes, JointPositionGuards)
{
  Fixture f;
  f.add_arm("right_arm_", Component::kRightArm, false);
  f.params = ParameterTable({{"joint_position.max_tracking_error", "0.1"}, {"joint_position.max_step", "0.05"}});
  const auto modes = make_modes(f.joints, f.gpios, f.params);
  const auto * mode = select(modes, Component::kRightArm, f.positions("right_arm_", 7));
  ASSERT_NE(mode, nullptr);

  std::vector<double> measured(7, 0.0);
  const StateView state{&measured, &measured, &measured};
  std::vector<double> commands(f.slots, kNaN);
  std::vector<double> conditioned(f.slots, kNaN);
  mode->on_activate(commands, state);
  EXPECT_DOUBLE_EQ(commands[0], 0.0);

  size_t clamps = 0;
  commands[0] = 0.5;     // beyond tracking error (0.1)
  commands[1] = 5.0;     // beyond the joint limit (1.0)
  commands[2] = kNaN;    // invalid -> measured
  ASSERT_TRUE(mode->condition(commands, conditioned, state, clamps));
  EXPECT_DOUBLE_EQ(conditioned[0], 0.1);
  EXPECT_DOUBLE_EQ(conditioned[1], 0.1);
  EXPECT_DOUBLE_EQ(conditioned[2], 0.0);
  EXPECT_GE(clamps, 3u);

  // step limit against the previous conditioned target
  measured[0] = 0.1;
  commands[0] = 0.2;
  ASSERT_TRUE(mode->condition(commands, conditioned, state, clamps));
  EXPECT_NEAR(conditioned[0], 0.15, 1e-12);

  const auto command = std::get<JointPositionCommand>(mode->build(conditioned));
  ASSERT_EQ(command.position.size(), 7u);
  EXPECT_NEAR(command.position[0], 0.15, 1e-12);
  ASSERT_EQ(command.velocity_limit.size(), 7u);
  EXPECT_DOUBLE_EQ(command.velocity_limit[0], 1.0);  // URDF 2.0 * default scale 0.5
}

TEST(CommandModes, JointImpedanceDefaults)
{
  Fixture f;
  f.add_arm("right_arm_", Component::kRightArm, true);
  f.params = ParameterTable({{"joint_impedance.right_arm.stiffness", "100,100,100,100,50,50,50"},
    {"joint_impedance.damping_ratio", "0.8"}});
  const auto modes = make_modes(f.joints, f.gpios, f.params);
  std::set<std::string> claims;
  for (int i = 0; i < 7; ++i) {
    const auto n = "right_arm_" + std::to_string(i);
    claims.insert(n + "/position");
    claims.insert(n + "/stiffness");
    claims.insert(n + "/damping_ratio");
  }
  const auto * mode = select(modes, Component::kRightArm, claims);
  ASSERT_NE(mode, nullptr);
  std::vector<double> measured(7, 0.2);
  const StateView state{&measured, &measured, &measured};
  std::vector<double> commands(f.slots, kNaN);
  std::vector<double> conditioned(f.slots, kNaN);
  mode->on_activate(commands, state);
  size_t clamps = 0;
  ASSERT_TRUE(mode->condition(commands, conditioned, state, clamps));
  const auto c = std::get<JointImpedanceCommand>(mode->build(conditioned));
  EXPECT_DOUBLE_EQ(c.position[3], 0.2);
  EXPECT_DOUBLE_EQ(c.stiffness[0], 100.0);
  EXPECT_DOUBLE_EQ(c.stiffness[6], 50.0);
  EXPECT_DOUBLE_EQ(c.damping_ratio, 0.8);
  EXPECT_DOUBLE_EQ(c.torque_limit[0], 50.0);  // URDF effort

  commands[f.joints[0].cmd_stiffness] = 300.0;
  commands[f.joints[0].cmd_damping_ratio] = 1.0;
  ASSERT_TRUE(mode->condition(commands, conditioned, state, clamps));
  const auto c2 = std::get<JointImpedanceCommand>(mode->build(conditioned));
  EXPECT_DOUBLE_EQ(c2.stiffness[0], 300.0);
}

TEST(CommandModes, GpioModes)
{
  Fixture f;
  f.add_arm("right_arm_", Component::kRightArm, false);
  f.add_gpio("right_arm_gravity_compensation", "gravity_compensation", Component::kRightArm, {}, {"enable"});
  f.add_gpio("base_se2", "se2_velocity", Component::kMobility, {{"max_linear_velocity", "0.5"}},
    {"linear_x", "linear_y", "angular_z"});
  auto pose = pose_interfaces(0);
  pose.push_back("j0.position");
  f.add_gpio("right_arm_cartesian", "cartesian", Component::kRightArm,
    {{"targets", "base:ee_right"}, {"joint_targets", "right_arm_2"}}, pose);
  f.add_gpio("body_optimal_control", "optimal_control", Component::kBody,
    {{"targets", "base:ee_right,base:ee_left"}, {"com_ref_link", "base"}},
    [] {
      auto v = pose_interfaces(0);
      for (const auto & s : pose_interfaces(1)) {v.push_back(s);}
      for (const auto * s : {"com.x", "com.y", "com.z"}) {v.push_back(s);}
      return v;
    }());
  const auto modes = make_modes(f.joints, f.gpios, f.params);

  const auto * gravity = select(modes, Component::kRightArm, {"right_arm_gravity_compensation/enable"});
  ASSERT_NE(gravity, nullptr);
  EXPECT_STREQ(gravity->name(), "gravity_compensation");

  const auto * se2 = select(modes, Component::kMobility,
      {"base_se2/linear_x", "base_se2/linear_y", "base_se2/angular_z"});
  ASSERT_NE(se2, nullptr);
  std::vector<double> measured(7, 0.0);
  const StateView state{&measured, &measured, &measured};
  std::vector<double> commands(f.slots, kNaN);
  std::vector<double> conditioned(f.slots, kNaN);
  se2->on_activate(commands, state);
  commands[f.gpios[1].command_slots.at("linear_x")] = 3.0;
  size_t clamps = 0;
  ASSERT_TRUE(se2->condition(commands, conditioned, state, clamps));
  const auto v = std::get<SE2VelocityCommand>(se2->build(conditioned));
  EXPECT_DOUBLE_EQ(v.linear_x, 0.5);

  std::set<std::string> cart_claims;
  for (const auto & s : pose) {
    cart_claims.insert("right_arm_cartesian/" + s);
  }
  const auto * cartesian = select(modes, Component::kRightArm, cart_claims);
  ASSERT_NE(cartesian, nullptr);
  EXPECT_STREQ(cartesian->name(), "cartesian");
  cartesian->on_activate(commands, state);
  // not ready until the controller writes a pose
  EXPECT_FALSE(cartesian->condition(commands, conditioned, state, clamps));
  const auto & slots = f.gpios[2].command_slots;
  commands[slots.at("t0.x")] = 0.4;
  commands[slots.at("t0.y")] = -0.2;
  commands[slots.at("t0.z")] = 1.0;
  commands[slots.at("t0.qx")] = 0.0;
  commands[slots.at("t0.qy")] = 0.0;
  commands[slots.at("t0.qz")] = 0.0;
  commands[slots.at("t0.qw")] = 2.0;  // normalized by the guard
  commands[slots.at("j0.position")] = 0.3;
  ASSERT_TRUE(cartesian->condition(commands, conditioned, state, clamps));
  const auto c = std::get<CartesianCommand>(cartesian->build(conditioned));
  ASSERT_EQ(c.targets.size(), 1u);
  EXPECT_EQ(c.targets[0].link, "ee_right");
  EXPECT_DOUBLE_EQ(c.targets[0].pose.orientation[3], 1.0);
  ASSERT_EQ(c.joint_targets.size(), 1u);
  EXPECT_EQ(c.joint_targets[0].joint_name, "right_arm_2");

  std::set<std::string> oc_claims;
  for (const auto & e : f.gpios[3].command_slots) {
    oc_claims.insert("body_optimal_control/" + e.first);
  }
  const auto * oc = select(modes, Component::kBody, oc_claims);
  ASSERT_NE(oc, nullptr);
  EXPECT_STREQ(oc->name(), "optimal_control");
}

TEST(CommandModes, RejectsMissingGpioInterfaces)
{
  Fixture f;
  f.add_arm("right_arm_", Component::kRightArm, false);
  f.add_gpio("right_arm_cartesian", "cartesian", Component::kRightArm, {{"targets", "base:ee_right"}}, {"t0.x"});
  EXPECT_THROW(make_modes(f.joints, f.gpios, f.params), std::invalid_argument);
}

TEST(CommandModes, RejectsKindOnWrongComponent)
{
  Fixture f;
  f.add_gpio("head_se2", "se2_velocity", Component::kHead, {}, {"linear_x", "linear_y", "angular_z"});
  EXPECT_THROW(make_modes(f.joints, f.gpios, f.params), std::invalid_argument);
}

}  // namespace
}  // namespace rby1_hardware_interface
