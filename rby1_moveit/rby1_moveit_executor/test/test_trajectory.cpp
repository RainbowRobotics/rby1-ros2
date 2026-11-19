// Timing, transforms and robot checks of rby1_moveit_executor; no ROS graph.
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "rby1_moveit_executor/robot.hpp"
#include "rby1_moveit_executor/trajectory.hpp"

using namespace rby1_moveit_executor;

namespace {

JointTrajectory planned(const std::vector<double> & times, const std::vector<std::vector<double>> & positions) {
  JointTrajectory path;
  path.joint_names = {"a", "b"};
  for (size_t k = 0; k < times.size(); ++k) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions[k];
    point.time_from_start = stamp(times[k]);
    path.points.push_back(point);
  }
  return path;
}

// MoveIt's timing is uneven; ours must be even.
JointTrajectory moveit_path() {
  std::vector<double> times{0.0, 0.3, 0.5, 1.4, 2.0};
  std::vector<std::vector<double>> positions;
  for (int k = 0; k < 5; ++k) positions.push_back({0.8 * k / 4.0, -0.4 * k / 4.0});
  return planned(times, positions);
}

std::string failure(const std::vector<double> & values) {
  try {
    check_transform(values);
  } catch (const std::invalid_argument & error) {
    return error.what();
  }
  return "";
}

}  // namespace

TEST(Retime, IsEvenStartsAndEndsAtRestAndKeepsTheEnds) {
  const auto moved = retime(moveit_path(), 3.0, 0.05);
  const auto times = times_of(moved);
  EXPECT_NEAR(times.back(), 3.0, 1e-9);
  for (size_t k = 1; k < times.size(); ++k) EXPECT_NEAR(times[k] - times[k - 1], 0.05, 1e-9);
  EXPECT_DOUBLE_EQ(moved.points.front().positions[0], 0.0);
  EXPECT_NEAR(moved.points.back().positions[0], 0.8, 1e-12);
  EXPECT_NEAR(moved.points.back().positions[1], -0.4, 1e-12);
  EXPECT_EQ(moved.points.front().velocities, (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(moved.points.back().velocities, (std::vector<double>{0.0, 0.0}));
}

TEST(Retime, RefusesAStepTheDriverWouldTimeOutOn) {
  EXPECT_THROW(retime(moveit_path(), 3.0, 1.2), std::invalid_argument);
}

TEST(Retime, NeverLeavesTheJointLimits) {
  // A joint parked exactly at its limit (OMPL leaves them there) and a path that turns
  // sharply: a plain cubic spline swings past both.
  const auto path = planned({0.0, 0.2, 0.4, 2.0},
                            {{-M_PI, 0.0}, {-M_PI + 0.3, 0.5}, {-M_PI + 0.31, 0.5}, {-M_PI + 0.35, 0.0}});
  const auto moved = retime(path, 2.0, 0.05, {{"a", {-M_PI, M_PI}}, {"b", {-1.0, 0.5}}});
  for (const auto & point : moved.points) {
    EXPECT_GT(point.positions[0], -M_PI);
    EXPECT_LT(point.positions[1], 0.5);
  }
}

TEST(Pchip, MatchesScipyOnAKnownCurve) {
  // scipy.interpolate.PchipInterpolator([0, 1, 2, 3], [0, 1, 1, 3]) at [0.5, 1.5, 2.5, 3.0]:
  // values [0.6875, 1, 1.625, 3], derivatives [1.125, 0, 2.25, 3].
  std::vector<double> value, slope;
  pchip({0, 1, 2, 3}, {0, 1, 1, 3}, {0.5, 1.5, 2.5, 3.0}, value, slope);
  const std::vector<double> values{0.6875, 1.0, 1.625, 3.0}, slopes{1.125, 0.0, 2.25, 3.0};
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(value[i], values[i], 1e-12) << i;
    EXPECT_NEAR(slope[i], slopes[i], 1e-12) << i;
  }
}

TEST(MoveDuration, Order) {
  EXPECT_EQ(move_duration(3.0, 0.0, 0.0).reason, "moveit");
  EXPECT_DOUBLE_EQ(move_duration(1.0, 0.0, 2.0).duration, 2.0);
  EXPECT_EQ(move_duration(1.0, 0.0, 2.0).reason, "minimum_time");
  EXPECT_DOUBLE_EQ(move_duration(1.0, 5.0, 2.0).duration, 5.0);
  EXPECT_EQ(move_duration(1.0, 5.0, 2.0).reason, "duration");
}

TEST(PeakSpeed, IsMeasuredAgainstTheJointLimits) {
  const auto fast = peak_speed_ratio(retime(moveit_path(), 0.5, 0.05), {{"a", 1.0}, {"b", 1.0}});
  EXPECT_EQ(fast.second, "a");
  EXPECT_GT(fast.first, 1.0);
  EXPECT_LT(peak_speed_ratio(retime(moveit_path(), 5.0, 0.05), {{"a", 1.0}, {"b", 1.0}}).first, 1.0);
}

TEST(Hold, AppendsTheFinalPoseAtRest) {
  const auto held = with_hold(retime(moveit_path(), 2.0, 0.05), 0.5, 0.05);
  EXPECT_NEAR(seconds(held.points.back().time_from_start), 2.5, 1e-9);
  EXPECT_NEAR(held.points.back().positions[0], 0.8, 1e-12);
}

TEST(Stamp, NeverStepsBackInTime) {
  double t = 0.0, last = 0.0;
  for (int i = 0; i < 400; ++i) {
    t += 0.05;
    const double now = seconds(stamp(t));
    EXPECT_NEAR(now, t, 1e-9);
    EXPECT_GT(now, last);
    last = now;
  }
}

TEST(Target, BadTransformsAreRefused) {
  std::vector<double> fifteen(15, 0.0);
  EXPECT_NE(failure(fifteen).find("16 values"), std::string::npos);
  EXPECT_NE(failure({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1}).find("bottom row"), std::string::npos);
  EXPECT_NE(failure({2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}).find("rotation"), std::string::npos);
  EXPECT_EQ(failure({1, 0, 0, 0.4, 0, 1, 0, -0.3, 0, 0, 1, 1.1, 0, 0, 0, 1}), "");
}

TEST(Robot, LimitsComeFromTheUrdf) {
  const std::string urdf =
    "<robot name='r'><link name='l0'/><link name='l1'/><link name='l2'/>"
    "<joint name='j1' type='revolute'><parent link='l0'/><child link='l1'/>"
    "<limit velocity='2.5' effort='1' lower='-1' upper='2'/></joint>"
    "<joint name='w' type='continuous'><parent link='l1'/><child link='l2'/>"
    "<limit velocity='3.0' effort='1'/></joint></robot>";
  const auto velocity = joint_velocity_limits(urdf);
  EXPECT_DOUBLE_EQ(velocity.at("j1"), 2.5);
  EXPECT_DOUBLE_EQ(velocity.at("w"), 3.0);
  const auto bounds = joint_position_limits(urdf);
  ASSERT_EQ(bounds.size(), 1u);
  EXPECT_DOUBLE_EQ(bounds.at("j1").first, -1.0);
  EXPECT_DOUBLE_EQ(bounds.at("j1").second, 2.0);
}

TEST(Robot, OnlyStraightPartsAreBent) {
  using Parts = std::vector<std::string>;
  const Posture ready{{"right_arm", {0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0}},
                      {"left_arm", {0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0}},
                      {"torso", {0.0, 0.1, -0.2, 0.1, 0.0, 0.0}}};
  EXPECT_EQ(straight_parts({{"right_arm_3", 0.05}, {"left_arm_3", -1.2}, {"torso_2", -0.2}}, ready, 0.3),
            Parts{"right_arm"});
  EXPECT_EQ(straight_parts({{"right_arm_3", 0.0}, {"left_arm_3", 0.0}, {"torso_2", 0.0}}, ready, 0.3),
            (Parts{"left_arm", "right_arm", "torso"}));
  EXPECT_EQ(straight_parts({{"right_arm_3", -1.57}, {"left_arm_3", -1.57}, {"torso_2", -0.05}}, ready, 0.3),
            Parts{"torso"});
  EXPECT_TRUE(straight_parts({{"right_arm_3", -1.57}, {"left_arm_3", -1.57}, {"torso_2", -0.2}}, ready, 0.3).empty());
  EXPECT_TRUE(straight_parts({}, ready, 0.3).empty());  // a part the driver does not report is left alone
  // A ready torso that is straight itself is never moved; a part without a ready pose neither.
  Posture flat = ready;
  flat["torso"] = std::vector<double>(6, 0.0);
  flat.erase("left_arm");
  EXPECT_EQ(straight_parts({{"right_arm_3", 0.0}, {"left_arm_3", 0.0}, {"torso_2", 0.0}}, flat, 0.3),
            Parts{"right_arm"});
}

TEST(Robot, TheSolutionTheArmTurnsLeastForIsTaken) {
  const std::vector<double> current{0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0};
  // The same hand pose three ways: a wrist joint half a turn away, nearly where the
  // arm is, and every joint a little off.
  const std::vector<std::vector<double>> solutions{
    {0.0, -0.5, 0.0, -1.57, 3.14, 0.0, 0.0},
    {0.01, -0.52, 0.0, -1.55, 0.0, 0.02, 0.0},
    {0.2, -0.7, 0.2, -1.77, 0.2, 0.2, 0.2}};
  EXPECT_EQ(rby1_moveit_executor::least_motion(current, solutions), 1u);
  EXPECT_NEAR(rby1_moveit_executor::largest_turn(current, solutions[0]), 3.14, 1e-12);
  EXPECT_NEAR(rby1_moveit_executor::largest_turn(current, solutions[1]), 0.02, 1e-12);
  EXPECT_THROW(rby1_moveit_executor::least_motion(current, {}), std::invalid_argument);
  EXPECT_THROW(rby1_moveit_executor::largest_turn(current, {0.0}), std::invalid_argument);
}

TEST(Robot, EmergencyStopAndMajorFaultStopIt) {
  rby1_msgs::msg::RobotState state;
  const std::string reset = "/robotA/control_manager_command";
  EXPECT_FALSE(robot_problem(state, reset).has_value());
  state.emo_state = true;
  EXPECT_NE(robot_problem(state, reset)->find("Emergency stop"), std::string::npos);
  state.emo_state = false;
  state.control_manager_state = rby1_msgs::msg::RobotState::STATE_MAJOR_FAULT;
  EXPECT_NE(robot_problem(state, reset)->find("command: 3"), std::string::npos);
  EXPECT_NE(robot_problem(state, reset)->find(reset), std::string::npos);
}
