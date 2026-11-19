// Move the RB-Y1's arms to 4x4 tool targets from topics: MoveIt (OMPL) plans, the driver
// executes.
//
// The same interface as the cuMotion target executor in rby1_isaac_ros, so the
// rby1_examples target examples run against either, unchanged:
//
//   /rby1/<arm>/target_pose    std_msgs/Float64MultiArray  16 values, row-major 4x4 pose of the
//                              arm's tool frame (ee_right / ee_left) in `frame` (base)
//   /rby1/<arm>/target_status  std_msgs/String  READY, PLANNING, EXECUTING ..., DONE, FAILED: <reason>
// with <arm> right_arm or left_arm: each arm has its own topics, and both are served.
// /rby1 is `target_namespace`; the driver's topics are under `driver_namespace`.
//
// Targets for both arms that are waiting when a move is planned go in one move, planned
// for both arms as one (the SRDF group both_arms). One arm's target is planned for that
// arm alone, and the other arm stays where it is.
//
// move_group plans only (plan_only, no ros2_control): the planned path is timed (MoveIt's
// velocity/acceleration scaling, `minimum_time`, or a fixed `duration`), resampled evenly
// and sent to the driver's follow_joint_trajectory, and arrival is confirmed from measured
// joints. Obstacles in the planning scene (rby1_moveit_objects) are planned around; unlike
// cuMotion's executor, nothing is checked while the arm moves (watches_while_moving false).
//
// On start it checks the robot (emergency stop, faults), turns power and servos on
// (enable_robot), bends straight arms and a straight torso to the ready pose
// (ready_if_straight) and waits for move_group. A target that arrives while a move runs
// is kept (the latest per arm) and runs when it ends.
//
// ROS callbacks run on the executor thread; planning and execution, which wait, run on
// the thread that calls run().
//
// Moves the robot.
#pragma once

#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rby1_msgs/action/rby1_joint_command.hpp>
#include <rby1_msgs/msg/robot_state.hpp>
#include <rby1_msgs/srv/set_trajectory_impedance.hpp>
#include <rby1_msgs/srv/state_on_off.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "rby1_moveit_executor/robot.hpp"
#include "rby1_moveit_executor/trajectory.hpp"

namespace rby1_moveit_executor {

// The driver's stream_control, switched off and on again within ~0.1 s, can hand out a
// stream that is already expired: the next trajectory never reaches the robot.
constexpr double kStreamReopenGap = 0.5;
// IK for a target: from the arm's posture, and unless that turns no joint more than
// kNearTurn (rad), from kIkSeeds random ones too; each call is given kIkTime (s). The
// planner is then sent to the chosen joints within kJointTolerance (rad).
// Simulator: one call 1.3 ms, all nine 25 ms. Solutions reached without a jump turned
// a joint 0.99 rad at most for hand moves up to 30 cm.
constexpr int kIkSeeds = 8;
constexpr double kIkTime = 0.05;
constexpr double kNearTurn = 1.0;
constexpr double kJointTolerance = 1e-4;
// MoveIt returns an empty trajectory for a goal its start already meets. With the joint
// goal within this of the arm (rad; under 1 mm at the hand) that is DONE, not a failure.
constexpr double kAlreadyThere = 1e-3;
// The driver's stream channels an arm trajectory goes out on (stream_control's parameters): both
// arms' -- the executor plans for either arm, or both.
constexpr char kStreamChannel[] = "right_arm, left_arm";
// The SRDF group a move of both arms is planned in; one arm's is planned in its own.
constexpr char kBothArms[] = "both_arms";
// How many times a target is planned when MoveIt refuses the path it planned
// (INVALID_MOTION_PLAN: 1 of 29 plans around a 6 cm box, with the finer check of the launch).
constexpr int kPlanAttempts = 3;
// After one arm's target, the other arm's is waited for this long (s), so that targets
// sent to both arms together move them together (without it: one arm, then the other).
constexpr double kGather = 0.1;

std::string planning_failure(int code);

class MoveItExecutor : public rclcpp::Node {
public:
  MoveItExecutor();
  // Start (checks, power, ready pose, move_group), then serve targets until shutdown.
  void run();

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using GetPositionIK = moveit_msgs::srv::GetPositionIK;
  using JointCommand = rby1_msgs::action::Rby1JointCommand;
  using StateOnOff = rby1_msgs::srv::StateOnOff;
  using Targets = std::map<std::string, Eigen::Matrix4d>;  // arm -> its tool's pose

  struct Arm {
    std::string name, tool;
    std::vector<std::string> joints;  // in the planning group's order
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr targets;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
  };

  template<typename T>
  T param(const std::string & name) { return get_parameter(name).get_value<T>(); }
  std::string driver(const std::string & relative) const;
  std::string target(const std::string & relative) const;
  std::string under(const std::string & parameter, const std::string & relative) const;

  void on_joints(const sensor_msgs::msg::JointState & msg);
  void on_target(const std::string & arm, const std_msgs::msg::Float64MultiArray & msg);
  rcl_interfaces::msg::SetParametersResult check_change(const std::vector<rclcpp::Parameter> & parameters);
  // To the status topics of the arms in `targets`; of every arm without them.
  void report(const std::string & text, const Targets & targets = {});

  void start();
  std::map<std::string, double> fresh_positions(std::chrono::duration<double> timeout = std::chrono::seconds(5));
  std::string switch_driver(const std::string & service, bool on, const std::string & parameters = "all");
  void apply_impedance(const Targets & targets);
  void close_stream();
  void move_to_ready(const std::vector<std::string> & parts);
  void process(const Targets & targets);
  std::optional<std::vector<double>> joint_goal(const Arm & arm, const Eigen::Matrix4d & target,
                                                const std::vector<double> & current);
  void pose_goal(const Arm & arm, const Eigen::Matrix4d & target, moveit_msgs::msg::Constraints & constraints);
  std::optional<std::pair<JointTrajectory, double>> plan(const Targets & targets);
  void execute(const JointTrajectory & trajectory, const Targets & targets);

  std::vector<Arm> arms_;                        // right_arm, left_arm: the planning group's order
  Posture ready_;                                // the ready pose, from the parameters ready.<part>
  std::mt19937 random_{std::random_device{}()};  // IK seeds
  std::map<std::string, double> velocity_limits_;
  Bounds bounds_;

  std::mutex mutex_;
  std::condition_variable wake_;
  std::map<std::string, double> positions_;
  std::chrono::steady_clock::time_point received_{};
  std::optional<rby1_msgs::msg::RobotState> robot_state_;
  Targets pending_;  // the latest per arm, not yet planned
  bool busy_ = false;
  std::chrono::steady_clock::time_point stream_closed_{};
  bool owns_stream_ = false;                   // the arm stream channel was opened by this executor
  std::optional<std::vector<double>> applied_impedance_;  // last impedance handed to the driver

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<rby1_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp_action::Client<MoveGroup>::SharedPtr planner_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr follower_;
  rclcpp_action::Client<JointCommand>::SharedPtr joint_command_;
  OnSetParametersCallbackHandle::SharedPtr parameter_check_;
};

}  // namespace rby1_moveit_executor
