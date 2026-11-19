// The RB-Y1 as this package sees it: tool frames, the ready pose, limits from the
// URDF, and whether the robot is fit to move.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <rby1_msgs/msg/robot_state.hpp>

#include "rby1_moveit_executor/trajectory.hpp"

namespace rby1_moveit_executor {

// The tool frame MoveIt moves to, per arm (the SRDF chain tips).
extern const std::map<std::string, std::string> kTool;

// The parts with a ready pose and how many joints each has. The pose itself is a
// setting (config/ready_pose.yaml): arms bent away from the straight-arm singularity,
// where a 7-joint arm reaches the same hand pose in many shapes, and the torso's knee
// bent a little so it is not straight either for a planner that moves it.
extern const std::map<std::string, size_t> kPartJoints;
using Posture = std::map<std::string, std::vector<double>>;  // part -> joint angles (rad)

std::map<std::string, double> joint_velocity_limits(const std::string & urdf);
Bounds joint_position_limits(const std::string & urdf);

// The parts of `ready` that are straight: an arm whose elbow (joint 3) is within
// `elbow` rad of 0, the torso when its knee (joint 2) is under half its ready angle.
std::vector<std::string> straight_parts(const std::map<std::string, double> & positions, const Posture & ready,
                                        double elbow);

// How far the joint that turns most turns between two postures (rad).
double largest_turn(const std::vector<double> & from, const std::vector<double> & to);

// A 7-joint arm reaches one hand pose in many shapes, some half a turn of a joint
// apart. Of `solutions`, the one the arm turns least to reach from `current`
// (largest_turn); its index. Throws when there is none.
size_t least_motion(const std::vector<double> & current, const std::vector<std::vector<double>> & solutions);

// Why the robot cannot move now, or nothing. `reset_service`: the driver's control_manager_command, named in the advice.
std::optional<std::string> robot_problem(const rby1_msgs::msg::RobotState & state, const std::string & reset_service);

}  // namespace rby1_moveit_executor
