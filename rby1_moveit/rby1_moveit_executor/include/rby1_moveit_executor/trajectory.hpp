// Timing a planned joint path for the RB-Y1 driver's follow_joint_trajectory.
//
// The driver streams each waypoint to the robot and sleeps for the gap to the next
// one, so the waypoint times *are* the motion's speed: they must be evenly spaced and
// short. It reports success
// as soon as it has sent the last waypoint, so a short hold at the end lets the arm
// actually settle there.
#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace rby1_moveit_executor {

using trajectory_msgs::msg::JointTrajectory;
using Bounds = std::map<std::string, std::pair<double, double>>;  // joint -> (lower, upper)

constexpr double kMaxStep = 0.5;  // s between waypoints
// The driver refuses a waypoint outside a joint's limits; one right at a limit can read
// as outside after rounding. Keep this far inside.
constexpr double kLimitMargin = 1e-4;

double seconds(const builtin_interfaces::msg::Duration & stamp);
// Rounds the total first: splitting first turns 1.9999999999 into 1 s + 1e9 ns, wrapped
// to 1.0 s -- a step back in time that the driver sleeps through.
builtin_interfaces::msg::Duration stamp(double t);
std::vector<double> times_of(const JointTrajectory & trajectory);

struct Timing {
  double duration;
  std::string reason;  // duration, minimum_time or moveit
};
// How long the move takes: a fixed `duration` > 0, otherwise MoveIt's own timing
// (`natural`) but never shorter than `minimum_time`.
Timing move_duration(double natural, double duration, double minimum_time);

// `trajectory`'s path in exactly `duration` s, one waypoint every `step` s, at rest at
// both ends. Shape-preserving interpolation (PCHIP) never swings past the planned
// waypoints; `bounds` then keeps every waypoint strictly inside the joint limits.
JointTrajectory retime(const JointTrajectory & trajectory, double duration, double step,
                       const Bounds & bounds = {});

// Highest |joint velocity| / limit over the trajectory, and which joint.
std::pair<double, std::string> peak_speed_ratio(const JointTrajectory & trajectory,
                                                const std::map<std::string, double> & limits);

// `hold` s of the final pose appended, at rest, in `step` increments.
JointTrajectory with_hold(const JointTrajectory & trajectory, double hold, double step);

// A 4x4 row-major homogeneous transform from 16 numbers; std::invalid_argument otherwise.
Eigen::Matrix4d check_transform(const std::vector<double> & values);

// Monotone piecewise-cubic interpolation (Fritsch-Carlson, as scipy's
// PchipInterpolator): value and first derivative of each column at `at`.
void pchip(const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & at,
           std::vector<double> & value, std::vector<double> & slope);

}  // namespace rby1_moveit_executor
