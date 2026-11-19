#include "rby1_moveit_executor/robot.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <urdf/model.h>

namespace rby1_moveit_executor {

const std::map<std::string, std::string> kTool = {{"right_arm", "ee_right"}, {"left_arm", "ee_left"}};

const std::map<std::string, size_t> kPartJoints = {{"right_arm", 7}, {"left_arm", 7}, {"torso", 6}};

namespace {

urdf::Model parse(const std::string & text) {
  urdf::Model model;
  if (!model.initString(text)) throw std::invalid_argument("robot_description is not a URDF this package can read");
  return model;
}

}  // namespace

std::map<std::string, double> joint_velocity_limits(const std::string & urdf) {
  std::map<std::string, double> limits;
  for (const auto & [name, joint] : parse(urdf).joints_) {
    if (joint->limits && joint->limits->velocity > 0) limits[name] = joint->limits->velocity;
  }
  return limits;
}

Bounds joint_position_limits(const std::string & urdf) {
  Bounds bounds;
  for (const auto & [name, joint] : parse(urdf).joints_) {
    if (joint->limits && (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC)) {
      bounds[name] = {joint->limits->lower, joint->limits->upper};
    }
  }
  return bounds;
}

std::vector<std::string> straight_parts(const std::map<std::string, double> & positions, const Posture & ready,
                                        double elbow) {
  std::vector<std::string> parts;
  for (const auto & [part, pose] : ready) {
    const bool torso = part == "torso";
    const auto joint = positions.find(part + (torso ? "_2" : "_3"));
    const double threshold = torso ? 0.5 * std::abs(pose.at(2)) : elbow;
    if (joint != positions.end() && std::abs(joint->second) < threshold) parts.push_back(part);
  }
  return parts;
}

double largest_turn(const std::vector<double> & from, const std::vector<double> & to) {
  if (from.size() != to.size()) throw std::invalid_argument("postures of different sizes");
  double turn = 0.0;
  for (size_t i = 0; i < from.size(); ++i) turn = std::max(turn, std::abs(to[i] - from[i]));
  return turn;
}

size_t least_motion(const std::vector<double> & current, const std::vector<std::vector<double>> & solutions) {
  if (solutions.empty()) throw std::invalid_argument("no joint solution to choose from");
  size_t best = 0;
  for (size_t i = 1; i < solutions.size(); ++i) {
    if (largest_turn(current, solutions[i]) < largest_turn(current, solutions[best])) best = i;
  }
  return best;
}

std::optional<std::string> robot_problem(const rby1_msgs::msg::RobotState & state, const std::string & reset_service) {
  if (state.emo_state) return "Emergency stop is pressed; release it and send the target again";
  if (state.control_manager_state == rby1_msgs::msg::RobotState::STATE_MAJOR_FAULT) {
    return "Control manager is in a major fault. Reset it with: ros2 service call " + reset_service +
           " rby1_msgs/srv/ControlManagerCommand \"{command: 3}\"";
  }
  return std::nullopt;
}

}  // namespace rby1_moveit_executor
