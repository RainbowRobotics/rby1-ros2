#include "rby1_moveit_executor/executor.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "rby1_moveit_executor/robot.hpp"

namespace rby1_moveit_executor {

using namespace std::chrono_literals;
using moveit_msgs::msg::MoveItErrorCodes;

namespace {

// Changeable while running (ros2 param set), each with its legal range.
const std::map<std::string, std::pair<double, double>> kMotion = {
  {"velocity_scaling", {1e-9, 1.0}}, {"acceleration_scaling", {1e-9, 1.0}},
  {"minimum_time", {0.0, 600.0}}, {"duration", {0.0, 600.0}}, {"planning_time", {1e-9, 60.0}},
  {"impedance.stiffness", {1.0, 5000.0}}, {"impedance.damping_ratio", {0.01, 10.0}},
  {"impedance.torque_limit", {0.1, 300.0}}};

template<typename Future>
auto wait(Future && future, std::chrono::duration<double> timeout, const std::string & what) {
  if (future.wait_for(timeout) != std::future_status::ready) {
    throw std::runtime_error("no answer from " + what + " within " +
                             std::to_string(static_cast<int>(timeout.count())) + " s");
  }
  return future.get();
}

std::string round2(double value, const char * format = "%.2f") {
  char text[32];
  std::snprintf(text, sizeof(text), format, value);
  return text;
}

}  // namespace

std::string planning_failure(int code) {
  static const std::map<int, std::string> names = {
    {MoveItErrorCodes::PLANNING_FAILED, "PLANNING_FAILED"},
    {MoveItErrorCodes::INVALID_MOTION_PLAN, "INVALID_MOTION_PLAN"},
    {MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE, "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"},
    {MoveItErrorCodes::CONTROL_FAILED, "CONTROL_FAILED"},
    {MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA, "UNABLE_TO_AQUIRE_SENSOR_DATA"},
    {MoveItErrorCodes::TIMED_OUT, "TIMED_OUT"},
    {MoveItErrorCodes::PREEMPTED, "PREEMPTED"},
    {MoveItErrorCodes::START_STATE_IN_COLLISION, "START_STATE_IN_COLLISION"},
    {MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS, "START_STATE_VIOLATES_PATH_CONSTRAINTS"},
    {MoveItErrorCodes::GOAL_IN_COLLISION, "GOAL_IN_COLLISION"},
    {MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS, "GOAL_VIOLATES_PATH_CONSTRAINTS"},
    {MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED, "GOAL_CONSTRAINTS_VIOLATED"},
    {MoveItErrorCodes::INVALID_GROUP_NAME, "INVALID_GROUP_NAME"},
    {MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS, "INVALID_GOAL_CONSTRAINTS"},
    {MoveItErrorCodes::INVALID_ROBOT_STATE, "INVALID_ROBOT_STATE"},
    {MoveItErrorCodes::INVALID_LINK_NAME, "INVALID_LINK_NAME"},
    {MoveItErrorCodes::INVALID_OBJECT_NAME, "INVALID_OBJECT_NAME"},
    {MoveItErrorCodes::FRAME_TRANSFORM_FAILURE, "FRAME_TRANSFORM_FAILURE"},
    {MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE, "COLLISION_CHECKING_UNAVAILABLE"},
    {MoveItErrorCodes::ROBOT_STATE_STALE, "ROBOT_STATE_STALE"},
    {MoveItErrorCodes::SENSOR_INFO_STALE, "SENSOR_INFO_STALE"},
    {MoveItErrorCodes::NO_IK_SOLUTION, "NO_IK_SOLUTION"},
    {MoveItErrorCodes::FAILURE, "FAILURE"},
  };
  const auto name = names.find(code);
  std::string text = "planning failed: " + (name == names.end() ? std::string("UNKNOWN") : name->second) +
                     " (MoveIt error " + std::to_string(code) + ")";
  if (code == MoveItErrorCodes::START_STATE_IN_COLLISION ||
      code == MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS) {
    text += " -- the arm already touches an obstacle, or itself: move the obstacle away "
            "(ros2 run rby1_moveit_objects scene list) and send the target again";
  } else if (code == MoveItErrorCodes::FAILURE) {
    text += " -- OMPL found no valid start or goal: the arm, or an object attached to it (rby1_moveit_objects), "
            "touches something where it is or where it would end -- an obstacle, or a link missing from the "
            "object's touch_links. Move the target or the obstacle";
  } else if (code == MoveItErrorCodes::PLANNING_FAILED || code == MoveItErrorCodes::INVALID_MOTION_PLAN ||
             code == MoveItErrorCodes::NO_IK_SOLUTION || code == MoveItErrorCodes::GOAL_IN_COLLISION ||
             code == MoveItErrorCodes::TIMED_OUT) {
    text += " -- no collision-free motion to that pose found: it may be out of reach, or an obstacle may be "
            "in the way (ros2 run rby1_moveit_objects scene list). Move the target or the obstacle, or raise "
            "planning_time";
  }
  return text;
}

MoveItExecutor::MoveItExecutor()
: rclcpp::Node("rby1_target_executor") {
  // Name and topics shared with the cuMotion executor: whichever planner runs, targets
  // and `ros2 param set /rby1_target_executor ...` go to the same place.
  declare_parameter("driver_namespace", "rby1");
  declare_parameter("target_namespace", "rby1");   // <target_namespace>/<arm>/target_pose, target_status
  declare_parameter("frame", "base");
  declare_parameter("robot_description", "");
  declare_parameter("enable_robot", true);
  declare_parameter("ready_if_straight", true);
  declare_parameter("straight_elbow", 0.3);
  declare_parameter("ready_time", 4.0);
  declare_parameter("pipeline", "ompl");
  declare_parameter("planner_id", "");
  declare_parameter("planning_attempts", 5);
  declare_parameter("planning_time", 5.0);
  // For a target planned to as a pose: when IK found no joints for it (joint_goal).
  declare_parameter("position_tolerance", 0.001);
  declare_parameter("orientation_tolerance", 0.01);
  declare_parameter("velocity_scaling", 0.5);
  declare_parameter("acceleration_scaling", 0.5);
  declare_parameter("minimum_time", 2.0);
  declare_parameter("duration", 0.0);
  declare_parameter("step", 0.05);
  declare_parameter("hold", 0.5);
  declare_parameter("endpoint_tolerance", 0.03);
  // Follow trajectories in joint impedance (the arm yields to contact), handed to the
  // driver's set_trajectory_impedance before a move; false leaves the driver's own setting.
  declare_parameter("impedance.enabled", false);
  declare_parameter("impedance.stiffness", 500.0);     // N·m/rad, every arm joint (100 left the sim wrist 0.5 rad short)
  declare_parameter("impedance.damping_ratio", 1.0);
  declare_parameter("impedance.torque_limit", 50.0);   // N·m per joint
  // Read by the examples: nothing is checked here while the arm moves.
  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only = true;
  declare_parameter("watches_while_moving", false, read_only);

  // No default: the ready pose has one place, config/ready_pose.yaml, which the launch loads.
  for (const auto & [part, count] : kPartJoints) {
    ready_[part] = declare_parameter("ready." + part, std::vector<double>{});
    if (ready_[part].size() != count) {
      throw std::invalid_argument("ready." + part + " needs " + std::to_string(count) + " joint angles (rad), got " +
                                  std::to_string(ready_[part].size()) + " -- they come from config/ready_pose.yaml "
                                  "through the launch (ready_pose:=/path/to/file.yaml for another file)");
    }
  }
  for (const std::string name : {"right_arm", "left_arm"}) {
    Arm arm;
    arm.name = name;
    arm.tool = kTool.at(name);
    for (size_t i = 0; i < kPartJoints.at(name); ++i) arm.joints.push_back(name + "_" + std::to_string(i));
    arm.status = create_publisher<std_msgs::msg::String>(target(name + "/target_status"), 10);
    arm.targets = create_subscription<std_msgs::msg::Float64MultiArray>(
      target(name + "/target_pose"), 10, [this, name](const std_msgs::msg::Float64MultiArray & msg) {
        on_target(name, msg);
      });
    arms_.push_back(arm);
  }
  const auto urdf = param<std::string>("robot_description");
  if (!urdf.empty()) {
    velocity_limits_ = joint_velocity_limits(urdf);
    bounds_ = joint_position_limits(urdf);
  }

  joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    driver("joint_overall_states"), rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState & msg) {
      on_joints(msg);
    });
  state_sub_ = create_subscription<rby1_msgs::msg::RobotState>(
    driver("robot_state"), 10, [this](const rby1_msgs::msg::RobotState & msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      robot_state_ = msg;
    });
  planner_ = rclcpp_action::create_client<MoveGroup>(this, "move_action");
  ik_ = create_client<GetPositionIK>("compute_ik");
  follower_ = rclcpp_action::create_client<FollowJointTrajectory>(this, driver("follow_joint_trajectory"));
  joint_command_ = rclcpp_action::create_client<JointCommand>(this, driver("robot_joint"));
  parameter_check_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) { return check_change(parameters); });
}

std::string MoveItExecutor::driver(const std::string & relative) const {
  return under("driver_namespace", relative);
}

std::string MoveItExecutor::target(const std::string & relative) const {
  return under("target_namespace", relative);
}

std::string MoveItExecutor::under(const std::string & parameter, const std::string & relative) const {
  std::string ns = get_parameter(parameter).as_string();
  while (!ns.empty() && ns.front() == '/') ns.erase(ns.begin());
  while (!ns.empty() && ns.back() == '/') ns.pop_back();
  return (ns.empty() ? "" : "/" + ns) + "/" + relative;
}

void MoveItExecutor::on_joints(const sensor_msgs::msg::JointState & msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) positions_[msg.name[i]] = msg.position[i];
  received_ = std::chrono::steady_clock::now();
}

void MoveItExecutor::on_target(const std::string & arm, const std_msgs::msg::Float64MultiArray & msg) {
  Eigen::Matrix4d target;
  try {
    target = check_transform(msg.data);
  } catch (const std::invalid_argument & error) {
    report(std::string("FAILED: rejected target: ") + error.what(), {{arm, Eigen::Matrix4d::Identity()}});
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Only an arm's latest target counts. MoveIt plans from standing arms, so one that
    // arrives during a move runs when the move ends (cuMotion's executor replans at once).
    if (busy_) RCLCPP_INFO(get_logger(), "new target for %s: it runs when the current move ends", arm.c_str());
    pending_[arm] = target;
  }
  wake_.notify_one();
}

rcl_interfaces::msg::SetParametersResult MoveItExecutor::check_change(
  const std::vector<rclcpp::Parameter> & parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto range = kMotion.find(parameter.get_name());
    if (range == kMotion.end()) continue;
    const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                           ? static_cast<double>(parameter.as_int()) : parameter.as_double();
    if (!(value >= range->second.first && value <= range->second.second)) {
      result.successful = false;
      result.reason = parameter.get_name() + " is out of range: " + std::to_string(value);
    }
  }
  return result;
}

void MoveItExecutor::report(const std::string & text, const Targets & targets) {
  std_msgs::msg::String msg;
  msg.data = text;
  std::string who;
  for (const auto & arm : arms_) {
    if (!targets.empty() && !targets.count(arm.name)) continue;
    arm.status->publish(msg);
    who += (who.empty() ? "" : ", ") + arm.name;
  }
  if (text.rfind("FAILED", 0) == 0) {
    RCLCPP_ERROR(get_logger(), "%s: %s", who.c_str(), text.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "%s: %s", who.c_str(), text.c_str());
  }
}

std::map<std::string, double> MoveItExecutor::fresh_positions(std::chrono::duration<double> timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  const auto since = std::chrono::steady_clock::now();
  const auto deadline = since + std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout);
  while (received_ <= since) {
    lock.unlock();
    if (std::chrono::steady_clock::now() > deadline || !rclcpp::ok()) {
      throw std::runtime_error("no joint_overall_states from the driver -- is the RB-Y1 driver running "
                               "(ros2 launch rby1_driver rby1_ros2_driver.launch.py)?");
    }
    std::this_thread::sleep_for(10ms);
    lock.lock();
  }
  return positions_;
}

std::string MoveItExecutor::switch_driver(const std::string & service, bool on, const std::string & parameters) {
  auto client = create_client<StateOnOff>(driver(service));
  if (!client->wait_for_service(30s)) {
    throw std::runtime_error("Driver service unavailable: " + driver(service) + " -- start the driver "
                             "(ros2 launch rby1_driver rby1_ros2_driver.launch.py) on the same ROS_DOMAIN_ID");
  }
  auto request = std::make_shared<StateOnOff::Request>();
  request->state = on;
  request->parameters = parameters;
  const auto response = wait(client->async_send_request(request), 30s, driver(service));
  if (!response->success) throw std::runtime_error(service + ": " + response->message);
  return response->message;
}

// Switch the stream off after a move -- only if this executor switched it on.
void MoveItExecutor::close_stream() {
  if (!owns_stream_) return;
  try {
    switch_driver("stream_control", false, kStreamChannel);
  } catch (const std::exception & error) {  // the driver may already have dropped it
    RCLCPP_WARN(get_logger(), "stream_control off: %s", error.what());
  }
  stream_closed_ = std::chrono::steady_clock::now();
  owns_stream_ = false;
}

// The arms with a target in joint impedance, the other in position, when that changed
// since the last move. Never enabled: the driver's own setting (fjt_impedance_parts) stands.
void MoveItExecutor::apply_impedance(const Targets & targets) {
  const bool enabled = param<bool>("impedance.enabled");
  const bool right = enabled && targets.count("right_arm"), left = enabled && targets.count("left_arm");
  const std::vector<double> key{right ? 1.0 : 0.0, left ? 1.0 : 0.0, param<double>("impedance.stiffness"),
                                param<double>("impedance.damping_ratio"), param<double>("impedance.torque_limit")};
  if (applied_impedance_ == key || (!applied_impedance_ && !enabled)) return;
  auto client = create_client<rby1_msgs::srv::SetTrajectoryImpedance>(driver("set_trajectory_impedance"));
  if (!client->wait_for_service(10s)) {
    throw std::runtime_error("Driver service unavailable: " + driver("set_trajectory_impedance") +
                             " -- rebuild the driver (it needs joint impedance for follow_joint_trajectory)");
  }
  auto request = std::make_shared<rby1_msgs::srv::SetTrajectoryImpedance::Request>();
  request->state = {false, right, left};  // torso, right_arm, left_arm
  if (right) request->right_arm_stiffness = std::vector<double>(7, key[2]);
  if (left) request->left_arm_stiffness = std::vector<double>(7, key[2]);
  request->damping_ratio = {key[3]};
  request->torque_limit = {key[4]};
  const auto response = wait(client->async_send_request(request), 10s, driver("set_trajectory_impedance"));
  if (!response->success) throw std::runtime_error("set_trajectory_impedance: " + response->message);
  applied_impedance_ = key;
  RCLCPP_INFO(get_logger(), "trajectories in joint %s",
              right && left ? "impedance" : right ? "impedance (right arm)" : left ? "impedance (left arm)" : "position");
}

void MoveItExecutor::move_to_ready(const std::vector<std::string> & parts) {
  if (!joint_command_->wait_for_action_server(30s)) {
    throw std::runtime_error("Driver action unavailable: " + driver("robot_joint") + " -- is the driver running?");
  }
  JointCommand::Goal goal;
  for (const auto & part : parts) {
    auto & command = part == "right_arm" ? goal.right_arm : part == "left_arm" ? goal.left_arm : goal.torso;
    command.position = ready_.at(part);
    command.minimum_time = param<double>("ready_time");
  }
  const auto handle = wait(joint_command_->async_send_goal(goal), 10s, "robot_joint");
  if (!handle) throw std::runtime_error("Driver rejected the ready-pose command");
  const auto result = wait(joint_command_->async_get_result(handle),
                           std::chrono::duration<double>(param<double>("ready_time") + 30.0), "robot_joint");
  if (!result.result->success) throw std::runtime_error("Ready-pose move failed: " + result.result->finish_code);
}

void MoveItExecutor::start() {
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (robot_state_) break;
    }
    if (std::chrono::steady_clock::now() > deadline || !rclcpp::ok()) {
      throw std::runtime_error("No robot_state from the driver within 15 s -- is the RB-Y1 driver running "
                               "(ros2 launch rby1_driver rby1_ros2_driver.launch.py)?");
    }
    std::this_thread::sleep_for(50ms);
  }
  rby1_msgs::msg::RobotState state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = *robot_state_;
  }
  if (const auto problem = robot_problem(state, driver("control_manager_command"))) throw std::runtime_error(*problem);
  if (param<bool>("enable_robot")) {
    switch_driver("robot_power", true);
    switch_driver("robot_servo", true);
    RCLCPP_INFO(get_logger(), "power and servos on");
  }
  if (param<bool>("ready_if_straight")) {
    const auto parts = straight_parts(fresh_positions(), ready_, param<double>("straight_elbow"));
    if (!parts.empty()) {
      std::string names;
      for (const auto & part : parts) names += (names.empty() ? "" : ", ") + part;
      RCLCPP_INFO(get_logger(), "straight: %s -- moving them to the ready pose together", names.c_str());
      move_to_ready(parts);
    }
  }
  if (!follower_->wait_for_action_server(30s)) {
    throw std::runtime_error("Driver action unavailable: " + driver("follow_joint_trajectory"));
  }
  if (!planner_->wait_for_action_server(60s)) {
    throw std::runtime_error("move_group (move_action) did not come up within 60 s -- see its log above");
  }
  for (const auto & arm : arms_) {
    RCLCPP_INFO(get_logger(), "waiting for 4x4 targets on %s (tool=%s, frame=%s)", arm.targets->get_topic_name(),
                arm.tool.c_str(), param<std::string>("frame").c_str());
  }
  report("READY");
}

// The arm's joints for the tool at `target`. A 7-joint arm reaches a pose in many
// shapes, and a planner given the pose alone may end in one half a turn of a joint
// away from where the arm is. So MoveIt's IK is asked from the arm's posture, and when
// that fails or turns a joint far, from random ones too; the solution the arm turns
// least to reach is taken.
std::optional<std::vector<double>> MoveItExecutor::joint_goal(const Arm & arm, const Eigen::Matrix4d & target,
                                                              const std::vector<double> & current) {
  if (!ik_->wait_for_service(10s)) throw std::runtime_error("move_group's compute_ik service is unavailable");
  auto request = std::make_shared<GetPositionIK::Request>();
  auto & ik = request->ik_request;
  ik.group_name = arm.name;  // the arm's own group: it has the IK solver
  ik.ik_link_name = arm.tool;
  ik.avoid_collisions = true;
  ik.timeout = rclcpp::Duration::from_seconds(kIkTime);
  ik.robot_state.is_diff = true;  // the other joints: where the robot is
  ik.robot_state.joint_state.name = arm.joints;
  ik.pose_stamped.header.frame_id = param<std::string>("frame");
  auto & pose = ik.pose_stamped.pose;
  pose.position.x = target(0, 3);
  pose.position.y = target(1, 3);
  pose.position.z = target(2, 3);
  const Eigen::Quaterniond q(Eigen::Matrix3d(target.topLeftCorner<3, 3>()));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  std::vector<std::vector<double>> solutions;
  for (int seed = 0; seed <= kIkSeeds; ++seed) {
    ik.robot_state.joint_state.position = current;
    if (seed > 0) {
      for (size_t i = 0; i < arm.joints.size(); ++i) {
        const auto limits = bounds_.find(arm.joints[i]);
        if (limits == bounds_.end()) continue;
        ik.robot_state.joint_state.position[i] =
          std::uniform_real_distribution<double>(limits->second.first, limits->second.second)(random_);
      }
    }
    const auto response = wait(ik_->async_send_request(request), 10s, "compute_ik");
    if (response->error_code.val != MoveItErrorCodes::SUCCESS) continue;
    const auto & state = response->solution.joint_state;
    std::vector<double> solution;
    for (const auto & joint : arm.joints) {
      const auto found = std::find(state.name.begin(), state.name.end(), joint);
      if (found != state.name.end()) solution.push_back(state.position[found - state.name.begin()]);
    }
    if (solution.size() == arm.joints.size()) solutions.push_back(solution);
    // From the arm's own posture IK lands on the nearest shape when the target is
    // near (40 of 40 targets within 7 cm): the random starts are for when it did not.
    if (seed == 0 && !solutions.empty() && largest_turn(current, solutions[0]) <= kNearTurn) break;
  }
  if (solutions.empty()) {
    // The planner's own goal sampling reaches poses this IK does not (it tries far more
    // starts, within the goal's tolerances), so the target goes to it as a pose.
    RCLCPP_WARN(get_logger(), "%s: IK found no joints for the target from %d starts: planning to the pose instead "
                "-- the arm may end in another shape than the nearest", arm.name.c_str(), kIkSeeds + 1);
    return std::nullopt;
  }
  const auto & best = solutions[least_motion(current, solutions)];
  RCLCPP_INFO(get_logger(), "%s: joint goal: the arm turns at most %.3f rad (the least of %zu solutions)",
              arm.name.c_str(), largest_turn(current, best), solutions.size());
  return best;
}

// The arm's tool at the pose `target`, added to `constraints`: for a target IK found no
// joints for (joint_goal).
void MoveItExecutor::pose_goal(const Arm & arm, const Eigen::Matrix4d & target,
                               moveit_msgs::msg::Constraints & constraints) {
  const auto frame = param<std::string>("frame");
  moveit_msgs::msg::PositionConstraint position;
  position.header.frame_id = frame;
  position.link_name = arm.tool;
  position.weight = 1.0;
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions.push_back(param<double>("position_tolerance"));
  geometry_msgs::msg::Pose where;
  where.position.x = target(0, 3);
  where.position.y = target(1, 3);
  where.position.z = target(2, 3);
  where.orientation.w = 1.0;
  position.constraint_region.primitives.push_back(sphere);
  position.constraint_region.primitive_poses.push_back(where);

  moveit_msgs::msg::OrientationConstraint orientation;
  orientation.header.frame_id = frame;
  orientation.link_name = arm.tool;
  orientation.weight = 1.0;
  const Eigen::Quaterniond q(Eigen::Matrix3d(target.topLeftCorner<3, 3>()));
  orientation.orientation.x = q.x();
  orientation.orientation.y = q.y();
  orientation.orientation.z = q.z();
  orientation.orientation.w = q.w();
  orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance =
    orientation.absolute_z_axis_tolerance = param<double>("orientation_tolerance");
  constraints.position_constraints.push_back(position);
  constraints.orientation_constraints.push_back(orientation);
}

// The path for `targets` and how long planning took; nothing when the arms are already
// there. Each arm is planned to its joints for its target (joint_goal). One arm's target
// is planned in that arm's group, so the other arm is not the planner's to move: planned
// as both arms with the other to end where it is, it swung away and back around an
// obstacle (up to 1.57 rad, 29 of 30 plans around a 6 cm box).
std::optional<std::pair<JointTrajectory, double>> MoveItExecutor::plan(const Targets & targets) {
  MoveGroup::Goal goal;
  auto & request = goal.request;
  request.group_name = targets.size() == arms_.size() ? kBothArms : targets.begin()->first;
  request.pipeline_id = param<std::string>("pipeline");
  request.planner_id = param<std::string>("planner_id");
  request.num_planning_attempts = static_cast<int32_t>(param<int64_t>("planning_attempts"));
  request.allowed_planning_time = param<double>("planning_time");
  request.max_velocity_scaling_factor = param<double>("velocity_scaling");
  request.max_acceleration_scaling_factor = param<double>("acceleration_scaling");
  request.start_state.is_diff = true;
  goal.planning_options.plan_only = true;

  const auto positions = fresh_positions();
  moveit_msgs::msg::Constraints constraints;
  bool there = true;  // every arm's goal is where the arm is
  for (const auto & arm : arms_) {
    const auto target = targets.find(arm.name);
    if (target == targets.end()) continue;
    std::vector<double> current;
    for (const auto & joint : arm.joints) current.push_back(positions.at(joint));
    const auto joints = joint_goal(arm, target->second, current);
    if (!joints) {
      pose_goal(arm, target->second, constraints);
      there = false;
      continue;
    }
    there = there && largest_turn(current, *joints) <= kAlreadyThere;
    for (size_t i = 0; i < arm.joints.size(); ++i) {
      moveit_msgs::msg::JointConstraint joint;
      joint.joint_name = arm.joints[i];
      joint.position = (*joints)[i];
      joint.tolerance_above = joint.tolerance_below = kJointTolerance;
      joint.weight = 1.0;
      constraints.joint_constraints.push_back(joint);
    }
  }
  request.goal_constraints.push_back(constraints);
  rclcpp_action::ClientGoalHandle<MoveGroup>::WrappedResult result;
  for (int attempt = 1;; ++attempt) {
    const auto handle = wait(planner_->async_send_goal(goal), 10s, "move_group");
    if (!handle) throw std::runtime_error("move_group rejected the planning request");
    result = wait(planner_->async_get_result(handle),
                  std::chrono::duration<double>(param<double>("planning_time") + 30.0), "move_group");
    // A path can cut a corner between the states the planner checked; MoveIt's own
    // check of the result then refuses it. Planning again gives another path.
    if (result.result->error_code.val != MoveItErrorCodes::INVALID_MOTION_PLAN || attempt == kPlanAttempts) break;
    RCLCPP_WARN(get_logger(), "MoveIt refused the path it planned (attempt %d of %d): planning again", attempt,
                kPlanAttempts);
  }
  if (result.result->error_code.val != MoveItErrorCodes::SUCCESS) {
    throw std::runtime_error(planning_failure(result.result->error_code.val));
  }
  const auto & path = result.result->planned_trajectory.joint_trajectory;
  if (path.points.size() < 2) {
    // MoveIt answers a goal its start already meets with an empty trajectory.
    if (there) return std::nullopt;
    throw std::runtime_error("move_group returned an empty trajectory -- the arm may already be there");
  }
  return std::make_pair(path, result.result->planning_time);
}

void MoveItExecutor::execute(const JointTrajectory & trajectory, const Targets & targets) {
  const auto sent = with_hold(trajectory, param<double>("hold"), param<double>("step"));
  apply_impedance(targets);
  std::this_thread::sleep_until(
    stream_closed_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(kStreamReopenGap)));
  // The arms' stream channel only. One that was open already -- someone else's --
  // stays open after this move: the driver says which it opened.
  owns_stream_ = switch_driver("stream_control", true, kStreamChannel).rfind("Stream channels opened", 0) == 0;
  try {
    FollowJointTrajectory::Goal goal;
    goal.trajectory = sent;
    const auto handle = wait(follower_->async_send_goal(goal), 10s, "follow_joint_trajectory");
    if (!handle) {
      throw std::runtime_error("Driver rejected the trajectory -- ros2_control may be holding the robot (stop any "
                               "rby1_moveit_* demo.launch.py), or its start is too far from where the robot is "
                               "(the driver log says which)");
    }
    const auto result = wait(follower_->async_get_result(handle),
                             std::chrono::duration<double>(seconds(sent.points.back().time_from_start) + 30.0),
                             "follow_joint_trajectory");
    if (result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
      throw std::runtime_error("Driver aborted the trajectory (" + std::to_string(result.result->error_code) +
                               "): " + result.result->error_string);
    }
  } catch (...) {
    close_stream();
    throw;
  }
  close_stream();

  const auto & final = trajectory.points.back().positions;
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (true) {
    const auto current = fresh_positions();
    std::ostringstream off;
    for (size_t j = 0; j < trajectory.joint_names.size(); ++j) {
      const auto found = current.find(trajectory.joint_names[j]);
      if (found == current.end() || std::abs(found->second - final[j]) > param<double>("endpoint_tolerance")) {
        off << (off.tellp() > 0 ? ", " : "") << "'" << trajectory.joint_names[j] << "': "
            << (found == current.end() ? std::string("missing") : round2(found->second - final[j], "%.4f"));
      }
    }
    if (off.tellp() == 0) return;
    if (std::chrono::steady_clock::now() > deadline) {
      throw std::runtime_error("Driver reported success but joints stopped short of the endpoint (rad): {" +
                               off.str() + "}. " +
                               (param<bool>("impedance.enabled")
                                  ? "In joint impedance the arm yields: raise impedance.stiffness or "
                                    "endpoint_tolerance"
                                  : "Send the target again, slower (velocity_scaling, minimum_time)"));
    }
  }
}

void MoveItExecutor::process(const Targets & targets) {
  try {
    report("PLANNING", targets);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (robot_state_) {
        if (const auto problem = robot_problem(*robot_state_, driver("control_manager_command"))) throw std::runtime_error(*problem);
      }
    }
    const auto planned = plan(targets);
    if (!planned) {
      RCLCPP_INFO(get_logger(), "at the target already: nothing to move");
      report("DONE", targets);
      return;
    }
    const auto & [path, planning_time] = *planned;
    const auto timing = move_duration(seconds(path.points.back().time_from_start), param<double>("duration"),
                                      param<double>("minimum_time"));
    const auto trajectory = retime(path, timing.duration, param<double>("step"), bounds_);
    const auto [load, joint] = peak_speed_ratio(trajectory, velocity_limits_);
    if (load > 1.0) {
      throw std::runtime_error(round2(timing.duration) + " s is too fast: " + joint + " would reach " + round2(load) +
                               "x its velocity limit. Use at least " + round2(timing.duration * load) +
                               " s (duration), or 0 to let MoveIt time it");
    }
    report("EXECUTING planner_time=" + round2(planning_time, "%.3f") + "s duration=" + round2(timing.duration) +
           "s (" + timing.reason + ") steps=" + std::to_string(trajectory.points.size()) +
           " peak_velocity=" + round2(load * 100, "%.0f") + "%", targets);
    execute(trajectory, targets);
    report("DONE", targets);
  } catch (const std::exception & error) {
    report(std::string("FAILED: ") + error.what(), targets);
  }
}

void MoveItExecutor::run() {
  start();
  while (rclcpp::ok()) {
    Targets targets;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock, 100ms, [this] { return !pending_.empty(); });
      if (pending_.empty()) continue;
      // Targets sent to both arms together arrive one after the other.
      wake_.wait_for(lock, std::chrono::duration<double>(kGather), [this] { return pending_.size() == arms_.size(); });
      targets.swap(pending_);  // both arms' when both wait: they go in one move
      busy_ = true;
    }
    process(targets);
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
  }
}

}  // namespace rby1_moveit_executor
