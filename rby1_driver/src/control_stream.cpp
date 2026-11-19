// Command streams of the RB-Y1 driver: the five channels, follow_joint_trajectory,
// stream_joint, stream_cartesian, cmd_vel and the trajectory impedance.
// Compiled as part of rby1_ros2_driver.cpp, which includes this file: the driver is a
// class template instantiated once per robot model there, not here.
#include "rby1_ros2_driver.hpp"
#include <sstream>

namespace rby1_ros2 {

// --- FollowJointTrajectory Handlers ---
template <typename ModelType>
int RBY1_ROS2_DRIVER<ModelType>::fjt_parts_running() const {
  int parts = 0;
  for (const auto &[part, goal] : fjt_goals_)
    if (goal && goal->is_active())
      parts |= part;
  return parts;
}

template <typename ModelType>
int RBY1_ROS2_DRIVER<ModelType>::fjt_parts_of(const FjtGoal &goal) const {
  int parts = 0;
  for (const auto &[part, running] : fjt_goals_)
    if (running == goal)
      parts |= part;
  return parts;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::fjt_release(const FjtGoal &goal) {
  for (auto it = fjt_goals_.begin(); it != fjt_goals_.end();)
    it = it->second == goal ? fjt_goals_.erase(it) : std::next(it);
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::fjt_abort_overlapping(int parts,
                                                        const std::string &why) {
  std::vector<FjtGoal> ending;
  for (const auto &[part, goal] : fjt_goals_)
    if ((part & parts) && goal &&
        std::find(ending.begin(), ending.end(), goal) == ending.end())
      ending.push_back(goal);
  for (const auto &goal : ending) {
    fjt_release(goal);  // its thread sees it owns no part and sends nothing more
    if (!goal->is_active())
      continue;
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = why;
    try {
      goal->abort(result);
    } catch (...) {
    }
  }
}

template <typename ModelType>
rclcpp_action::GoalResponse
RBY1_ROS2_DRIVER<ModelType>::handle_follow_joint_trajectory_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received FollowJointTrajectory request");
  (void)uuid;
  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting FollowJointTrajectory: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // An invalid trajectory means the sender's idea of the robot -- its posture, or
  // which channels are open -- has gone wrong. It is refused, and every trajectory
  // running is stopped as well (user decision 2026-10-29: when in doubt, stop); the
  // robot holds the last waypoint each was given.
  std::string refused;
  if (goal->trajectory.points.empty()) {
    refused = "Trajectory has no points.";
  } else {
    const int named = trajectory_parts(goal->trajectory);
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (!stream_active_)
      refused = "Stream control is not active: open the channels of the parts this "
                "trajectory names with stream_control first.";
    else if (named == 0)
      refused = "The trajectory names no joint of the torso, the arms or the head.";
    else if (const int closed = channels_of_parts(named) & ~open_stream_channels())
      refused = "Stream channel '" + stream_channel_names(closed) +
                "' is not open: open it with stream_control (parameters: " +
                stream_channel_names(closed) + ") and send the trajectory again.";
  }
  if (refused.empty()) {
    // A start the robot cannot reach in time: the sender's posture is wrong.
    if (auto problem = start_problem(goal->trajectory))
      refused = problem.value();
  }
  if (!refused.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int running = fjt_parts_running();
    RCLCPP_ERROR(this->get_logger(),
                 "\033[1;31m[SAFETY] Rejecting FollowJointTrajectory%s: %s\033[0m",
                 running ? " and stopping every trajectory running" : "",
                 refused.c_str());
    fjt_abort_overlapping(STREAM_TORSO | STREAM_RIGHT_ARM | STREAM_LEFT_ARM | STREAM_HEAD,
                          "Stopped: a trajectory sent to the driver was rejected (" +
                              refused + ")");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// The first waypoint must be reachable from the measured posture in the time the
// trajectory gives it: the implied speed (difference / time to the first
// waypoint) may not exceed a joint's velocity limit (times kFjtStartVelocityScale,
// 1.0). A far start is fine if it is given enough time; a near one is not if it
// must happen at once.
template <typename ModelType>
std::optional<std::string> RBY1_ROS2_DRIVER<ModelType>::start_problem(
    const trajectory_msgs::msg::JointTrajectory &trajectory) {
  if (!dynamics_ || !dyn_state_ || qdot_upper_.size() == 0)
    return std::nullopt;
  const auto &first = trajectory.points.front();
  double t0 = first.time_from_start.sec + first.time_from_start.nanosec * 1e-9;
  if (t0 <= 0.0)
    t0 = kCommandTimeFloor;
  auto state = get_state_with_retry();
  for (size_t j = 0; j < trajectory.joint_names.size() && j < first.positions.size(); ++j) {
    const auto &name = trajectory.joint_names[j];
    int state_idx = -1;
    for (size_t k = 0; k < info_.joint_infos.size(); ++k) {
      if (info_.joint_infos[k].name == name) {
        state_idx = static_cast<int>(k);
        break;
      }
    }
    int dyn_idx = -1;
    for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
      if (dyn_joint_names_[k] == name) {
        dyn_idx = static_cast<int>(k);
        break;
      }
    }
    if (state_idx < 0 || dyn_idx < 0)
      continue;
    const double current = state.position[state_idx];
    const double needed = std::abs(first.positions[j] - current) / t0;
    const double limit = qdot_upper_[dyn_idx] * kFjtStartVelocityScale;
    if (needed > limit) {
      std::ostringstream reason;
      reason << "start too far from the robot: joint '" << name << "' is at "
             << current << " rad, the first waypoint " << first.positions[j]
             << " rad in " << t0 << " s needs " << needed << " rad/s > "
             << limit << " rad/s";
      return reason.str();
    }
  }
  return std::nullopt;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_follow_joint_trajectory_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "Received request to cancel FollowJointTrajectory goal");
  // Only the streams this trajectory is on. robot_->CancelControl() ends every
  // stream of the robot, so a head tracker's stream went with a cancelled arm
  // trajectory and did not come back (simulator: the head stood still after 10 of
  // 10 cancels).
  int parts = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    parts = fjt_parts_of(goal_handle);
  }
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    // A cancelled stream is dead: the next command on it is refused as expired, and
    // a stream opened at once in its place comes up expired too (simulator
    // 2026-10-26: 17 reopenings, one per waypoint, for 0.85 s after a cancel). So it
    // is marked, and reopened kStreamReopenGap later by the next command.
    auto cancel = [](BodyStream &stream) {
      if (!stream.handler)
        return;
      try {
        stream.handler->Cancel();
      } catch (...) {
      }
      stream.expired_at = std::chrono::steady_clock::now();
    };
    if (parts & STREAM_RIGHT_ARM)
      cancel(right_arm_stream_);
    if (parts & STREAM_LEFT_ARM)
      cancel(left_arm_stream_);
    if (parts & STREAM_TORSO)
      cancel(torso_stream_);
    if (parts & STREAM_HEAD)
      cancel(head_stream_);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_follow_joint_trajectory_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>
        goal_handle) {
  using namespace std::placeholders;
  RCLCPP_INFO(
      this->get_logger(),
      "FollowJointTrajectory Goal accepted. Detaching execution thread...");

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int parts = trajectory_parts(goal_handle->get_goal()->trajectory);
    fjt_abort_overlapping(parts, "Preempted by a new trajectory on the same part");
    for (int part : {STREAM_TORSO, STREAM_RIGHT_ARM, STREAM_LEFT_ARM, STREAM_HEAD})
      if (parts & part)
        fjt_goals_[part] = goal_handle;
  }

  std::thread{
      std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_follow_joint_trajectory,
                this, _1),
      goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_follow_joint_trajectory(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>
        goal_handle) {
  has_printed_collision_log_ = false;
  // Whatever way this ends, the goal's parts are free again.
  struct Release {
    RBY1_ROS2_DRIVER *self;
    FjtGoal goal;
    ~Release() {
      std::lock_guard<std::mutex> lock(self->mutex_);
      self->fjt_release(goal);
    }
  } release{this, goal_handle};

  auto result = std::make_shared<FollowJointTrajectory::Result>();

  // The parts the trajectory names go out each on its own channel, which must be
  // open: the driver does not open a channel for a command (2026-10-26 user
  // decision), the client opens the channels it will use with stream_control.
  const int named = trajectory_parts(goal_handle->get_goal()->trajectory);
  const bool head_commanded = named & STREAM_HEAD;
  int parts = 0;
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    std::string refused;
    if (!stream_active_)
      refused = "Stream control is not active: open the channels of the parts this "
                "trajectory names with stream_control first.";
    else if (named == 0)
      refused = "The trajectory names no joint of the torso, the arms or the head.";
    else if (const int closed = channels_of_parts(named) & ~open_stream_channels())
      refused = "Stream channel '" + stream_channel_names(closed) +
                "' is not open: open it with stream_control (parameters: " +
                stream_channel_names(closed) + ") and send the trajectory again.";
    if (!refused.empty()) {
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      result->error_string = refused;
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[CONTROL REJECTED] FJT rejected: %s\033[0m",
                  refused.c_str());
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
    parts = named;
  }

  is_control_canceled_ = false;

  // Mark the commanded body parts as controlled.
  std::vector<size_t> fjt_parts;
  if (parts & STREAM_TORSO)
    fjt_parts.push_back(PART_TORSO);
  if (parts & STREAM_RIGHT_ARM)
    fjt_parts.push_back(PART_RIGHT_ARM);
  if (parts & STREAM_LEFT_ARM)
    fjt_parts.push_back(PART_LEFT_ARM);
  if (head_commanded)
    fjt_parts.push_back(PART_HEAD);
  // In stream mode, set is_controlling_ flags
  for (auto p : fjt_parts)
    is_controlling_[p].store(true, std::memory_order_release);

  // RAII cleanup
  struct FjtGuard {
    std::atomic<bool> *flags;
    std::vector<size_t> parts;
    FjtGuard(std::atomic<bool> *f, std::vector<size_t> p)
        : flags(f), parts(std::move(p)) {}
    ~FjtGuard() {
      for (auto i : parts)
        flags[i].store(false, std::memory_order_release);
    }
  } fguard(is_controlling_, fjt_parts);

  RCLCPP_INFO(this->get_logger(), "Starting FollowJointTrajectory (stream=%s)",
              stream_active_ ? "ON" : "OFF");
  const auto goal = goal_handle->get_goal();
  const auto &trajectory = goal->trajectory;
  std::vector<int> joint_mapping(trajectory.joint_names.size(), -1);

  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    for (size_t j = 0; j < info_.joint_infos.size(); ++j) {
      if (trajectory.joint_names[i] == info_.joint_infos[j].name) {
        joint_mapping[i] = j;
        break;
      }
    }
  }

  // Joint limit checks across all waypoints
  if (dynamics_ && dyn_state_ && q_lower_.size() > 0) {
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &pt = trajectory.points[i];
      for (size_t j = 0; j < joint_mapping.size(); ++j) {
        if (joint_mapping[j] < 0 || j >= pt.positions.size())
          continue;
        double pos = pt.positions[j];
        std::string jname = info_.joint_infos[joint_mapping[j]].name;
        int dyn_idx = -1;
        for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
          if (dyn_joint_names_[k] == jname) {
            dyn_idx = k;
            break;
          }
        }
        if (dyn_idx < 0)
          continue;
        if (pos < q_lower_[dyn_idx] || pos > q_upper_[dyn_idx]) {
          result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
          result->error_string = "Limit exceeded: joint '" + jname +
                                 "' = " + std::to_string(pos) + " out of [" +
                                 std::to_string(q_lower_[dyn_idx]) + ", " +
                                 std::to_string(q_upper_[dyn_idx]) +
                                 "] rad at waypoint " + std::to_string(i);
          RCLCPP_ERROR(this->get_logger(), "\033[1;31m[SAFETY] %s\033[0m",
                       result->error_string.c_str());
          try {
            goal_handle->abort(result);
          } catch (...) {
          }
          return;
        }
      }
    }
  }

  // Pre-collision check per waypoint -- disabled. It only ever logged a warning
  // (it never rejected anything) but read the robot state over gRPC once per
  // waypoint, which delayed the first command of every trajectory, and of every
  // trajectory that replaces a running one, by tens of milliseconds.
  /*
  // Pre-collision check per waypoint
  if (dynamics_ && dyn_state_) {
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &pt = trajectory.points[i];
      Eigen::Vector<double, ModelType::kRobotDOF> tq =
          Eigen::Vector<double, ModelType::kRobotDOF>::Zero();
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto cs = get_state_with_retry();
        for (int k = 0; k < (int)dyn_joint_names_.size(); ++k) {
          for (size_t jj = 0; jj < info_.joint_infos.size(); ++jj) {
            if (info_.joint_infos[jj].name == dyn_joint_names_[k]) {
              tq[k] = cs.position[jj];
              break;
            }
          }
        }
      }
      for (size_t j = 0; j < joint_mapping.size(); ++j) {
        if (joint_mapping[j] >= 0 && j < pt.positions.size()) {
          std::string jn = info_.joint_infos[joint_mapping[j]].name;
          for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
            if (dyn_joint_names_[k] == jn) {
              tq[k] = pt.positions[j];
              break;
            }
          }
        }
      }
      auto reason = get_predicted_collision_reason(tq);
      if (reason.has_value()) {
        RCLCPP_WARN(this->get_logger(),
                    "\033[1;33m[PREDICTIVE COLLISION WARNING] %s at waypoint "
                    "%zu\033[0m",
                    reason.value().c_str(), i);
      }
    }
  }
  */

  // An enabled control manager takes commands at once. Only one that had to be
  // reset or enabled needs time to settle, so a trajectory replacing a running
  // one continues without a pause.
  if (robot_->GetControlManagerState().state !=
      rb::ControlManagerState::State::kEnabled) {
    this->check_controll_manager();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  auto cur = get_state_with_retry();
  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(cur.position.data(),
                                                        cur.position.size());

  int failed_sends = 0;         // consecutive waypoints the stream refused
  try {
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &pt = trajectory.points[i];

      // Preemption / cancel check
      {
        std::lock_guard<std::mutex> lk(mutex_);
        if (fjt_parts_of(goal_handle) == 0)  // replaced or stopped: send nothing more
          return;
      }
      if (goal_handle->is_canceling() || is_control_canceled_ ||
          !rclcpp::ok()) {
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = "Canceled";
        try {
          goal_handle->canceled(result);
        } catch (...) {
        }
        return;
      }

      // Update q with this waypoint
      for (size_t j = 0; j < joint_mapping.size(); ++j) {
        if (joint_mapping[j] >= 0 && j < pt.positions.size())
          q[joint_mapping[j]] = pt.positions[j];
      }

      // Publish feedback
      auto fb = std::make_shared<FollowJointTrajectory::Feedback>();
      fb->joint_names = trajectory.joint_names;
      fb->desired = pt;
      try {
        goal_handle->publish_feedback(fb);
      } catch (...) {
      }

      double pt_time =
          (i > 0)
              ? (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 -
                 trajectory.points[i - 1].time_from_start.sec -
                 trajectory.points[i - 1].time_from_start.nanosec * 1e-9)
              : (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9);
      if (pt_time <= 0.0)
        pt_time = kCommandTimeFloor;

      // Build the waypoint and send it: the arms, the torso and the head each on
      // their own stream.
      {
        rb::ComponentBasedCommandBuilder head_comp;
        BodyCommand body;
        body.parts = parts & ~STREAM_HEAD;

        auto part_q = [&](const std::vector<unsigned int> &idx_vec) {
          Eigen::VectorXd pq(idx_vec.size());
          for (size_t k = 0; k < idx_vec.size(); ++k)
            pq[k] = q[idx_vec[k]];
          return pq;
        };
        // A body part in joint impedance (set_trajectory_impedance) or, by
        // default, in joint position.
        auto body_part = [&](const std::vector<unsigned int> &idx_vec,
                             bool impedance,
                             const std::vector<double> &stiffness,
                             auto &&set) {
          const Eigen::VectorXd pq = part_q(idx_vec);
          if (impedance) {
            Eigen::VectorXd k = Eigen::VectorXd::Constant(
                pq.size(), kDefaultJointStiffness);
            if (stiffness.size() == static_cast<size_t>(pq.size()))
              k = Eigen::Map<const Eigen::VectorXd>(stiffness.data(),
                                                    stiffness.size());
            rb::JointImpedanceControlCommandBuilder b;
            make_joint_impedance_builder(b, pq, pt_time, kControlHoldTime, k,
                                         trajectory_damping_ratio_,
                                         trajectory_torque_limit_, 0.0, 0.0);
            set(b);
          } else {
            rb::JointPositionCommandBuilder b;
            make_joint_pos_builder(b, pq, pt_time, kControlHoldTime);
            set(b);
          }
        };
        if (parts & STREAM_TORSO)
          body_part(info_.torso_joint_idx, trajectory_impedance_enabled_torso_,
                    trajectory_stiffness_torso_, [&](const auto &b) {
                      body.torso.SetTorsoCommand(rb::TorsoCommandBuilder(b));
                    });
        if (parts & STREAM_RIGHT_ARM)
          body_part(info_.right_arm_joint_idx,
                    trajectory_impedance_enabled_right_arm_,
                    trajectory_stiffness_right_arm_, [&](const auto &b) {
                      body.right_arm.SetRightArmCommand(rb::ArmCommandBuilder(b));
                    });
        if (parts & STREAM_LEFT_ARM)
          body_part(info_.left_arm_joint_idx,
                    trajectory_impedance_enabled_left_arm_,
                    trajectory_stiffness_left_arm_, [&](const auto &b) {
                      body.left_arm.SetLeftArmCommand(rb::ArmCommandBuilder(b));
                    });
        if (head_commanded) {
          rb::JointPositionCommandBuilder b;
          make_joint_pos_builder(b, part_q(info_.head_joint_idx), pt_time, kControlHoldTime);
          head_comp.SetHeadCommand(rb::HeadCommandBuilder(b));
        }

        // A waypoint the robot did not get is counted, not passed over: a
        // trajectory it never received must not end as a success.
        try {
          std::lock_guard<std::mutex> sl(stream_mutex_);
          const std::string refused = send_body(body, pt_time, kStreamSendWait, false);
          if (!refused.empty())
            throw std::runtime_error(refused);
          if (head_commanded)
            send_head(head_comp);
          failed_sends = 0;
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "[FJT-Normal] SendCommand error: %s",
                       e.what());
          if (++failed_sends >= kFjtMaxFailedSends) {
            throw std::runtime_error(
                std::string("Could not send the trajectory to the robot: ") +
                e.what() +
                " -- the command stream failed. Switch stream_control off, "
                "wait a moment, switch it on and send again");
          }
        }
      }

      std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<long long>(pt_time * 1e6)));
    }

    // Final completion
    RCLCPP_INFO(this->get_logger(), "FollowJointTrajectory completed.");
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    try {
      goal_handle->succeed(result);
    } catch (...) {
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[FJT] Exception: %s", e.what());
    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
    result->error_string = e.what();
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  has_printed_collision_log_ = false;
  try {
    {
      std::lock_guard<std::mutex> lock(stream_mutex_);
      if (hardware_control_active_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "cmd_vel received but hardware control is active. "
                             "Ignoring command.");
        return;
      }
      if (!mobility_stream_.handler) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "cmd_vel received but stream channel 'mobile' is not "
                             "open: open it with stream_control (parameters: "
                             "mobile). Ignoring command.");
        return;
      }
    }

    double hz = params_.stream_command_hz;
    if (hz <= 0.0)
      hz = 15.0;
    double minimum_time = (1.0 / hz) * 1.1;

    rb::ComponentBasedCommandBuilder comp_builder;
    rb::MobilityCommandBuilder mobility_cmd;
    build_mobility_cmd(mobility_cmd, msg->linear.x, msg->linear.y,
                       msg->angular.z, minimum_time);
    comp_builder.SetMobilityCommand(mobility_cmd);

    rb::RobotCommandBuilder robot_cmd;
    robot_cmd.SetCommand(comp_builder);

    std::lock_guard<std::mutex> lock(stream_mutex_);
    if (mobility_stream_.handler) {
      mobility_stream_.handler->SendCommand(robot_cmd);
      mobility_stream_.commanded = true;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in cmd_vel_callback: %s",
                 e.what());
    deactivate_all_streams();
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::stream_control_callback(
    const std::shared_ptr<rby1_msgs::srv::StateOnOff::Request> request,
    std::shared_ptr<rby1_msgs::srv::StateOnOff::Response> response) {
  std::lock_guard<std::mutex> lock(mutex_);
  const int channels = parse_stream_channels(request->parameters);
  if (channels == 0) {
    response->success = false;
    response->message = "Unknown stream channel in '" + request->parameters +
                        "': name right_arm, left_arm, torso, mobile or head, "
                        "separated by commas; or all (also when empty).";
    RCLCPP_WARN(this->get_logger(), "stream_control: %s",
                response->message.c_str());
    return;
  }
  if (request->state) {
    bool is_gc_active = gravity_compensation_torso_ ||
                        gravity_compensation_right_arm_ ||
                        gravity_compensation_left_arm_;
    if (is_gc_active) {
      response->success = false;
      response->message =
          "Failed to activate stream control: Gravity compensation is active.";
      RCLCPP_WARN(this->get_logger(),
                  "stream_control: Rejecting activation request because "
                  "gravity compensation is active.");
      return;
    }

    this->get_parameter("stream_command_hz", params_.stream_command_hz);
    if (params_.stream_command_hz <= 0.0)
      params_.stream_command_hz = 15.0;

    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    const int already = channels & open_stream_channels();
    const int wanted = channels & ~already;
    if (wanted == 0) {
      response->success = true;
      response->message =
          "Stream channels already open: " + stream_channel_names(already) + ".";
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    const std::string refused = open_channels(wanted);
    if (!refused.empty()) {
      response->success = false;
      response->message = refused;
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    response->success = true;
    response->message = "Stream channels opened: " + stream_channel_names(wanted);
    if (already != 0)
      response->message += "; already open: " + stream_channel_names(already);
    response->message += ".";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  } else {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    const int closing = channels & open_stream_channels();
    close_stream_channels(channels, false);
    response->success = true;
    response->message =
        closing == 0 ? "No such stream channel was open."
                     : "Stream channels closed: " + stream_channel_names(closing) + ".";
    if (stream_active_)
      response->message +=
          " Still open: " + stream_channel_names(open_stream_channels()) + ".";
  }
}

// "right_arm, head" as channel bits; nothing or "all" as every channel; 0 for an
// unknown name.
template <typename ModelType>
int RBY1_ROS2_DRIVER<ModelType>::parse_stream_channels(
    const std::string &parameters) {
  static const std::pair<const char *, int> names[] = {
      {"right_arm", CHANNEL_RIGHT_ARM}, {"left_arm", CHANNEL_LEFT_ARM},
      {"torso", CHANNEL_TORSO},         {"mobile", CHANNEL_MOBILE},
      {"head", CHANNEL_HEAD},           {"all", CHANNEL_ALL},
      {".*", CHANNEL_ALL}};
  int channels = 0;
  std::stringstream ss(parameters);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token.erase(0, token.find_first_not_of(" \t"));
    token.erase(token.find_last_not_of(" \t") + 1);
    if (token.empty())
      continue;
    int bit = 0;
    for (const auto &[name, value] : names) {
      if (token == name)
        bit = value;
    }
    if (bit == 0)
      return 0;
    channels |= bit;
  }
  return channels == 0 ? static_cast<int>(CHANNEL_ALL) : channels;
}

template <typename ModelType>
std::string RBY1_ROS2_DRIVER<ModelType>::stream_channel_names(int channels) {
  std::string text;
  for (const auto &[name, bit] :
       {std::pair<const char *, int>{"right_arm", CHANNEL_RIGHT_ARM},
        {"left_arm", CHANNEL_LEFT_ARM}, {"torso", CHANNEL_TORSO},
        {"mobile", CHANNEL_MOBILE}, {"head", CHANNEL_HEAD}}) {
    if (channels & bit)
      text += std::string(text.empty() ? "" : ", ") + name;
  }
  return text;
}

// The channels (stream_channel bits) the parts (stream_part bits) go out on.
template <typename ModelType>
int RBY1_ROS2_DRIVER<ModelType>::channels_of_parts(int parts) {
  return ((parts & STREAM_RIGHT_ARM) ? CHANNEL_RIGHT_ARM : 0) |
         ((parts & STREAM_LEFT_ARM) ? CHANNEL_LEFT_ARM : 0) |
         ((parts & STREAM_TORSO) ? CHANNEL_TORSO : 0) |
         ((parts & STREAM_HEAD) ? CHANNEL_HEAD : 0);
}

template <typename ModelType>
typename RBY1_ROS2_DRIVER<ModelType>::BodyStream &
RBY1_ROS2_DRIVER<ModelType>::channel_stream(int channel) {
  switch (channel) {
  case CHANNEL_RIGHT_ARM:
    return right_arm_stream_;
  case CHANNEL_LEFT_ARM:
    return left_arm_stream_;
  case CHANNEL_TORSO:
    return torso_stream_;
  case CHANNEL_MOBILE:
    return mobility_stream_;
  case CHANNEL_HEAD:
    return head_stream_;
  default:
    throw std::invalid_argument("not one stream channel: " + std::to_string(channel));
  }
}

// Call with stream_mutex_ held.
template <typename ModelType>
int RBY1_ROS2_DRIVER<ModelType>::open_stream_channels() const {
  return (right_arm_stream_.handler ? CHANNEL_RIGHT_ARM : 0) |
         (left_arm_stream_.handler ? CHANNEL_LEFT_ARM : 0) |
         (torso_stream_.handler ? CHANNEL_TORSO : 0) |
         (mobility_stream_.handler ? CHANNEL_MOBILE : 0) |
         (head_stream_.handler ? CHANNEL_HEAD : 0);
}

// Opens the channels `wanted` names (stream_channel bits). Nothing is opened unless
// every one can be: a channel whose servo is off would take commands the robot
// never carries out. Call with stream_mutex_ held.
template <typename ModelType>
std::string RBY1_ROS2_DRIVER<ModelType>::open_channels(int wanted) {
  std::string blocked;
  for (const auto &[channel, servos] :
       {std::pair<int, const char *>{CHANNEL_RIGHT_ARM, "^right_arm_.*"},
        {CHANNEL_LEFT_ARM, "^left_arm_.*"}, {CHANNEL_TORSO, "^torso_.*"},
        {CHANNEL_MOBILE, ".*wheel.*"}, {CHANNEL_HEAD, "^head_.*"}}) {
    if ((wanted & channel) && !robot_->IsServoOn(servos))
      blocked += " " + stream_channel_names(channel) + " (its servo is off);";
  }
  if (!blocked.empty())
    return "Cannot open stream channel:" + blocked +
           " nothing was opened. Switch the servo on first.";
  try {
    // A stream opened right after one was closed comes up dead: it takes
    // commands without an error and the robot does not move (none at 0.3 s).
    std::this_thread::sleep_until(stream_closed_at_ + kStreamReopenGap);
    for (int channel : kStreamChannels) {
      if (!(wanted & channel))
        continue;
      BodyStream &stream = channel_stream(channel);
      stream = BodyStream{};
      stream.handler = robot_->CreateCommandStream(10);
      stream.opened_at = std::chrono::steady_clock::now();
      hold_current_pose(stream, channel);
    }
    stream_active_ = true;
    return "";
  } catch (const std::exception &e) {
    close_stream_channels(wanted, false);
    return std::string("Failed to activate stream control: ") + e.what();
  }
}

// The first command on a channel just opened: its part, held where it is measured
// (2026-10-26 user decision). The part is under the stream from the start, and a
// client's first command finds a stream that already carries it. In the kind the
// part's trajectory commands use -- joint impedance when set_trajectory_impedance
// switched it on -- since the robot does not take a new command kind on a stream.
// The mobile base has no pose to hold. Call with stream_mutex_ held.
template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::hold_current_pose(BodyStream &stream, int channel) {
  if (channel == CHANNEL_MOBILE)
    return;
  const std::vector<unsigned int> *joints = nullptr;
  int part = 0;
  bool impedance = false;
  const std::vector<double> *stiffness = nullptr;
  switch (channel) {
  case CHANNEL_RIGHT_ARM:
    joints = &info_.right_arm_joint_idx;
    part = STREAM_RIGHT_ARM;
    impedance = trajectory_impedance_enabled_right_arm_;
    stiffness = &trajectory_stiffness_right_arm_;
    break;
  case CHANNEL_LEFT_ARM:
    joints = &info_.left_arm_joint_idx;
    part = STREAM_LEFT_ARM;
    impedance = trajectory_impedance_enabled_left_arm_;
    stiffness = &trajectory_stiffness_left_arm_;
    break;
  case CHANNEL_TORSO:
    joints = &info_.torso_joint_idx;
    part = STREAM_TORSO;
    impedance = trajectory_impedance_enabled_torso_;
    stiffness = &trajectory_stiffness_torso_;
    break;
  case CHANNEL_HEAD:
    joints = &info_.head_joint_idx;
    part = STREAM_HEAD;
    break;
  }
  const auto state = robot_->GetState();
  Eigen::VectorXd q(joints->size());
  for (size_t i = 0; i < joints->size(); ++i)
    q[i] = state.position[(*joints)[i]];

  rb::ComponentBasedCommandBuilder component;
  if (channel == CHANNEL_HEAD) {
    // The head has joint position only.
    rb::JointPositionCommandBuilder b;
    make_joint_pos_builder(b, q, kDefaultMinimumTime, kControlHoldTime);
    component.SetHeadCommand(rb::HeadCommandBuilder(b));
  } else {
    rb::BodyComponentBasedCommandBuilder body;
    auto set = [&](const auto &b) {
      switch (channel) {
      case CHANNEL_RIGHT_ARM:
        body.SetRightArmCommand(rb::ArmCommandBuilder(b));
        break;
      case CHANNEL_LEFT_ARM:
        body.SetLeftArmCommand(rb::ArmCommandBuilder(b));
        break;
      default:
        body.SetTorsoCommand(rb::TorsoCommandBuilder(b));
        break;
      }
    };
    if (impedance) {
      Eigen::VectorXd k = Eigen::VectorXd::Constant(q.size(), kDefaultJointStiffness);
      if (stiffness->size() == static_cast<size_t>(q.size()))
        k = Eigen::Map<const Eigen::VectorXd>(stiffness->data(), stiffness->size());
      rb::JointImpedanceControlCommandBuilder b;
      make_joint_impedance_builder(b, q, kDefaultMinimumTime, kControlHoldTime, k,
                                   trajectory_damping_ratio_,
                                   trajectory_torque_limit_, 0.0, 0.0);
      set(b);
    } else {
      rb::JointPositionCommandBuilder b;
      make_joint_pos_builder(b, q, kDefaultMinimumTime, kControlHoldTime);
      set(b);
    }
    component.SetBodyCommand(rb::BodyCommandBuilder(body));
  }
  rb::RobotCommandBuilder robot_cmd;
  robot_cmd.SetCommand(component);
  stream.handler->SendCommand(robot_cmd);
  stream.parts = part;
  stream.one_shot = false;
}

// Closes these channels and frees their parts; `cancel` also stops what they were
// doing. Call with stream_mutex_ held.
template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::close_stream_channels(int channels,
                                                       bool cancel) {
  auto close = [this, cancel](auto &handler) {
    if (!handler)
      return;
    stream_closed_at_ = std::chrono::steady_clock::now();
    if (cancel) {
      try {
        handler->Cancel();
      } catch (...) {
      }
    }
    handler.reset();
  };
  auto release = [this](std::initializer_list<size_t> parts) {
    for (auto part : parts)
      is_controlling_[part].store(false, std::memory_order_release);
  };
  if (channels & CHANNEL_RIGHT_ARM) {
    close(right_arm_stream_.handler);
    release({PART_RIGHT_ARM});
  }
  if (channels & CHANNEL_LEFT_ARM) {
    close(left_arm_stream_.handler);
    release({PART_LEFT_ARM});
  }
  if (channels & CHANNEL_TORSO) {
    close(torso_stream_.handler);
    release({PART_TORSO});
  }
  if (channels & CHANNEL_MOBILE) {
    close(mobility_stream_.handler);
    release({PART_MOBILE});
  }
  if (channels & CHANNEL_HEAD) {
    close(head_stream_.handler);
    release({PART_HEAD});
  }
  stream_active_ = open_stream_channels() != 0;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::deactivate_all_streams() {
  std::lock_guard<std::mutex> stream_lock(stream_mutex_);
  if (stream_active_) {
    RCLCPP_WARN(this->get_logger(),
                "\033[1;31m[STREAM SAFETY WARNING] Stream failure detected! "
                "Deactivating all streams.\033[0m");
    close_stream_channels(CHANNEL_ALL, true);
  }
}

// Readies the arms' or the torso's stream for a command naming `parts` (stream_part
// bits) that moves for `busy_for` s. The robot ignores, on a command stream, the parts that
// stream's earlier commands did not name -- after head-only commands (a head tracker)
// it never moves the arms, after right-arm commands never the left -- so such a
// command goes out on a new stream. Closing the old stream ends the command on it,
// though, so while that command is still moving (inside its minimum_time, or its
// joints still turning) this waits for it, at most `wait_limit` s, and otherwise
// returns false: the caller refuses its command rather than cut the other one short.
// Call with stream_mutex_ held.
template <typename ModelType>
bool RBY1_ROS2_DRIVER<ModelType>::fit_stream_to(BodyStream &stream, int parts,
                                               double busy_for, double wait_limit,
                                               bool one_shot) {
  if (!stream.handler)
    return false;
  if ((stream.parts != 0 && (parts & ~stream.parts) != 0) ||
      stream.kind_changed) {
    // Busy: within the last command's minimum_time, or -- a one-shot command, whose
    // move can outlast its minimum_time -- its parts still moving.
    auto moving = [this, &stream] {
      if (!stream.one_shot)
        return false;
      try {
        const auto state = robot_->GetState();
        const std::pair<int, const std::vector<unsigned int> *> groups[] = {
            {STREAM_TORSO, &info_.torso_joint_idx},
            {STREAM_RIGHT_ARM, &info_.right_arm_joint_idx},
            {STREAM_LEFT_ARM, &info_.left_arm_joint_idx},
            {STREAM_HEAD, &info_.head_joint_idx}};
        for (const auto &[bit, joints] : groups) {
          if (!(stream.parts & bit))
            continue;
          for (auto idx : *joints) {
            if (idx < state.velocity.size() && std::abs(state.velocity[idx]) > 0.02)
              return true;
          }
        }
      } catch (const std::exception &) {
      }
      return false;
    };
    const auto give_up = std::chrono::steady_clock::now() +
                         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                             std::chrono::duration<double>(wait_limit));
    while (std::chrono::steady_clock::now() < stream.busy_until || moving()) {
      if (std::chrono::steady_clock::now() >= give_up)
        return false;
      std::this_thread::sleep_for(kStreamWaitStep);
    }
    // Close the old stream first -- its commands hold their parts until it goes,
    // and a command naming any of those parts is refused -- and give the robot a
    // moment: a stream opened within ~0.1 s of closing one comes up expired.
    stream.handler.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    stream.handler = robot_->CreateCommandStream(10);
    if (stream.kind_changed)
      RCLCPP_INFO(this->get_logger(), "Command stream reopened: trajectory "
                                      "impedance changed");
    else
      RCLCPP_INFO(this->get_logger(),
                  "Command stream reopened: this command moves parts the "
                  "previous stream never carried (parts 0x%x, stream had 0x%x)",
                  parts, stream.parts);
    stream.parts = parts;
    stream.kind_changed = false;
  } else {
    stream.parts |= parts;
  }
  stream.one_shot = one_shot;
  stream.busy_until =
      std::max(stream.busy_until,
               std::chrono::steady_clock::now() +
                   std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(busy_for)));
  return true;
}

// Sends an upper-body command, the arms' half and the torso's each on its own
// stream. Returns why not when it cannot: a channel that is not open, an arm the arm
// channel does not carry, or a stream whose last command is still moving
// (fit_stream_to). Throws what SendCommand throws. Call with stream_mutex_ held.
template <typename ModelType>
std::string RBY1_ROS2_DRIVER<ModelType>::send_body(BodyCommand &command,
                                                  double busy_for,
                                                  double wait_limit,
                                                  bool one_shot) {
  const int body_parts =
      command.parts & (STREAM_RIGHT_ARM | STREAM_LEFT_ARM | STREAM_TORSO);
  if (const int closed = channels_of_parts(body_parts) & ~open_stream_channels())
    return "stream channel '" + stream_channel_names(closed) +
           "' is not open: open it with stream_control (parameters: " +
           stream_channel_names(closed) + ") and send again";

  struct Send {
    BodyStream *stream;
    const rb::BodyComponentBasedCommandBuilder *body;
    int part;
    const char *channel;
    std::exception_ptr error;
  };
  std::vector<Send> sends;
  if (body_parts & STREAM_RIGHT_ARM)
    sends.push_back({&right_arm_stream_, &command.right_arm, STREAM_RIGHT_ARM, "right_arm", {}});
  if (body_parts & STREAM_LEFT_ARM)
    sends.push_back({&left_arm_stream_, &command.left_arm, STREAM_LEFT_ARM, "left_arm", {}});
  if (body_parts & STREAM_TORSO)
    sends.push_back({&torso_stream_, &command.torso, STREAM_TORSO, "torso", {}});
  // A stream that was cancelled or found expired is replaced, kStreamReopenGap after
  // that: opened sooner, the new one comes up expired as well. The command waits
  // for it (at most the gap), so a trajectory sent right after a cancel is not cut.
  auto reopen = [this](BodyStream &stream, int part, const char *channel) {
    std::this_thread::sleep_until(stream.expired_at + kStreamReopenGap);
    stream.handler = robot_->CreateCommandStream(10);
    stream.parts = part;
    stream.expired_at = {};
    RCLCPP_INFO(this->get_logger(),
                "Command stream '%s' had ended (cancelled or expired); opened a new one",
                channel);
  };
  for (auto &s : sends) {
    if (s.stream->expired_at != std::chrono::steady_clock::time_point{})
      reopen(*s.stream, s.part, s.channel);
    if (!fit_stream_to(*s.stream, s.part, busy_for, wait_limit, one_shot))
      return "another command on the stream is still moving other parts; send again "
             "when it is done";
  }

  auto send_once = [](Send &s) {
    rb::ComponentBasedCommandBuilder component;
    component.SetBodyCommand(rb::BodyCommandBuilder(*s.body));
    rb::RobotCommandBuilder robot_cmd;
    robot_cmd.SetCommand(component);
    s.stream->handler->SendCommand(robot_cmd);
  };
  // The channels' commands go out at the same time. Each SendCommand is one SDK
  // round trip (it waits for the robot's answer), so sent one after another they
  // added up per cycle; side by side a cycle costs the longest one (2026-10-26).
  const auto started = std::chrono::steady_clock::now();
  auto run = [&send_once](Send &s) {
    try {
      send_once(s);
    } catch (...) {
      s.error = std::current_exception();
    }
  };
  std::vector<std::thread> others;
  for (size_t i = 1; i < sends.size(); ++i)
    others.emplace_back(run, std::ref(sends[i]));
  if (!sends.empty())
    run(sends[0]);
  for (auto &t : others)
    t.join();
  RCLCPP_DEBUG(this->get_logger(), "send_body: %zu channel(s) in %.2f ms",
               sends.size(),
               std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - started)
                   .count());
  for (auto &s : sends) {
    if (s.error) {
      try {
        std::rethrow_exception(s.error);
      } catch (const std::exception &e) {
        // The stream was found expired (ended by the robot, or by
        // robot_->CancelControl()): a fresh one, after the gap, takes the command.
        if (std::string(e.what()).find("expired") == std::string::npos)
          throw;
        s.stream->expired_at = std::chrono::steady_clock::now();
        reopen(*s.stream, s.part, s.channel);
        send_once(s);
      }
    }
    s.stream->commanded = true;
  }
  return "";
}

// The head has its own command stream, as the mobile base
// does, so head commands (a head tracker) and an arm trajectory on the upper-body
// stream run side by side. Call with stream_mutex_ held.
template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::send_head(
    const rb::ComponentBasedCommandBuilder &head) {
  if (!head_stream_.handler)
    throw std::runtime_error("the head command stream is not open");
  // An expired stream (robot_->CancelControl() ends every stream) is opened again
  // only after the reopen gap, and the commands until then are dropped. Opened at
  // once it comes up expired again, at every command: the head then stood still for
  // good (simulator, 20 Hz commands: 103 reopenings in 5 s, no motion).
  const auto now = std::chrono::steady_clock::now();
  if (head_stream_.expired_at != std::chrono::steady_clock::time_point{}) {
    if (now < head_stream_.expired_at + kStreamReopenGap)
      return;
    head_stream_.handler = robot_->CreateCommandStream(10);
    head_stream_.expired_at = {};
    RCLCPP_WARN(this->get_logger(),
                "[head] Command stream had expired; opened a new one");
  }
  try {
    head_stream_.handler->SendCommand(rb::RobotCommandBuilder().SetCommand(head));
    head_stream_.commanded = true;
  } catch (const std::exception &e) {
    if (std::string(e.what()).find("expired") == std::string::npos)
      throw;
    head_stream_.expired_at = now;
  }
}

// The parts a trajectory names a joint of (stream_part bits). Each goes on its own
// channel: the arms, the torso, the head.
template <typename ModelType>
int RBY1_ROS2_DRIVER<ModelType>::trajectory_parts(
    const trajectory_msgs::msg::JointTrajectory &trajectory) const {
  const std::pair<int, const std::vector<unsigned int> *> groups[] = {
      {STREAM_TORSO, &info_.torso_joint_idx},
      {STREAM_RIGHT_ARM, &info_.right_arm_joint_idx},
      {STREAM_LEFT_ARM, &info_.left_arm_joint_idx},
      {STREAM_HEAD, &info_.head_joint_idx}};
  int parts = 0;
  for (const auto &name : trajectory.joint_names) {
    for (const auto &[bit, joints] : groups) {
      for (auto idx : *joints) {
        if (idx < info_.joint_infos.size() && info_.joint_infos[idx].name == name)
          parts |= bit;
      }
    }
  }
  return parts;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::check_stream_safety() {
  // A channel opened and then never commanded is closed after kStreamIdleTimeout:
  // its client went away before it sent anything. Once commanded a channel stays
  // open, each on its own (2026-10-26 user decision): its last command holds its
  // part, and a client that plans for long between commands keeps its stream.
  std::lock_guard<std::mutex> stream_lock(stream_mutex_);
  if (!stream_active_)
    return;
  const auto now = std::chrono::steady_clock::now();
  int idle = 0;
  for (int channel : kStreamChannels) {
    const BodyStream &stream = channel_stream(channel);
    if (stream.handler && !stream.commanded &&
        now - stream.opened_at > std::chrono::duration<double>(kStreamIdleTimeout))
      idle |= channel;
  }
  if (idle == 0)
    return;
  RCLCPP_WARN(this->get_logger(),
              "\033[1;31m[STREAM TIMEOUT] Stream channel '%s' got no command for "
              "%.0f s after it was opened: closing it.\033[0m",
              stream_channel_names(idle).c_str(), kStreamIdleTimeout);
  close_stream_channels(idle, false);
  if (!stream_active_ && hardware_control_active_) {
    hardware_control_active_ = false;
    RCLCPP_WARN(this->get_logger(),
                "Hardware control released due to stream timeout.");
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::set_trajectory_impedance_callback(
    const std::shared_ptr<rby1_msgs::srv::SetTrajectoryImpedance::Request>
        request,
    std::shared_ptr<rby1_msgs::srv::SetTrajectoryImpedance::Response>
        response) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Not while a trajectory runs: its waypoints would switch control mode midway.
  // With the stream open, the next upper-body command goes out on a new stream
  // (fit_stream_to), since the robot does not take a new command kind on one.
  if (fjt_parts_running() != 0) {
    response->success = false;
    response->message = "Cannot configure trajectory impedance while a "
                        "follow_joint_trajectory goal is executing; send it "
                        "when the trajectory is done.";
    RCLCPP_WARN(this->get_logger(), "[TRAJECTORY IMPEDANCE] %s",
                response->message.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    for (BodyStream *stream : {&right_arm_stream_, &left_arm_stream_, &torso_stream_})
      stream->kind_changed = stream->handler != nullptr;
  }

  // For Torso
  bool torso_active = (request->state.size() > 0) ? request->state[0] : false;
  if (torso_active) {
    trajectory_impedance_enabled_torso_ = true;
    if (!request->torso_stiffness.empty()) {
      if (request->torso_stiffness.size() != info_.torso_joint_idx.size()) {
        response->success = false;
        response->message = "Invalid torso stiffness size: expected " +
                            std::to_string(info_.torso_joint_idx.size()) +
                            ", got " +
                            std::to_string(request->torso_stiffness.size());
        RCLCPP_ERROR(this->get_logger(), "[TRAJECTORY IMPEDANCE] %s",
                     response->message.c_str());
        return;
      }
      trajectory_stiffness_torso_ = request->torso_stiffness;
    } else {
      trajectory_stiffness_torso_.assign(info_.torso_joint_idx.size(), kDefaultJointStiffness);
    }
  } else {
    trajectory_impedance_enabled_torso_ = false;
  }

  // For Right Arm
  bool right_arm_active =
      (request->state.size() > 1) ? request->state[1] : false;
  if (right_arm_active) {
    trajectory_impedance_enabled_right_arm_ = true;
    if (!request->right_arm_stiffness.empty()) {
      if (request->right_arm_stiffness.size() !=
          info_.right_arm_joint_idx.size()) {
        response->success = false;
        response->message = "Invalid right arm stiffness size: expected " +
                            std::to_string(info_.right_arm_joint_idx.size()) +
                            ", got " +
                            std::to_string(request->right_arm_stiffness.size());
        RCLCPP_ERROR(this->get_logger(), "[TRAJECTORY IMPEDANCE] %s",
                     response->message.c_str());
        return;
      }
      trajectory_stiffness_right_arm_ = request->right_arm_stiffness;
    } else {
      trajectory_stiffness_right_arm_.assign(info_.right_arm_joint_idx.size(), kDefaultJointStiffness);
    }
  } else {
    trajectory_impedance_enabled_right_arm_ = false;
  }

  // For Left Arm
  bool left_arm_active =
      (request->state.size() > 2) ? request->state[2] : false;
  if (left_arm_active) {
    trajectory_impedance_enabled_left_arm_ = true;
    if (!request->left_arm_stiffness.empty()) {
      if (request->left_arm_stiffness.size() !=
          info_.left_arm_joint_idx.size()) {
        response->success = false;
        response->message = "Invalid left arm stiffness size: expected " +
                            std::to_string(info_.left_arm_joint_idx.size()) +
                            ", got " +
                            std::to_string(request->left_arm_stiffness.size());
        RCLCPP_ERROR(this->get_logger(), "[TRAJECTORY IMPEDANCE] %s",
                     response->message.c_str());
        return;
      }
      trajectory_stiffness_left_arm_ = request->left_arm_stiffness;
    } else {
      trajectory_stiffness_left_arm_.assign(info_.left_arm_joint_idx.size(), kDefaultJointStiffness);
    }
  } else {
    trajectory_impedance_enabled_left_arm_ = false;
  }

  trajectory_damping_ratio_ = (!request->damping_ratio.empty())
                                  ? request->damping_ratio[0]
                                  : kDefaultDampingRatio;
  trajectory_torque_limit_ = (!request->torque_limit.empty())
                                 ? request->torque_limit[0]
                                 : kDefaultTorqueLimit;

  response->success = true;
  response->message =
      "Trajectory impedance configured. Torso=" +
      std::to_string(trajectory_impedance_enabled_torso_) +
      ", RightArm=" + std::to_string(trajectory_impedance_enabled_right_arm_) +
      ", LeftArm=" + std::to_string(trajectory_impedance_enabled_left_arm_);
  RCLCPP_INFO(this->get_logger(), "[TRAJECTORY IMPEDANCE] Configured: %s",
              response->message.c_str());
}

template <typename ModelType>
rclcpp_action::GoalResponse
RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const StreamJoint::Goal> goal) {
  (void)uuid;
  (void)goal;
  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting StreamJoint: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamJoint>>
        goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_stream_joint_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamJoint>>
        goal_handle) {
  std::thread{
      std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_stream_joint, this, _1),
      goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_stream_joint(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamJoint>>
        goal_handle) {
  has_printed_collision_log_ = false;

  auto result = std::make_shared<StreamJoint::Result>();
  const auto goal = goal_handle->get_goal();
  const auto &cmd = goal->command;

  // 1. Check if stream is active
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (!stream_active_) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[STREAM REJECTED] stream_joint ignored: Stream is "
                  "not active.\033[0m");
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // 2. Check if a trajectory holds any of the commanded parts
  const int commanded_bits =
      (cmd.torso.position.empty() ? 0 : STREAM_TORSO) |
      (cmd.right_arm.position.empty() ? 0 : STREAM_RIGHT_ARM) |
      (cmd.left_arm.position.empty() ? 0 : STREAM_LEFT_ARM) |
      (cmd.head.position.empty() ? 0 : STREAM_HEAD);
  int running = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running = fjt_parts_running();
  }
  if ((commanded_bits & running) != 0) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "\033[1;33m[STREAM REJECTED] stream_joint ignored: a trajectory is "
        "executing on these parts.\033[0m");
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // Determine commanded parts
  std::vector<size_t> commanded_parts;
  if (!cmd.torso.position.empty())
    commanded_parts.push_back(PART_TORSO);
  if (!cmd.right_arm.position.empty())
    commanded_parts.push_back(PART_RIGHT_ARM);
  if (!cmd.left_arm.position.empty())
    commanded_parts.push_back(PART_LEFT_ARM);
  if (!cmd.head.position.empty())
    commanded_parts.push_back(PART_HEAD);

  if (commanded_parts.empty()) {
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // 3. Check for non-stream control conflict
  for (auto part : commanded_parts) {
    if (is_controlling_[part].load(std::memory_order_acquire)) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[STREAM REJECTED] stream_joint ignored: Part %zu "
                  "is controlled by non-stream command.\033[0m",
                  part);
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // // 4. Calculate required acceleration and verify limits
  // uble dt = part_cmd.minimum_time;
  //     iauto check_accel_limit = [&](const rby1_msgs::msg::JointCommand&
  //     part_cmd, const std::vector<unsigned int>& joint_indices, const
  //     std::string& part_name) -> bool { if (part_cmd.position.empty()) return
  //     true;

  //     dof (dt <= 0.0) dt = 0.1;

  //     auto state = robot_->GetState();

  //     for (size_t i = 0; i < part_cmd.position.size() && i <
  //     joint_indices.size(); ++i) {
  //         unsigned int sdk_idx = joint_indices[i];
  //         std::string joint_name = info_.joint_infos[sdk_idx].name;
  //         //double current_pos = state.position[sdk_idx];
  //         //double target_pos = part_cmd.position[i];

  //         //double req_accel = 2.0 * std::abs(target_pos - current_pos) / (dt
  //         * dt);

  //         double limit = kAccelerationLimit;
  //         if (dynamics_ && dyn_state_) {
  //             auto it = std::find(dyn_joint_names_.begin(),
  //             dyn_joint_names_.end(), joint_name); if (it !=
  //             dyn_joint_names_.end()) {
  //                 int dyn_idx = std::distance(dyn_joint_names_.begin(), it);
  //                 limit = qddot_upper_[dyn_idx];
  //             }
  //         }

  //         // if (req_accel > limit) {
  //         //     RCLCPP_WARN(this->get_logger(),
  //         //                 "\033[1;31m[STREAM REJECTED] Joint '%s' (%s)
  //         required acceleration (%.4f rad/s^2) exceeds limit (%.4f rad/s^2)
  //         with dt=%.4f s. Current: %.4f, Target: %.4f\033[0m",
  //         //                 joint_name.c_str(), part_name.c_str(),
  //         req_accel, limit, dt, current_pos, target_pos);
  //         //     //return false;
  //         // }
  //     }
  //     return true;
  // };

  // if (!check_accel_limit(cmd.torso, info_.torso_joint_idx, "torso") ||
  //     !check_accel_limit(cmd.right_arm, info_.right_arm_joint_idx,
  //     "right_arm") || !check_accel_limit(cmd.left_arm,
  //     info_.left_arm_joint_idx, "left_arm") || !check_accel_limit(cmd.head,
  //     info_.head_joint_idx, "head")) { result->success = false; try {
  //     goal_handle->abort(result); } catch (...) {} return;
  // }

  // 5. Build and send command
  try {
    BodyCommand body; // the torso and the arms, each on its own stream
    body.parts = commanded_bits & ~STREAM_HEAD;
    rb::ComponentBasedCommandBuilder head_comp_builder; // on the head's own stream

    // Torso and arms in joint impedance when set_trajectory_impedance turned it
    // on for the part (as follow_joint_trajectory does), else in joint position.
    auto body_part = [&](const rby1_msgs::msg::JointCommand &part,
                         bool impedance, const std::vector<double> &stiffness,
                         auto &&set) {
      if (part.position.empty())
        return;
      const Eigen::VectorXd pq = Eigen::Map<const Eigen::VectorXd>(
          part.position.data(), part.position.size());
      const double time = part.minimum_time <= 0.0 ? 0.1 : part.minimum_time;
      if (impedance) {
        Eigen::VectorXd k =
            Eigen::VectorXd::Constant(pq.size(), kDefaultJointStiffness);
        if (stiffness.size() == static_cast<size_t>(pq.size()))
          k = Eigen::Map<const Eigen::VectorXd>(stiffness.data(),
                                                stiffness.size());
        rb::JointImpedanceControlCommandBuilder b;
        make_joint_impedance_builder(b, pq, time, kControlHoldTime, k,
                                     trajectory_damping_ratio_,
                                     trajectory_torque_limit_, 0.0, 0.0);
        set(b);
      } else {
        rb::JointPositionCommandBuilder b;
        make_joint_pos_builder(b, pq, time, kControlHoldTime, 0.0, 0.0);
        set(b);
      }
    };
    body_part(cmd.torso, trajectory_impedance_enabled_torso_,
              trajectory_stiffness_torso_, [&](const auto &b) {
                body.torso.SetTorsoCommand(rb::TorsoCommandBuilder(b));
              });
    body_part(cmd.right_arm, trajectory_impedance_enabled_right_arm_,
              trajectory_stiffness_right_arm_, [&](const auto &b) {
                body.right_arm.SetRightArmCommand(rb::ArmCommandBuilder(b));
              });
    body_part(cmd.left_arm, trajectory_impedance_enabled_left_arm_,
              trajectory_stiffness_left_arm_, [&](const auto &b) {
                body.left_arm.SetLeftArmCommand(rb::ArmCommandBuilder(b));
              });

    if (!cmd.head.position.empty()) {
      Eigen::VectorXd head_q = Eigen::Map<const Eigen::VectorXd>(
          cmd.head.position.data(), cmd.head.position.size());
      rb::HeadCommandBuilder head_builder;
      head_builder.SetCommand(
          rb::JointPositionCommandBuilder()
              .SetCommandHeader(
                  rb::CommandHeaderBuilder().SetControlHoldTime(kControlHoldTime))
              .SetPosition(head_q)
              .SetMinimumTime(
                  cmd.head.minimum_time <= 0.0 ? 0.1 : cmd.head.minimum_time));
      head_comp_builder.SetHeadCommand(head_builder);
    }

    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if ((commanded_bits & STREAM_HEAD) && !head_stream_.handler) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "\033[1;33m[STREAM REJECTED] stream_joint ignored: stream channel "
          "'head' is not open: open it with stream_control (parameters: head) "
          "and send again.\033[0m");
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
    auto part_time = [](double t) { return t <= 0.0 ? 0.1 : t; };
    double moves_for = 0.0;
    if (body.parts & STREAM_TORSO)
      moves_for = std::max(moves_for, part_time(cmd.torso.minimum_time));
    if (body.parts & STREAM_RIGHT_ARM)
      moves_for = std::max(moves_for, part_time(cmd.right_arm.minimum_time));
    if (body.parts & STREAM_LEFT_ARM)
      moves_for = std::max(moves_for, part_time(cmd.left_arm.minimum_time));
    const std::string refused = send_body(body, moves_for, 0.0, false);
    if (!refused.empty()) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "\033[1;33m[STREAM REJECTED] stream_joint ignored: %s\033[0m",
          refused.c_str());
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
    if (commanded_bits & STREAM_HEAD)
      send_head(head_comp_builder);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error in StreamJoint SendCommand: %s",
                 e.what());
    deactivate_all_streams();
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  result->success = true;
  try {
    goal_handle->succeed(result);
  } catch (...) {
  }
}

template <typename ModelType>
rclcpp_action::GoalResponse
RBY1_ROS2_DRIVER<ModelType>::handle_stream_cartesian_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const StreamCartesian::Goal> goal) {
  (void)uuid;
  (void)goal;
  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting StreamCartesian: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_stream_cartesian_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamCartesian>>
        goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_stream_cartesian_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamCartesian>>
        goal_handle) {
  std::thread{std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_stream_cartesian,
                        this, _1),
              goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_stream_cartesian(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StreamCartesian>>
        goal_handle) {
  has_printed_collision_log_ = false;

  auto result = std::make_shared<StreamCartesian::Result>();
  const auto goal = goal_handle->get_goal();
  const auto &cmd = goal->command;

  // 1. Check if stream is active
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (!stream_active_) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[STREAM REJECTED] stream_cartesian ignored: "
                  "Stream is not active.\033[0m");
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // 2. A trajectory on one of the commanded parts (a part is commanded when its
  //    ref_link or target_link is given)
  const auto names = [](const rby1_msgs::msg::CartesianCommand &c) {
    return !(c.ref_link.empty() && c.target_link.empty());
  };
  const int commanded_bits = (names(cmd.torso) ? STREAM_TORSO : 0) |
                             (names(cmd.right_arm) ? STREAM_RIGHT_ARM : 0) |
                             (names(cmd.left_arm) ? STREAM_LEFT_ARM : 0);
  int running = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running = fjt_parts_running();
  }
  if ((commanded_bits & running) != 0) {
    RCLCPP_WARN(this->get_logger(),
                "\033[1;33m[STREAM REJECTED] stream_cartesian ignored: a trajectory "
                "is executing on these parts.\033[0m");
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // Determine commanded parts
  std::vector<size_t> commanded_parts;
  if (!cmd.torso.ref_link.empty() || !cmd.torso.target_link.empty())
    commanded_parts.push_back(PART_TORSO);
  if (!cmd.right_arm.ref_link.empty() || !cmd.right_arm.target_link.empty())
    commanded_parts.push_back(PART_RIGHT_ARM);
  if (!cmd.left_arm.ref_link.empty() || !cmd.left_arm.target_link.empty())
    commanded_parts.push_back(PART_LEFT_ARM);

  if (commanded_parts.empty()) {
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // 3. Check for non-stream control conflict
  for (auto part : commanded_parts) {
    if (is_controlling_[part].load(std::memory_order_acquire)) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[STREAM REJECTED] stream_cartesian ignored: Part "
                  "%zu is controlled by non-stream command.\033[0m",
                  part);
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // 4. Parse Cartesian commands and solve IK
  std::vector<typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget>
      link_targets;

  auto add_target_helper =
      [&](const rby1_msgs::msg::CartesianCommand &part_cmd) -> bool {
    if (part_cmd.ref_link.empty() && part_cmd.target_link.empty())
      return true;
    if (part_cmd.ref_link.empty() || part_cmd.target_link.empty())
      return false;

    int ref_idx = get_link_index(part_cmd.ref_link);
    int target_idx = get_link_index(part_cmd.target_link);
    if (ref_idx == -1 || target_idx == -1)
      return false;

    typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget target;
    target.ref_link_index = ref_idx;
    target.link_index = target_idx;

    Eigen::Quaterniond q_rot(
        part_cmd.transform.rotation.w, part_cmd.transform.rotation.x,
        part_cmd.transform.rotation.y, part_cmd.transform.rotation.z);
    Eigen::Vector3d t(part_cmd.transform.translation.x,
                      part_cmd.transform.translation.y,
                      part_cmd.transform.translation.z);

    rb::math::SE3::MatrixType T = rb::math::SE3::MatrixType::Identity();
    T.block(0, 0, 3, 3) = q_rot.toRotationMatrix();
    T.block(0, 3, 3, 1) = t;
    target.T = T;

    target.weight_position = kTranslationWeight;
    target.weight_orientation = kRotationWeight;

    link_targets.push_back(target);
    return true;
  };

  bool ok = true;
  ok &= add_target_helper(cmd.torso);
  ok &= add_target_helper(cmd.right_arm);
  ok &= add_target_helper(cmd.left_arm);

  if (!ok || link_targets.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "stream_cartesian: Invalid Cartesian targets");
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // Best-effort collision pre-check: in the regular Cartesian action the IK
  // is used only for collision prediction. However in stream_cartesian the
  // solved joint positions ARE the actual command, so we must abort if IK
  // fails.
  auto solved_q = solve_cartesian_ik(link_targets);
  if (!solved_q.has_value()) {
    RCLCPP_WARN(this->get_logger(), "stream_cartesian: IK did not converge — "
                                    "cannot compute joint targets.");
    result->success = false;
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // 5. Find max minimum_time
  double dt = 0.0;
  if (!cmd.torso.ref_link.empty())
    dt = std::max(dt, cmd.torso.minimum_time);
  if (!cmd.right_arm.ref_link.empty())
    dt = std::max(dt, cmd.right_arm.minimum_time);
  if (!cmd.left_arm.ref_link.empty())
    dt = std::max(dt, cmd.left_arm.minimum_time);

  if (dt <= 0.0)
    dt = 0.1;

  // 6. Calculate required acceleration and verify limits
  auto state = get_state_with_retry();
  for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
    std::string joint_name = dyn_joint_names_[k];
    double target_pos = solved_q.value()[k];

    double current_pos = 0.0;
    bool found = false;
    for (size_t j = 0; j < info_.joint_infos.size(); ++j) {
      if (info_.joint_infos[j].name == joint_name) {
        current_pos = state.position[j];
        found = true;
        break;
      }
    }
    if (!found)
      continue;

    double req_accel = 2.0 * std::abs(target_pos - current_pos) / (dt * dt);
    double limit = qddot_upper_[k];

    if (req_accel > limit) {
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;31m[STREAM REJECTED] Cartesian target: Joint '%s' "
                  "required acceleration (%.4f rad/s^2) exceeds limit (%.4f "
                  "rad/s^2) with dt=%.4f s. Current: %.4f, Target: %.4f. "
                  "Stopped: the commanded parts hold where they are.\033[0m",
                  joint_name.c_str(), req_accel, limit, dt, current_pos,
                  target_pos);
      {
        // A target past the limit means the sender has lost track of the arm: the
        // parts it commanded are held where they are measured, rather than left
        // to finish the last command. On their own streams, which stay open --
        // cancelling a stream and opening another was tried for this: the new one
        // took commands the robot ignored, and after a few rounds the driver hung.
        const double stop_time = kCancelHoldTime;
        BodyCommand hold;
        auto here = [&](const std::vector<unsigned int> &joints, int part,
                        auto &&set) {
          Eigen::VectorXd q(joints.size());
          for (size_t i = 0; i < joints.size(); ++i)
            q[i] = state.position[joints[i]];
          rb::JointPositionCommandBuilder b;
          make_joint_pos_builder(b, q, stop_time, kControlHoldTime);
          set(b);
          hold.parts |= part;
        };
        if (!cmd.torso.ref_link.empty())
          here(info_.torso_joint_idx, STREAM_TORSO, [&](const auto &b) {
            hold.torso.SetTorsoCommand(rb::TorsoCommandBuilder(b));
          });
        if (!cmd.right_arm.ref_link.empty())
          here(info_.right_arm_joint_idx, STREAM_RIGHT_ARM, [&](const auto &b) {
            hold.right_arm.SetRightArmCommand(rb::ArmCommandBuilder(b));
          });
        if (!cmd.left_arm.ref_link.empty())
          here(info_.left_arm_joint_idx, STREAM_LEFT_ARM, [&](const auto &b) {
            hold.left_arm.SetLeftArmCommand(rb::ArmCommandBuilder(b));
          });
        try {
          std::lock_guard<std::mutex> stream_lock(stream_mutex_);
          const std::string refused = send_body(hold, stop_time, 0.0, false);
          if (!refused.empty())
            RCLCPP_WARN(this->get_logger(), "[STREAM REJECTED] not held: %s",
                        refused.c_str());
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(),
                       "[STREAM REJECTED] holding the parts failed: %s", e.what());
        }
      }
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // 7. Map solved joint values to target command arrays
  Eigen::VectorXd torso_q(info_.torso_joint_idx.size());
  Eigen::VectorXd right_q(info_.right_arm_joint_idx.size());
  Eigen::VectorXd left_q(info_.left_arm_joint_idx.size());

  for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
    std::string joint_name = dyn_joint_names_[k];
    double pos_val = solved_q.value()[k];

    for (size_t i = 0; i < info_.torso_joint_idx.size(); ++i) {
      if (info_.joint_infos[info_.torso_joint_idx[i]].name == joint_name) {
        torso_q[i] = pos_val;
        break;
      }
    }
    for (size_t i = 0; i < info_.right_arm_joint_idx.size(); ++i) {
      if (info_.joint_infos[info_.right_arm_joint_idx[i]].name == joint_name) {
        right_q[i] = pos_val;
        break;
      }
    }
    for (size_t i = 0; i < info_.left_arm_joint_idx.size(); ++i) {
      if (info_.joint_infos[info_.left_arm_joint_idx[i]].name == joint_name) {
        left_q[i] = pos_val;
        break;
      }
    }
  }

  BodyCommand body; // the torso and the arms, each on its own stream

  if (!cmd.torso.ref_link.empty()) {
    rb::TorsoCommandBuilder torso_builder;
    torso_builder.SetCommand(
        rb::JointPositionCommandBuilder()
            .SetCommandHeader(
                rb::CommandHeaderBuilder().SetControlHoldTime(kControlHoldTime))
            .SetPosition(torso_q)
            .SetMinimumTime(
                cmd.torso.minimum_time <= 0.0 ? 0.1 : cmd.torso.minimum_time));
    body.torso.SetTorsoCommand(torso_builder);
    body.parts |= STREAM_TORSO;
  }
  if (!cmd.right_arm.ref_link.empty()) {
    rb::ArmCommandBuilder right_builder;
    right_builder.SetCommand(
        rb::JointPositionCommandBuilder()
            .SetCommandHeader(
                rb::CommandHeaderBuilder().SetControlHoldTime(kControlHoldTime))
            .SetPosition(right_q)
            .SetMinimumTime(cmd.right_arm.minimum_time <= 0.0
                                ? kStreamCartesianMinimumTime
                                : cmd.right_arm.minimum_time));
    body.right_arm.SetRightArmCommand(right_builder);
    body.parts |= STREAM_RIGHT_ARM;
  }
  if (!cmd.left_arm.ref_link.empty()) {
    rb::ArmCommandBuilder left_builder;
    left_builder.SetCommand(
        rb::JointPositionCommandBuilder()
            .SetCommandHeader(
                rb::CommandHeaderBuilder().SetControlHoldTime(kControlHoldTime))
            .SetPosition(left_q)
            .SetMinimumTime(cmd.left_arm.minimum_time <= 0.0
                                ? kStreamCartesianMinimumTime
                                : cmd.left_arm.minimum_time));
    body.left_arm.SetLeftArmCommand(left_builder);
    body.parts |= STREAM_LEFT_ARM;
  }

  if (body.parts != 0) {
    try {
      std::lock_guard<std::mutex> stream_lock(stream_mutex_);
      auto part_time = [](double t) { return t <= 0.0 ? 0.1 : t; };
      const double moves_for = std::max(
          {cmd.torso.ref_link.empty() ? 0.0 : part_time(cmd.torso.minimum_time),
           cmd.right_arm.ref_link.empty() ? 0.0
                                          : part_time(cmd.right_arm.minimum_time),
           cmd.left_arm.ref_link.empty() ? 0.0
                                         : part_time(cmd.left_arm.minimum_time)});
      const std::string refused = send_body(body, moves_for, 0.0, false);
      if (!refused.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "\033[1;33m[STREAM REJECTED] stream_cartesian ignored: %s\033[0m",
            refused.c_str());
        result->success = false;
        try {
          goal_handle->abort(result);
        } catch (...) {
        }
        return;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Error in StreamCartesian SendCommand: %s", e.what());
      deactivate_all_streams();
      result->success = false;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  result->success = true;
  try {
    goal_handle->succeed(result);
  } catch (...) {
  }
}

}  // namespace rby1_ros2
