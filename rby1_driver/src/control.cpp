// Single commands of the RB-Y1 driver: gravity compensation, robot_joint, robot_cartesian,
// and cancel_control, which stops every command, single or streamed.
// Compiled as part of rby1_ros2_driver.cpp, which includes this file: the driver is a
// class template instantiated once per robot model there, not here.
#include "rby1_ros2_driver.hpp"

namespace rby1_ros2 {

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::cancel_control_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Cancel Control service called");
  is_control_canceled_ = true;
  // Clear all per-part control flags
  for (size_t i = 0; i < PART_COUNT; ++i)
    is_controlling_[i].store(false, std::memory_order_release);
  // Increment preemption sequence to unblock any waiting actions
  ++command_seq_;
  robot_->CancelControl();
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    close_stream_channels(CHANNEL_ALL, true);
  }
  response->success = true;
  response->message = "Control cancelled";
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::gravity_compensation_callback(
    const std::shared_ptr<rby1_msgs::srv::GravityCompensation::Request> request,
    std::shared_ptr<rby1_msgs::srv::GravityCompensation::Response> response) {
  if (request->part_name != "torso" && request->part_name != "right_arm" &&
      request->part_name != "left_arm") {
    response->response = false;
    RCLCPP_ERROR(this->get_logger(),
                 "GravityCompensation: Invalid part_name: %s",
                 request->part_name.c_str());
    return;
  }

  size_t part_idx = PART_COUNT;
  if (request->part_name == "torso")
    part_idx = PART_TORSO;
  else if (request->part_name == "right_arm")
    part_idx = PART_RIGHT_ARM;
  else if (request->part_name == "left_arm")
    part_idx = PART_LEFT_ARM;

  // 1. Collision check: Robot must NOT be moving (executing/switching)
  const auto &cm_state = robot_->GetControlManagerState();

  bool current_gc_state = false;
  if (part_idx == PART_TORSO)
    current_gc_state = gravity_compensation_torso_;
  else if (part_idx == PART_RIGHT_ARM)
    current_gc_state = gravity_compensation_right_arm_;
  else if (part_idx == PART_LEFT_ARM)
    current_gc_state = gravity_compensation_left_arm_;

  // We only enforce the active execution check if:
  // - We are enabling gravity compensation (request->state == true)
  // - AND gravity compensation is not already active on this body part.
  if (request->state) {
    {
      std::lock_guard<std::mutex> stream_lock(stream_mutex_);
      if (stream_active_) {
        response->response = false;
        RCLCPP_WARN(this->get_logger(),
                    "GravityCompensation: Stream is active. Gravity "
                    "compensation cannot be enabled while stream is active.");
        return;
      }
    }
    if (is_controlling_[part_idx].load(std::memory_order_acquire) &&
        !current_gc_state) {
      response->response = false;
      RCLCPP_WARN(this->get_logger(),
                  "GravityCompensation: Part %s is already being controlled.",
                  request->part_name.c_str());
      return;
    }

    if (!current_gc_state) {
      if (cm_state.control_state ==
              rb::ControlManagerState::ControlState::kExecuting ||
          cm_state.control_state ==
              rb::ControlManagerState::ControlState::kSwitching) {
        response->response = false;
        RCLCPP_WARN(this->get_logger(),
                    "GravityCompensation: Robot is active in "
                    "execution/switching state. Rejecting request.");
        return;
      }
    }
  }

  // 2. Build and Send Gravity Compensation command
  auto b = rb::GravityCompensationCommandBuilder();
  b.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(
      10.0)); // standard control hold time
  b.SetOn(request->state);

  auto body_comp = rb::BodyComponentBasedCommandBuilder();
  if (request->part_name == "torso") {
    body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
  } else if (request->part_name == "right_arm") {
    body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
  } else if (request->part_name == "left_arm") {
    body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
  }

  auto component_cmd_builder = rb::ComponentBasedCommandBuilder();
  component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));

  try {
    if (request->state) {
      is_controlling_[part_idx].store(true, std::memory_order_release);
      auto cmd_handler = robot_->SendCommand(
          rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
      if (request->part_name == "torso") {
        gravity_compensation_torso_ = true;
        gravity_compensation_torso_handler_ = std::move(cmd_handler);
      } else if (request->part_name == "right_arm") {
        gravity_compensation_right_arm_ = true;
        gravity_compensation_right_arm_handler_ = std::move(cmd_handler);
      } else if (request->part_name == "left_arm") {
        gravity_compensation_left_arm_ = true;
        gravity_compensation_left_arm_handler_ = std::move(cmd_handler);
      }
    } else {
      // Reset/cancel the original ON command handler first
      if (request->part_name == "torso") {
        gravity_compensation_torso_handler_.reset();
        gravity_compensation_torso_ = false;
      } else if (request->part_name == "right_arm") {
        gravity_compensation_right_arm_handler_.reset();
        gravity_compensation_right_arm_ = false;
      } else if (request->part_name == "left_arm") {
        gravity_compensation_left_arm_handler_.reset();
        gravity_compensation_left_arm_ = false;
      }
      is_controlling_[part_idx].store(false, std::memory_order_release);

      // Send explicit command to turn gravity compensation OFF and wait for
      // completion
      auto cmd_handler = robot_->SendCommand(
          rb::RobotCommandBuilder().SetCommand(component_cmd_builder));
      if (cmd_handler) {
        cmd_handler->WaitFor(
            1000); // Wait up to 1 second for transition to complete
      }
    }
    response->response = true;
    RCLCPP_INFO(
        this->get_logger(),
        "GravityCompensation: Successfully set %s gravity compensation to %s",
        request->part_name.c_str(), request->state ? "ON" : "OFF");
  } catch (const std::exception &e) {
    if (request->state) {
      is_controlling_[part_idx].store(false, std::memory_order_release);
      if (request->part_name == "torso")
        gravity_compensation_torso_ = false;
      else if (request->part_name == "right_arm")
        gravity_compensation_right_arm_ = false;
      else if (request->part_name == "left_arm")
        gravity_compensation_left_arm_ = false;
    }
    response->response = false;
    RCLCPP_ERROR(this->get_logger(),
                 "GravityCompensation: Failed to send command: %s", e.what());
  }
}

// --- Rby1 Joint Command Handlers ---
template <typename ModelType>
rclcpp_action::GoalResponse RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Rby1JointCommand::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received Rby1JointCommand request");
  (void)uuid;
  // Validation happens in execute; here only whether a trajectory holds a part.
  const int parts = (goal->torso.position.empty() ? 0 : STREAM_TORSO) |
                    (goal->right_arm.position.empty() ? 0 : STREAM_RIGHT_ARM) |
                    (goal->left_arm.position.empty() ? 0 : STREAM_LEFT_ARM) |
                    (goal->head.position.empty() ? 0 : STREAM_HEAD);

  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting Rby1JointCommand: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((parts & fjt_parts_running()) != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Rejecting Rby1JointCommand: FollowJointTrajectory is "
                  "currently executing on these parts.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "Received request to cancel Rby1JointCommand goal");
  (void)goal_handle;
  robot_->CancelControl();
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_rby1_joint_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>>
        goal_handle) {
  using namespace std::placeholders;
  std::thread{
      std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_rby1_joint_command, this,
                _1),
      goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_rby1_joint_command(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1JointCommand>>
        goal_handle) {
  has_printed_collision_log_ = false;
  is_control_canceled_ = false;
  auto result = std::make_shared<Rby1JointCommand::Result>();
  const auto goal = goal_handle->get_goal();

  // Determine commanded parts
  std::vector<size_t> commanded_parts;
  if (!goal->torso.position.empty())
    commanded_parts.push_back(PART_TORSO);
  if (!goal->right_arm.position.empty())
    commanded_parts.push_back(PART_RIGHT_ARM);
  if (!goal->left_arm.position.empty())
    commanded_parts.push_back(PART_LEFT_ARM);
  if (!goal->head.position.empty())
    commanded_parts.push_back(PART_HEAD);

  if (commanded_parts.empty()) {
    result->success = false;
    result->finish_code = "Empty arrays";
    RCLCPP_WARN(this->get_logger(),
                "\033[1;33m[CONTROL REJECTED] Empty Rby1JointCommand.\033[0m");
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // ── NORMAL / NO-STREAM MODE ───────────────────────────────────────────
  // Conflict check: reject if any commanded part is already being controlled
  for (auto part : commanded_parts) {
    if (is_controlling_[part].load(std::memory_order_acquire)) {
      result->success = false;
      result->finish_code =
          "Part " + std::to_string(part) + " already being controlled";
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[CONTROL REJECTED] Part %zu already being "
                  "controlled.\033[0m",
                  part);
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // RAII: set is_controlling_ for commanded parts, clear on exit
  struct PartGuard {
    std::atomic<bool> *f;
    std::vector<size_t> p;
    PartGuard(std::atomic<bool> *flags, std::vector<size_t> parts)
        : f(flags), p(std::move(parts)) {
      for (auto i : p)
        f[i].store(true, std::memory_order_release);
    }
    ~PartGuard() {
      for (auto i : p)
        f[i].store(false, std::memory_order_release);
    }
  } guard(is_controlling_, commanded_parts);

  try {
    // Power/servo check
    auto cm_state_val = robot_->GetControlManagerState().state;
    bool is_ready = (cm_state_val == rb::ControlManagerState::State::kEnabled);
    if (!is_ready) {
      try {
        is_ready = robot_->IsPowerOn(".*") && robot_->IsServoOn(".*");
      } catch (...) {
        is_ready = false;
      }
    }
    if (!is_ready) {
      result->success = false;
      result->finish_code = "Robot is disabled (power/servo is off)";
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[CONTROL REJECTED] Robot disabled.\033[0m");
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    if (!robot_->HasEstablishedTimeSync())
      robot_->SyncTime();
    if (!this->check_controll_manager()) {
      result->success = false;
      result->finish_code = "Control Manager Error";
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31m[CONTROL MANAGER ERROR] Not ready.\033[0m");
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    // Pre-collision detection
    if (dynamics_ && dyn_state_) {
      Eigen::Vector<double, ModelType::kRobotDOF> target_q =
          Eigen::Vector<double, ModelType::kRobotDOF>::Zero();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto cs = get_state_with_retry();
        for (int i = 0; i < (int)dyn_joint_names_.size(); ++i)
          for (size_t j = 0; j < info_.joint_infos.size(); ++j)
            if (info_.joint_infos[j].name == dyn_joint_names_[i]) {
              target_q[i] = cs.position[j];
              break;
            }
      }
      auto update_tq = [&](const rby1_msgs::msg::JointCommand &cmd,
                           const std::string &pn) {
        if (cmd.position.empty())
          return;
        if (cmd.use_group_joint) {
          for (size_t i = 0;
               i < cmd.joint_names.size() && i < cmd.position.size(); ++i)
            for (size_t k = 0; k < dyn_joint_names_.size(); ++k)
              if (dyn_joint_names_[k] == cmd.joint_names[i]) {
                target_q[k] = cmd.position[i];
                break;
              }
        } else {
          const std::vector<unsigned int> *pi = nullptr;
          if (pn == "torso")
            pi = &info_.torso_joint_idx;
          else if (pn == "right_arm")
            pi = &info_.right_arm_joint_idx;
          else if (pn == "left_arm")
            pi = &info_.left_arm_joint_idx;
          else if (pn == "head")
            pi = &info_.head_joint_idx;
          if (pi && cmd.position.size() == pi->size())
            for (size_t i = 0; i < cmd.position.size(); ++i) {
              std::string jn = info_.joint_infos[(*pi)[i]].name;
              for (size_t k = 0; k < dyn_joint_names_.size(); ++k)
                if (dyn_joint_names_[k] == jn) {
                  target_q[k] = cmd.position[i];
                  break;
                }
            }
        }
      };
      update_tq(goal->torso, "torso");
      update_tq(goal->right_arm, "right_arm");
      update_tq(goal->left_arm, "left_arm");
      update_tq(goal->head, "head");
      auto reason = get_predicted_collision_reason(target_q);
      if (reason.has_value()) {
        RCLCPP_WARN(this->get_logger(),
                    "\033[1;33m[PREDICTIVE COLLISION WARNING] %s\033[0m",
                    reason.value().c_str());
      }
    }

    // Build command. In stream mode the head goes on its own stream, so it is
    // built apart.
    rb::ComponentBasedCommandBuilder component_cmd_builder;
    rb::ComponentBasedCommandBuilder head_cmd_builder;
    const bool head_apart = stream_active_;
    rb::BodyComponentBasedCommandBuilder body_comp;
    // In stream mode the torso and each arm go out on their own streams too.
    BodyCommand stream_body;
    auto &torso_comp = head_apart ? stream_body.torso : body_comp;
    auto &right_comp = head_apart ? stream_body.right_arm : body_comp;
    auto &left_comp = head_apart ? stream_body.left_arm : body_comp;
    bool use_body = false;
    bool use_head = false;
    std::string err_msg;

    if (!process_joint_part(goal->torso, "torso", info_.torso_joint_idx.size(),
                            torso_comp, component_cmd_builder, use_body,
                            use_head, err_msg) ||
        !process_joint_part(
            goal->right_arm, "right_arm", info_.right_arm_joint_idx.size(),
            right_comp, component_cmd_builder, use_body, use_head, err_msg) ||
        !process_joint_part(
            goal->left_arm, "left_arm", info_.left_arm_joint_idx.size(),
            left_comp, component_cmd_builder, use_body, use_head, err_msg) ||
        !process_joint_part(goal->head, "head", info_.head_joint_idx.size(),
                            body_comp,
                            head_apart ? head_cmd_builder : component_cmd_builder,
                            use_body, use_head, err_msg)) {
      result->success = false;
      result->finish_code = err_msg;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
    if (!use_body && !use_head) {
      result->success = false;
      result->finish_code = "Empty arrays";
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
    if (use_body)
      component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));

    // ── STREAM MODE: send on the parts' streams, wait minimum_time ──
    if (head_apart) {
      {
        std::lock_guard<std::mutex> sl(stream_mutex_);
        double moves_for =
            std::max({goal->torso.minimum_time, goal->right_arm.minimum_time,
                      goal->left_arm.minimum_time, goal->head.minimum_time});
        stream_body.parts =
            (goal->torso.position.empty() ? 0 : STREAM_TORSO) |
            (goal->right_arm.position.empty() ? 0 : STREAM_RIGHT_ARM) |
            (goal->left_arm.position.empty() ? 0 : STREAM_LEFT_ARM);
        const std::string refused =
            use_head && !head_stream_.handler
                ? "stream channel 'head' is not open: open it with stream_control "
                  "(parameters: head) and send again"
                : send_body(stream_body,
                            moves_for > 0.0 ? moves_for : kDefaultMinimumTime,
                            kStreamSendWait, true);
        if (!refused.empty()) {
          result->success = false;
          result->finish_code = "Stream: " + refused;
          try {
            goal_handle->abort(result);
          } catch (...) {
          }
          return;
        }
        if (use_head)
          send_head(head_cmd_builder);
      }
      double max_mt =
          std::max({goal->torso.minimum_time, goal->right_arm.minimum_time,
                    goal->left_arm.minimum_time, goal->head.minimum_time});
      if (max_mt <= 0.0)
        max_mt = kDefaultMinimumTime;
      uint64_t my_seq = ++command_seq_;
      auto deadline = std::chrono::steady_clock::now() +
                      std::chrono::duration<double>(max_mt);
      rclcpp::Rate rate(100);
      while (rclcpp::ok()) {
        if (command_seq_.load() != my_seq) {
          result->success = false;
          result->finish_code = "kPreempted";
          try {
            goal_handle->canceled(result);
          } catch (...) {
          }
          return;
        }
        if (goal_handle->is_canceling() || is_control_canceled_) {
          result->success = false;
          result->finish_code = "kCanceled";
          try {
            goal_handle->canceled(result);
          } catch (...) {
          }
          return;
        }
        if (std::chrono::steady_clock::now() >= deadline)
          break;
        rate.sleep();
      }
      result->success = true;
      result->finish_code = "kOk";
      try {
        goal_handle->succeed(result);
      } catch (...) {
      }
      return;
    }

    // ── NO STREAM: classic blocking SendCommand ──
    auto cmd_handler = robot_->SendCommand(
        rb::RobotCommandBuilder().SetCommand(component_cmd_builder),
        goal->priority);

    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !cmd_handler->IsDone()) {
      if (goal_handle->is_canceling()) {
        cmd_handler->Cancel();
        result->success = false;
        result->finish_code = "kCanceled";
        try {
          goal_handle->canceled(result);
        } catch (...) {
        }
        return;
      }
      auto cm_state = robot_->GetControlManagerState();
      if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
          cm_state.state == rb::ControlManagerState::State::kMinorFault) {
        cmd_handler->Cancel();
        result->success = false;
        result->finish_code = "Fault Detected";
        try {
          goal_handle->abort(result);
        } catch (...) {
        }
        return;
      }
      auto feedback = std::make_shared<Rby1JointCommand::Feedback>();
      feedback->current_state = "executing";
      try {
        goal_handle->publish_feedback(feedback);
      } catch (...) {
      }
      rate.sleep();
    }

    if (rclcpp::ok()) {
      auto rv = cmd_handler->Get();
      result->finish_code = this->finish_code_to_string(rv.finish_code());
      if (is_control_canceled_)
        result->finish_code = "kCanceled";
      result->success = (result->finish_code == "kOk");
      RCLCPP_INFO(this->get_logger(), "Rby1JointCommand finished: %s",
                  result->finish_code.c_str());
      if (result->success) {
        try {
          goal_handle->succeed(result);
        } catch (...) {
        }
      } else {
        try {
          goal_handle->abort(result);
        } catch (...) {
        }
        this->check_controll_manager();
      }
    } else {
      // Shutting down (Ctrl+C) with the goal still active: end it here. A goal handle
      // destroyed while active ends itself, and that throws once rcl is down, which
      // would terminate the driver instead of letting it exit.
      result->success = false;
      result->finish_code = "kShutdown";
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in Rby1JointCommand: %s",
                 e.what());
    result->success = false;
    result->finish_code = "kError";
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    this->check_controll_manager();
  }
}

// --- Rby1 Cartesian Command Handlers ---
template <typename ModelType>
rclcpp_action::GoalResponse
RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Rby1CartesianCommand::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received Rby1CartesianCommand request");
  (void)uuid;
  (void)goal;

  if (hardware_control_active_) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting Rby1CartesianCommand: Hardware control is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // A part is commanded when its ref_link or target_link is given (process_cartesian_part)
  const auto names = [](const rby1_msgs::msg::CartesianCommand &c) {
    return !(c.ref_link.empty() && c.target_link.empty());
  };
  const int parts = (names(goal->torso) ? STREAM_TORSO : 0) |
                    (names(goal->right_arm) ? STREAM_RIGHT_ARM : 0) |
                    (names(goal->left_arm) ? STREAM_LEFT_ARM : 0);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((parts & fjt_parts_running()) != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Rejecting Rby1CartesianCommand: FollowJointTrajectory is "
                  "currently executing on these parts.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ModelType>
rclcpp_action::CancelResponse
RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "Received request to cancel Rby1CartesianCommand goal");
  (void)goal_handle;
  robot_->CancelControl();
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::handle_rby1_cartesian_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>>
        goal_handle) {
  using namespace std::placeholders;
  std::thread{
      std::bind(&RBY1_ROS2_DRIVER<ModelType>::execute_rby1_cartesian_command,
                this, _1),
      goal_handle}
      .detach();
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::execute_rby1_cartesian_command(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rby1CartesianCommand>>
        goal_handle) {
  has_printed_collision_log_ = false;
  is_control_canceled_ = false;
  auto result = std::make_shared<Rby1CartesianCommand::Result>();
  const auto goal = goal_handle->get_goal();

  // 1. Determine commanded parts
  std::vector<size_t> commanded_parts;
  if (!goal->torso.ref_link.empty() || !goal->torso.target_link.empty())
    commanded_parts.push_back(PART_TORSO);
  if (!goal->right_arm.ref_link.empty() || !goal->right_arm.target_link.empty())
    commanded_parts.push_back(PART_RIGHT_ARM);
  if (!goal->left_arm.ref_link.empty() || !goal->left_arm.target_link.empty())
    commanded_parts.push_back(PART_LEFT_ARM);

  if (commanded_parts.empty()) {
    result->success = false;
    result->finish_code = "Empty targets";
    RCLCPP_WARN(
        this->get_logger(),
        "\033[1;33m[CONTROL REJECTED] Empty Rby1CartesianCommand.\033[0m");
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    return;
  }

  // Conflict check: reject if any commanded part is already being controlled
  for (auto part : commanded_parts) {
    if (is_controlling_[part].load(std::memory_order_acquire)) {
      result->success = false;
      result->finish_code =
          "Part " + std::to_string(part) + " already being controlled";
      RCLCPP_WARN(
          this->get_logger(),
          "\033[1;33m[CONTROL REJECTED] Part %zu already controlled.\033[0m",
          part);
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }
  }

  // PartGuard RAII
  struct PartGuard {
    std::atomic<bool> *f;
    std::vector<size_t> p;
    PartGuard(std::atomic<bool> *flags, std::vector<size_t> parts)
        : f(flags), p(std::move(parts)) {
      for (auto i : p)
        f[i].store(true, std::memory_order_release);
    }
    ~PartGuard() {
      for (auto i : p)
        f[i].store(false, std::memory_order_release);
    }
  } guard(is_controlling_, commanded_parts);

  try {
    // Power/servo check
    auto cm_state_val = robot_->GetControlManagerState().state;
    bool is_ready = (cm_state_val == rb::ControlManagerState::State::kEnabled);
    if (!is_ready) {
      try {
        is_ready = robot_->IsPowerOn(".*") && robot_->IsServoOn(".*");
      } catch (...) {
        is_ready = false;
      }
    }
    if (!is_ready) {
      result->success = false;
      result->finish_code = "Robot is disabled (power/servo is off)";
      RCLCPP_WARN(this->get_logger(),
                  "\033[1;33m[CONTROL REJECTED] Robot disabled.\033[0m");
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    if (!robot_->HasEstablishedTimeSync())
      robot_->SyncTime();

    if (!this->check_controll_manager()) {
      result->success = false;
      result->finish_code = "Control Manager Error";
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    // Pre-collision detection (IK and collision check)
    if (dynamics_ && dyn_state_) {
      std::vector<typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget>
          link_targets;
      auto add_target_helper =
          [&](const rby1_msgs::msg::CartesianCommand &cmd) -> bool {
        if (cmd.ref_link.empty() && cmd.target_link.empty())
          return true;
        if (cmd.ref_link.empty() || cmd.target_link.empty())
          return false;
        int ref_idx = get_link_index(cmd.ref_link);
        int target_idx = get_link_index(cmd.target_link);
        if (ref_idx == -1 || target_idx == -1)
          return false;

        typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget target;
        target.ref_link_index = ref_idx;
        target.link_index = target_idx;

        Eigen::Quaterniond q(cmd.transform.rotation.w, cmd.transform.rotation.x,
                             cmd.transform.rotation.y,
                             cmd.transform.rotation.z);
        Eigen::Vector3d t(cmd.transform.translation.x,
                          cmd.transform.translation.y,
                          cmd.transform.translation.z);
        rb::math::SE3::MatrixType T = rb::math::SE3::MatrixType::Identity();
        T.block(0, 0, 3, 3) = q.toRotationMatrix();
        T.block(0, 3, 3, 1) = t;
        target.T = T;
        target.weight_position = kTranslationWeight;
        target.weight_orientation = kRotationWeight;
        link_targets.push_back(target);
        return true;
      };

      bool ok = true;
      ok &= add_target_helper(goal->torso);
      ok &= add_target_helper(goal->right_arm);
      ok &= add_target_helper(goal->left_arm);

      if (ok && !link_targets.empty()) {
        auto solved_q = solve_cartesian_ik(link_targets);
        if (!solved_q.has_value()) {
          RCLCPP_WARN(this->get_logger(),
                      "[PREDICTIVE COLLISION CHECK] IK did not converge — "
                      "skipping collision pre-check. Command will proceed.");
        } else {
          auto reason = get_predicted_collision_reason(solved_q.value());
          if (reason.has_value()) {
            RCLCPP_WARN(this->get_logger(),
                        "\033[1;33m[PREDICTIVE COLLISION WARNING] %s\033[0m",
                        reason.value().c_str());
          }
        }
      }
    }

    // Build Cartesian Command
    rb::ComponentBasedCommandBuilder component_cmd_builder;
    rb::BodyComponentBasedCommandBuilder body_comp;
    // In stream mode the torso and each arm go out on their own streams.
    const bool streaming = stream_active_;
    BodyCommand stream_body;
    auto &torso_comp = streaming ? stream_body.torso : body_comp;
    auto &right_comp = streaming ? stream_body.right_arm : body_comp;
    auto &left_comp = streaming ? stream_body.left_arm : body_comp;
    bool use_body = false;
    std::string err_msg;

    if (!process_cartesian_part(goal->torso, "torso",
                                goal->stop_position_tracking_error,
                                goal->stop_orientation_tracking_error,
                                torso_comp, use_body, err_msg) ||
        !process_cartesian_part(goal->right_arm, "right_arm",
                                goal->stop_position_tracking_error,
                                goal->stop_orientation_tracking_error,
                                right_comp, use_body, err_msg) ||
        !process_cartesian_part(goal->left_arm, "left_arm",
                                goal->stop_position_tracking_error,
                                goal->stop_orientation_tracking_error,
                                left_comp, use_body, err_msg)) {

      result->success = false;
      result->finish_code = err_msg;
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    if (!use_body) {
      result->success = false;
      result->finish_code = "Empty targets";
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
      return;
    }

    component_cmd_builder.SetBodyCommand(rb::BodyCommandBuilder(body_comp));

    // ── STREAM MODE: send on the parts' streams, wait minimum_time ──
    if (streaming) {
      // Early-end check, switched off (2026-10-19 user decision): it ran 600 normal
      // commands without a false alarm and caught 10 of 10 forced ends (another client
      // taking the torso), but the robot's own early end did not come back in those 600.
      // Uncomment the four marked places below to turn it on again.
      // [early-end 1/4] The streams this command went out on.
      // const void *watched_torso = nullptr;
      // const void *watched_right_arm = nullptr;
      // const void *watched_left_arm = nullptr;
      {
        std::lock_guard<std::mutex> sl(stream_mutex_);
        double moves_for =
            std::max({goal->torso.minimum_time, goal->right_arm.minimum_time,
                      goal->left_arm.minimum_time});
        stream_body.parts =
            (goal->torso.ref_link.empty() ? 0 : STREAM_TORSO) |
            (goal->right_arm.ref_link.empty() ? 0 : STREAM_RIGHT_ARM) |
            (goal->left_arm.ref_link.empty() ? 0 : STREAM_LEFT_ARM);
        const std::string refused = send_body(
            stream_body,
            moves_for > 0.0 ? moves_for : kDefaultMinimumTime, kStreamSendWait, true);
        if (!refused.empty()) {
          result->success = false;
          result->finish_code = "Stream: " + refused;
          try {
            goal_handle->abort(result);
          } catch (...) {
          }
          return;
        }
        // [early-end 2/4]
        // if (stream_body.parts & STREAM_TORSO)
        //   watched_torso = torso_stream_.handler.get();
        // if (stream_body.parts & STREAM_RIGHT_ARM)
        //   watched_right_arm = right_arm_stream_.handler.get();
        // if (stream_body.parts & STREAM_LEFT_ARM)
        //   watched_left_arm = left_arm_stream_.handler.get();
      }
      double max_mt =
          std::max({goal->torso.minimum_time, goal->right_arm.minimum_time,
                    goal->left_arm.minimum_time});
      if (max_mt <= 0.0)
        max_mt = kDefaultMinimumTime;
      uint64_t my_seq = ++command_seq_;
      // [early-end 3/4] instead of the deadline line below:
      // const auto sent_at = std::chrono::steady_clock::now();
      // auto deadline = sent_at + std::chrono::duration<double>(max_mt);
      auto deadline = std::chrono::steady_clock::now() +
                      std::chrono::duration<double>(max_mt);
      rclcpp::Rate rate(100);
      while (rclcpp::ok()) {
        // [early-end 4/4] The robot sometimes ends a Cartesian command on its stream by
        // itself, long before minimum_time, and reports it done: the part stops short
        // (simulator 2026-10-19: 4 of 895 torso commands, after 0.03 to 1.97 s of 1 to
        // 3 s, with or without stop tracking errors). It then closes the stream. The last
        // 0.1 s is left to the robot's own end of the command.
        // if (deadline - std::chrono::steady_clock::now() > std::chrono::milliseconds(100)) {
        //   bool ended = false;
        //   {
        //     std::lock_guard<std::mutex> sl(stream_mutex_);
        //     auto gone = [](const BodyStream &stream, const void *watched) {
        //       return watched != nullptr && stream.handler.get() == watched &&
        //              stream.handler->IsDone();
        //     };
        //     ended = gone(torso_stream_, watched_torso) ||
        //             gone(right_arm_stream_, watched_right_arm) ||
        //             gone(left_arm_stream_, watched_left_arm);
        //   }
        //   if (ended) {
        //     const double after = std::chrono::duration<double>(
        //         std::chrono::steady_clock::now() - sent_at).count();
        //     RCLCPP_WARN(this->get_logger(),
        //                 "Rby1CartesianCommand: the robot ended it after %.2f s of "
        //                 "%.2f s (its stream closed); the part stopped short",
        //                 after, max_mt);
        //     result->success = false;
        //     result->finish_code = "kEndedEarly: the robot ended the command after " +
        //                           std::to_string(after).substr(0, 4) + " s of its " +
        //                           std::to_string(max_mt).substr(0, 4) + " s";
        //     try {
        //       goal_handle->abort(result);
        //     } catch (...) {
        //     }
        //     return;
        //   }
        // }
        if (command_seq_.load() != my_seq) {
          result->success = false;
          result->finish_code = "kPreempted";
          try {
            goal_handle->canceled(result);
          } catch (...) {
          }
          return;
        }
        if (goal_handle->is_canceling() || is_control_canceled_) {
          result->success = false;
          result->finish_code = "kCanceled";
          try {
            goal_handle->canceled(result);
          } catch (...) {
          }
          return;
        }
        if (std::chrono::steady_clock::now() >= deadline)
          break;
        rate.sleep();
      }
      result->success = true;
      result->finish_code = "kOk";
      try {
        goal_handle->succeed(result);
      } catch (...) {
      }
      return;
    }

    // ── NO STREAM: classic blocking SendCommand ──
    auto cmd_handler = robot_->SendCommand(
        rb::RobotCommandBuilder().SetCommand(component_cmd_builder),
        goal->priority);

    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !cmd_handler->IsDone()) {
      if (goal_handle->is_canceling()) {
        cmd_handler->Cancel();
        result->success = false;
        result->finish_code = "kCanceled";
        try {
          goal_handle->canceled(result);
        } catch (...) {
        }
        return;
      }

      auto cm_state = robot_->GetControlManagerState();
      if (cm_state.state == rb::ControlManagerState::State::kMajorFault ||
          cm_state.state == rb::ControlManagerState::State::kMinorFault) {
        cmd_handler->Cancel();
        result->success = false;
        result->finish_code = "Fault Detected";
        try {
          goal_handle->abort(result);
        } catch (...) {
        }
        return;
      }
      rate.sleep();
    }

    if (rclcpp::ok()) {
      auto rv = cmd_handler->Get();
      result->finish_code = this->finish_code_to_string(rv.finish_code());
      if (is_control_canceled_)
        result->finish_code = "kCanceled";
      result->success = (result->finish_code == "kOk");

      RCLCPP_INFO(this->get_logger(),
                  "Rby1CartesianCommand finished with code: %s",
                  result->finish_code.c_str());
      if (result->success) {
        try {
          goal_handle->succeed(result);
        } catch (...) {
        }
      } else {
        try {
          goal_handle->abort(result);
        } catch (...) {
        }
        this->check_controll_manager();
      }
    } else {
      // Shutting down with the goal still active: see execute_rby1_joint_command.
      result->success = false;
      result->finish_code = "kShutdown";
      try {
        goal_handle->abort(result);
      } catch (...) {
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in Rby1CartesianCommand: %s",
                 e.what());
    result->success = false;
    result->finish_code = "kError";
    try {
      goal_handle->abort(result);
    } catch (...) {
    }
    this->check_controll_manager();
  }
}

}  // namespace rby1_ros2
