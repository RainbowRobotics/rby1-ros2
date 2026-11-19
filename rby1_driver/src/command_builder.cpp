// Command building of the RB-Y1 driver: a JointCommand or CartesianCommand message into
// the SDK command builders, and the SDK finish code into text.
// Compiled as part of rby1_ros2_driver.cpp, which includes this file: the driver is a
// class template instantiated once per robot model there, not here.
#include "rby1_ros2_driver.hpp"

namespace rby1_ros2 {

template <typename ModelType>
std::string RBY1_ROS2_DRIVER<ModelType>::finish_code_to_string(
    rb::RobotCommandFeedback::FinishCode code) {
  switch (code) {
  case rb::RobotCommandFeedback::FinishCode::kUnknown:
    return "kUnknown";
  case rb::RobotCommandFeedback::FinishCode::kOk:
    return "kOk";
  case rb::RobotCommandFeedback::FinishCode::kCanceled:
    return "kCanceled";
  case rb::RobotCommandFeedback::FinishCode::kPreempted:
    return "kPreempted";
  case rb::RobotCommandFeedback::FinishCode::kInitializationFailed:
    return "kInitializationFailed";
  case rb::RobotCommandFeedback::FinishCode::kControlManagerIdle:
    return "kControlManagerIdle";
  case rb::RobotCommandFeedback::FinishCode::kControlManagerFault:
    return "kControlManagerFault";
  case rb::RobotCommandFeedback::FinishCode::kUnexpectedState:
    return "kUnexpectedState";
  default:
    return "Unknown";
  }
}

template <typename ModelType>
bool RBY1_ROS2_DRIVER<ModelType>::process_joint_part(
    const rby1_msgs::msg::JointCommand &cmd, const std::string &part_name,
    size_t expected_dof, rb::BodyComponentBasedCommandBuilder &body_comp,
    rb::ComponentBasedCommandBuilder &component_cmd_builder, bool &use_body,
    bool &use_head, std::string &err_msg) {
  if (cmd.position.empty())
    return true;

  double hold_time = cmd.control_hold_time;
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (stream_active_) {
      hold_time = kControlHoldTime;
    }
  }

  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(cmd.position.data(),
                                                        cmd.position.size());
  Eigen::VectorXd clamped_vel_limits(cmd.position.size());
  Eigen::VectorXd clamped_accel_limits(cmd.position.size());

  for (size_t i = 0; i < cmd.position.size(); ++i) {
    clamped_vel_limits[i] = cmd.velocity_limit;
    clamped_accel_limits[i] = cmd.acceleration_limit;
  }

  if (dynamics_ && dyn_state_ && q_lower_.size() > 0) {
    if (cmd.use_group_joint) {
      if (cmd.position.size() == cmd.joint_names.size()) {
        for (size_t i = 0; i < cmd.joint_names.size(); ++i) {
          std::string commanded_name = cmd.joint_names[i];

          int pinocchio_idx = -1;
          for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
            if (dyn_joint_names_[k] == commanded_name) {
              pinocchio_idx = k;
              break;
            }
          }
          if (pinocchio_idx == -1)
            continue;

          double pos = cmd.position[i];
          double min_p = q_lower_[pinocchio_idx];
          double max_p = q_upper_[pinocchio_idx];

          if (pos < min_p || pos > max_p) {
            err_msg = "Safety Limit Exceeded: Commanded position for joint '" +
                      commanded_name + "' (" + std::to_string(pos) +
                      " rad) is out of safety range [" + std::to_string(min_p) +
                      ", " + std::to_string(max_p) + "] rad.";
            RCLCPP_ERROR(this->get_logger(),
                         "\033[1;31m[SAFETY ERROR] %s\033[0m", err_msg.c_str());
            return false;
          }
        }
      }
    } else {
      const std::vector<unsigned int> *part_indices = nullptr;
      if (part_name == "torso")
        part_indices = &info_.torso_joint_idx;
      else if (part_name == "right_arm")
        part_indices = &info_.right_arm_joint_idx;
      else if (part_name == "left_arm")
        part_indices = &info_.left_arm_joint_idx;
      else if (part_name == "head")
        part_indices = &info_.head_joint_idx;

      if (part_indices && cmd.position.size() == part_indices->size()) {
        for (size_t i = 0; i < cmd.position.size(); ++i) {
          size_t global_idx = (*part_indices)[i];
          std::string joint_name = info_.joint_infos[global_idx].name;

          int pinocchio_idx = -1;
          for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
            if (dyn_joint_names_[k] == joint_name) {
              pinocchio_idx = k;
              break;
            }
          }
          if (pinocchio_idx == -1)
            continue;

          double pos = cmd.position[i];
          double min_p = q_lower_[pinocchio_idx];
          double max_p = q_upper_[pinocchio_idx];

          if (pos < min_p || pos > max_p) {
            err_msg = "Safety Limit Exceeded: Commanded position for joint '" +
                      joint_name + "' (" + std::to_string(pos) +
                      " rad) is out of safety range [" + std::to_string(min_p) +
                      ", " + std::to_string(max_p) + "] rad.";
            RCLCPP_ERROR(this->get_logger(),
                         "\033[1;31m[SAFETY ERROR] %s\033[0m", err_msg.c_str());
            return false;
          }
        }
      }
    }
  }

  if (cmd.use_group_joint && part_name == "torso") {
    if (cmd.joint_names.empty()) {
      err_msg = "Empty Joint Names: torso joint_names cannot be empty when "
                "use_group_joint is True.";
      RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
      return false;
    }
    if (cmd.position.size() != cmd.joint_names.size()) {
      err_msg = "Invalid Array Size: torso position size (" +
                std::to_string(cmd.position.size()) +
                ") must match joint_names size (" +
                std::to_string(cmd.joint_names.size()) +
                ") for Joint Group Control.";
      RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
      return false;
    }
    auto b = rb::JointGroupPositionCommandBuilder();
    b.SetJointNames(cmd.joint_names);
    b.SetPosition(q);
    b.SetMinimumTime(cmd.minimum_time);
    if (cmd.velocity_limit > 0.0) {
      b.SetVelocityLimit(clamped_vel_limits);
    }
    if (cmd.acceleration_limit > 0.0) {
      b.SetAccelerationLimit(clamped_accel_limits);
    }
    body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
    use_body = true;
    return true;
  }

  if (cmd.use_group_joint) {
    RCLCPP_WARN(this->get_logger(),
                "Joint group control is only supported for torso. Falling back "
                "to standard control for %s",
                part_name.c_str());
  }

  if (cmd.position.size() != expected_dof) {
    err_msg = "Invalid Array Size: " + part_name + " position requires " +
              std::to_string(expected_dof) + " elements, got " +
              std::to_string(cmd.position.size());
    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
    return false;
  }

  // Head early-return
  if (part_name == "head") {
    rb::JointPositionCommandBuilder b;
    make_joint_pos_builder(b, q, cmd.minimum_time, hold_time,
                           cmd.velocity_limit, cmd.acceleration_limit);
    component_cmd_builder.SetHeadCommand(rb::HeadCommandBuilder(b));
    use_head = true;
    return true;
  }

  if (cmd.use_impedance) {
    std::vector<double> stiffness = cmd.stiffness;
    if (stiffness.empty()) {
      stiffness.assign(expected_dof, kDefaultJointStiffness);
    } else if (stiffness.size() != expected_dof) {
      err_msg = "Invalid Array Size: " + part_name + " stiffness requires " +
                std::to_string(expected_dof) + " elements, got " +
                std::to_string(stiffness.size());
      RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
      return false;
    }
    Eigen::VectorXd stiffness_vec =
        Eigen::Map<const Eigen::VectorXd>(stiffness.data(), stiffness.size());

    rb::JointImpedanceControlCommandBuilder b;
    make_joint_impedance_builder(
        b, q, cmd.minimum_time, hold_time, stiffness_vec, cmd.damping_ratio,
        cmd.torque_limit, cmd.velocity_limit, cmd.acceleration_limit);

    if (part_name == "torso")
      body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
    else if (part_name == "right_arm")
      body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "left_arm")
      body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
    use_body = true;
  } else {
    rb::JointPositionCommandBuilder b;
    make_joint_pos_builder(b, q, cmd.minimum_time, hold_time,
                           cmd.velocity_limit, cmd.acceleration_limit);

    if (part_name == "torso")
      body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));
    else if (part_name == "right_arm")
      body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "left_arm")
      body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
    use_body = true;
  }
  return true;
}

template <typename ModelType>
bool RBY1_ROS2_DRIVER<ModelType>::process_cartesian_part(
    const rby1_msgs::msg::CartesianCommand &cmd, const std::string &part_name,
    double stop_position_tracking_error, double stop_orientation_tracking_error,
    rb::BodyComponentBasedCommandBuilder &body_comp, bool &use_body,
    std::string &err_msg) {
  if (cmd.ref_link.empty() && cmd.target_link.empty())
    return true;

  double hold_time = cmd.control_hold_time;
  {
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    if (stream_active_) {
      hold_time = kControlHoldTime;
    }
  }

  if (cmd.ref_link.empty() || cmd.target_link.empty()) {
    err_msg = "Invalid CartesianCommand: Both ref_link and target_link must be "
              "provided for " +
              part_name;
    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
    return false;
  }

  auto get_weight = [](const std::vector<double> &input, double default_val,
                       const std::string &err_prefix,
                       std::string &err) -> std::optional<Eigen::Vector3d> {
    if (input.empty())
      return Eigen::Vector3d(default_val, default_val, default_val);
    if (input.size() != 3) {
      err = err_prefix + " requires 3 elements, got " +
            std::to_string(input.size());
      return std::nullopt;
    }
    return Eigen::Vector3d(input[0], input[1], input[2]);
  };

  auto t_weight = get_weight(
      cmd.translation_weight, kTranslationWeight,
      "Invalid Array Size: " + part_name + " translation_weight", err_msg);
  if (!t_weight)
    return false;

  auto r_weight = get_weight(
      cmd.rotation_weight, kRotationWeight,
      "Invalid Array Size: " + part_name + " rotation_weight", err_msg);
  if (!r_weight)
    return false;

  Eigen::Quaterniond q(cmd.transform.rotation.w, cmd.transform.rotation.x,
                       cmd.transform.rotation.y, cmd.transform.rotation.z);
  Eigen::Vector3d t(cmd.transform.translation.x, cmd.transform.translation.y,
                    cmd.transform.translation.z);

  rb::math::SE3::MatrixType T = rb::math::SE3::MatrixType::Identity();
  T.block(0, 0, 3, 3) = q.toRotationMatrix();
  T.block(0, 3, 3, 1) = t;

  if (cmd.use_impedance) {
    auto b = rb::ImpedanceControlCommandBuilder();
    b.SetCommandHeader(
        rb::CommandHeaderBuilder().SetControlHoldTime(hold_time));
    b.SetReferenceLinkName(cmd.ref_link);
    b.SetLinkName(cmd.target_link);
    b.SetTranslationWeight(*t_weight);
    b.SetRotationWeight(*r_weight);
    b.SetTransformation(T);

    if (cmd.add_joint_position_target) {
      RCLCPP_WARN(this->get_logger(),
                  "add_joint_position_target is not supported for Cartesian "
                  "Impedance Control in this SDK version. Ignoring for %s",
                  part_name.c_str());
    }

    if (part_name == "right_arm")
      body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "left_arm")
      body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "torso")
      body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));

    use_body = true;
    return true;
  } else {
    auto b = rb::CartesianCommandBuilder();
    b.SetCommandHeader(
        rb::CommandHeaderBuilder().SetControlHoldTime(hold_time));
    b.SetMinimumTime(cmd.minimum_time);

    double lin_vel = (cmd.linear_velocity_limit <= 0.0) ? kLinearVelocityLimit
                                                        : cmd.linear_velocity_limit;
    double ang_vel = (cmd.angular_velocity_limit <= 0.0) ? kAngularVelocityLimit
                                                         : cmd.angular_velocity_limit;
    double acc_scale = (cmd.acceleration_limit_scaling <= 0.0)
                           ? kAccelerationLimit
                           : cmd.acceleration_limit_scaling;

    b.AddTarget(cmd.ref_link, cmd.target_link, T, lin_vel, ang_vel, acc_scale);
    b.SetStopPositionTrackingError(stop_position_tracking_error);
    b.SetStopOrientationTrackingError(stop_orientation_tracking_error);

    if (cmd.add_joint_position_target) {
      int pinocchio_idx = -1;
      for (size_t k = 0; k < dyn_joint_names_.size(); ++k) {
        if (dyn_joint_names_[k] == cmd.add_joint_name) {
          pinocchio_idx = k;
          break;
        }
      }
      if (pinocchio_idx != -1) {
        double min_p = q_lower_[pinocchio_idx];
        double max_p = q_upper_[pinocchio_idx];
        if (cmd.add_joint_value < min_p || cmd.add_joint_value > max_p) {
          err_msg = "Safety Limit Exceeded: Commanded additional joint '" +
                    cmd.add_joint_name + "' value (" +
                    std::to_string(cmd.add_joint_value) +
                    " rad) is out of safety range [" + std::to_string(min_p) +
                    ", " + std::to_string(max_p) + "] rad.";
          RCLCPP_ERROR(this->get_logger(), "\033[1;31m[SAFETY ERROR] %s\033[0m",
                       err_msg.c_str());
          return false;
        }
      }
      b.AddJointPositionTarget(cmd.add_joint_name, cmd.add_joint_value);
    }

    if (part_name == "right_arm")
      body_comp.SetRightArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "left_arm")
      body_comp.SetLeftArmCommand(rb::ArmCommandBuilder(b));
    else if (part_name == "torso")
      body_comp.SetTorsoCommand(rb::TorsoCommandBuilder(b));

    use_body = true;
    return true;
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::make_joint_pos_builder(
    rb::JointPositionCommandBuilder &builder, const Eigen::VectorXd &q,
    double minimum_time, double hold_time, double vel_limit, double acc_limit) {
  rb::CommandHeaderBuilder header;
  header.SetControlHoldTime(hold_time);

  builder.SetCommandHeader(header).SetPosition(q).SetMinimumTime(minimum_time);

  if (vel_limit > 0.0) {
    builder.SetVelocityLimit(Eigen::VectorXd::Constant(q.size(), vel_limit));
  }
  if (acc_limit > 0.0) {
    builder.SetAccelerationLimit(
        Eigen::VectorXd::Constant(q.size(), acc_limit));
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::make_joint_impedance_builder(
    rb::JointImpedanceControlCommandBuilder &builder, const Eigen::VectorXd &q,
    double minimum_time, double hold_time, const Eigen::VectorXd &stiffness,
    double damping_ratio, double torque_limit, double vel_limit,
    double acc_limit) {
  rb::CommandHeaderBuilder header;
  header.SetControlHoldTime(hold_time);

  builder.SetCommandHeader(header)
      .SetPosition(q)
      .SetMinimumTime(minimum_time)
      .SetStiffness(stiffness)
      .SetDampingRatio(damping_ratio)
      .SetTorqueLimit(Eigen::VectorXd::Constant(q.size(), torque_limit));

  if (vel_limit > 0.0) {
    builder.SetVelocityLimit(Eigen::VectorXd::Constant(q.size(), vel_limit));
  }
  if (acc_limit > 0.0) {
    builder.SetAccelerationLimit(
        Eigen::VectorXd::Constant(q.size(), acc_limit));
  }
}

template <typename ModelType>
void RBY1_ROS2_DRIVER<ModelType>::build_mobility_cmd(
    rb::MobilityCommandBuilder &builder, double vx, double vy, double wz,
    double minimum_time) {

  Eigen::Vector2d linear_vel(vx, vy);
  Eigen::Vector2d acceleration_limit(
      params_.se2_linear_acceleration_limit,
      params_.se2_linear_acceleration_limit);

  rb::SE2VelocityCommandBuilder se2_cmd;
  se2_cmd.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(kMobilityHoldTime));
  se2_cmd.SetMinimumTime(minimum_time);
  se2_cmd.SetVelocity(linear_vel, wz);
  se2_cmd.SetAccelerationLimit(acceleration_limit,
                               params_.se2_angular_acceleration_limit);

  builder.SetCommand(se2_cmd);
}

}  // namespace rby1_ros2
