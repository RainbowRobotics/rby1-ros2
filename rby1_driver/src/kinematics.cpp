// Kinematics of the RB-Y1 driver: the inverse kinematics behind Cartesian commands, and
// poses as messages.
// Compiled as part of rby1_ros2_driver.cpp, which includes this file: the driver is a
// class template instantiated once per robot model there, not here.
#include "rby1_ros2_driver.hpp"

namespace rby1_ros2 {

template <typename ModelType>
geometry_msgs::msg::Pose
RBY1_ROS2_DRIVER<ModelType>::matrix_to_pose(const Eigen::Matrix4d &matrix) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

template <typename ModelType>
std::optional<Eigen::VectorXd> RBY1_ROS2_DRIVER<ModelType>::solve_cartesian_ik(
    const std::vector<
        typename rb::OptimalControl<ModelType::kRobotDOF>::LinkTarget>
        &link_targets) {

  if (!dynamics_ || !dyn_state_) {
    return std::nullopt;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Clone current state as starting position for numerical IK
  auto temp_state =
      std::make_shared<rb::dyn::State<ModelType::kRobotDOF>>(*dyn_state_);
  temp_state->SetQdot(Eigen::Vector<double, ModelType::kRobotDOF>::Zero());

  // Print starting joint configuration for debugging
  std::stringstream ss_q;
  ss_q << "[";
  for (int i = 0; i < temp_state->GetQ().size(); ++i) {
    ss_q << temp_state->GetQ()[i]
         << (i == temp_state->GetQ().size() - 1 ? "" : ", ");
  }
  ss_q << "]";
  RCLCPP_DEBUG(this->get_logger(), "IK start Q: %s", ss_q.str().c_str());

  // Setup OptimalControl solver
  // Exclude joints with near-infinite limits (e.g. wheel joints whose URDF
  // limit is DBL_MAX). Including them makes the QP ill-conditioned and causes
  // divergence unrelated to the arm workspace being checked.
  std::vector<unsigned int> joint_idx;
  joint_idx.reserve(ModelType::kRobotDOF);
  for (unsigned int i = 0; i < ModelType::kRobotDOF; ++i) {
    if (q_lower_[i] > -kFiniteLimitThreshold &&
        q_upper_[i] < kFiniteLimitThreshold) {
      joint_idx.push_back(i);
    }
  }
  if (joint_idx.empty()) {
    // Fallback: use all joints if none passed the filter
    joint_idx.resize(ModelType::kRobotDOF);
    std::iota(joint_idx.begin(), joint_idx.end(), 0);
  }
  rb::OptimalControl<ModelType::kRobotDOF> ik_solver(dynamics_, joint_idx);

  double dt = kIkStep;
  int max_iterations = kIkMaxIterations;
  double tolerance = kIkTolerance;
  bool converged = false;

  typename rb::OptimalControl<ModelType::kRobotDOF>::Input in;
  in.link_targets = link_targets;
  // Stay near the joints the arm has now. Without it nothing holds a joint the pose does
  // not need: next to a singular arm shape (the wrist straight, as at the ready pose) the
  // solver turned two wrist joints against each other by a radian for a millimetre of the
  // hand. Weighted 1 against 1000 (position) and 100 (orientation): it does not pull the
  // pose off by anything that matters.
  typename rb::OptimalControl<ModelType::kRobotDOF>::JointAngleTarget stay;
  stay.q = temp_state->GetQ();
  stay.weight.setConstant(kPostureWeight);
  in.q_target = stay;
  // The pose is reached within these; the solver's own error also counts the term above.
  auto reached = [&]() {
    for (const auto &t : link_targets) {
      const Eigen::Matrix4d T_err =
          dynamics_->ComputeTransformation(temp_state, t.ref_link_index,
                                           t.link_index)
              .inverse() *
          t.T;
      const double turned = std::acos(std::clamp(
          (T_err.block<3, 3>(0, 0).trace() - 1.0) / 2.0, -1.0, 1.0));
      if (T_err.block<3, 1>(0, 3).norm() > kIkPositionTolerance ||
          turned > kIkOrientationTolerance)
        return false;
    }
    return true;
  };

  Eigen::Vector<double, ModelType::kRobotDOF> q_lb =
      q_lower_ - Eigen::Vector<double, ModelType::kRobotDOF>::Constant(kIkJointLimitMargin);
  Eigen::Vector<double, ModelType::kRobotDOF> q_ub =
      q_upper_ + Eigen::Vector<double, ModelType::kRobotDOF>::Constant(kIkJointLimitMargin);
  Eigen::Vector<double, ModelType::kRobotDOF> qdot_ub =
      Eigen::Vector<double, ModelType::kRobotDOF>::Constant(kIkVelocityBound);
  Eigen::Vector<double, ModelType::kRobotDOF> qddot_ub =
      Eigen::Vector<double, ModelType::kRobotDOF>::Constant(kIkAccelerationBound);

  auto debug_joint_names = temp_state->GetJointNames();
  for (size_t i = 0; i < debug_joint_names.size(); ++i) {
    RCLCPP_DEBUG(this->get_logger(),
                "Joint %zu (%s): Q=%.4f, Limit=[%.4f, %.4f]", i,
                std::string(debug_joint_names[i]).c_str(),
                temp_state->GetQ()[i], q_lb[i], q_ub[i]);
  }

  // Compute and print initial T_cur and T_err for targets
  for (size_t i = 0; i < link_targets.size(); ++i) {
    const auto &t = link_targets[i];
    Eigen::Matrix4d T_cur = dynamics_->ComputeTransformation(
        temp_state, t.ref_link_index, t.link_index);
    Eigen::Matrix4d T_err = T_cur.inverse() * t.T;
    RCLCPP_DEBUG(this->get_logger(),
                "Target %zu initial T_cur translation: [%.4f, %.4f, %.4f]", i,
                T_cur(0, 3), T_cur(1, 3), T_cur(2, 3));
    RCLCPP_DEBUG(this->get_logger(),
                "Target %zu initial T_err translation: [%.4f, %.4f, %.4f]", i,
                T_err(0, 3), T_err(1, 3), T_err(2, 3));
  }

  int final_iter = 0;
  for (int iter = 0; iter < max_iterations; ++iter) {
    final_iter = iter;
    auto sol = ik_solver.Solve(in, temp_state, dt, 1.0, q_lb, q_ub, qdot_ub,
                               qddot_ub, true);
    if (!sol.has_value()) {
      RCLCPP_WARN(
          this->get_logger(),
          "Iteration %d: Solve returned no value. ExitCode: %d, Msg: %s", iter,
          (int)ik_solver.GetExitCode(), ik_solver.GetExitCodeMessage().c_str());
      break;
    }

    // Clamp joint velocities to velocity limits to ensure numerical stability
    Eigen::VectorXd clamped_sol =
        sol.value().cwiseMax(-qdot_ub).cwiseMin(qdot_ub);

    // Solve() has just computed the kinematics of the pose before this step.
    if (reached()) {
      converged = true;
      break;
    }

    // One Newton step of the pose, and at rest again. The solver's cost is
    // J (qdot_old + qdot_new) / 2 * dt = pose error (trapezoidal); carrying qdot_new over
    // as the next qdot_old made the next step undo this one and its error never fell:
    // every target that was not the pose already held ended in "IK did not converge".
    Eigen::VectorXd q_next =
        temp_state->GetQ() + 0.5 * (temp_state->GetQdot() + clamped_sol) * dt;
    const double moved = (q_next - temp_state->GetQ()).cwiseAbs().maxCoeff();
    temp_state->SetQ(q_next);
    temp_state->SetQdot(Eigen::Vector<double, ModelType::kRobotDOF>::Zero());
    if (moved < kIkStallStep)
      break;  // no further: the pose is out of reach
  }
  RCLCPP_DEBUG(this->get_logger(),
              "IK loop finished at iteration %d, converged: %d", final_iter,
              converged);

  if (converged) {
    return temp_state->GetQ();
  }

  RCLCPP_WARN(this->get_logger(),
              "IK solver failed to converge. Targets count: %zu",
              link_targets.size());
  for (size_t i = 0; i < link_targets.size(); ++i) {
    const auto &t = link_targets[i];
    Eigen::Matrix4d T_mat = t.T;
    RCLCPP_WARN(this->get_logger(),
                "  Target %zu: ref_link_idx=%d, link_idx=%d", i,
                t.ref_link_index, t.link_index);
    RCLCPP_WARN(this->get_logger(), "    T translation: [%.4f, %.4f, %.4f]",
                T_mat(0, 3), T_mat(1, 3), T_mat(2, 3));
    Eigen::Matrix3d R_mat = T_mat.block(0, 0, 3, 3);
    Eigen::Quaterniond q_rot(R_mat);
    RCLCPP_WARN(this->get_logger(),
                "    T rotation: x=%.4f, y=%.4f, z=%.4f, w=%.4f", q_rot.x(),
                q_rot.y(), q_rot.z(), q_rot.w());
  }
  RCLCPP_WARN(this->get_logger(),
              "  Final solver error: %.6f (tolerance: %.6f)",
              ik_solver.GetError(), tolerance);

  return std::nullopt;
}

}  // namespace rby1_ros2
