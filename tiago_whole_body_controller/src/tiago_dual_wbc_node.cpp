#include "tiago_whole_body_controller/tiago_dual_wbc_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tiago_wbc
{

TiagoDualArmWholeBodyController::TiagoDualArmWholeBodyController(const rclcpp::NodeOptions & options)
: Node("tiago_dual_wbc_controller", options)
{
  const std::string urdf_xml = declare_parameter<std::string>("robot_description", "");
  const std::string urdf_path = declare_parameter<std::string>("urdf_path", "");
  const double rate_hz = declare_parameter<double>("control_rate_hz", 100.0);
  const std::string left_ee_name = declare_parameter<std::string>("left_ee_frame", "arm_left_tool_link");
  const std::string right_ee_name = declare_parameter<std::string>("right_ee_frame", "arm_right_tool_link");

  kp_lin_ = declare_parameter<double>("kp_linear", 3.0);
  kp_ang_ = declare_parameter<double>("kp_angular", 3.0);
  k_null_ = declare_parameter<double>("k_null", 1.5);
  base_standoff_x_ = declare_parameter<double>("base_standoff_x", 0.85);
  wbc_enabled_ = declare_parameter<bool>("start_enabled", false);

  const auto w_task_vec = declare_parameter<std::vector<double>>(
    "w_task", {100.0, 100.0, 100.0, 50.0, 50.0, 50.0, 100.0, 100.0, 100.0, 50.0, 50.0, 50.0});
  const auto w_reg_vec = declare_parameter<std::vector<double>>(
    "w_reg", {2.5, 2.5, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  const auto dq_max_vec = declare_parameter<std::vector<double>>(
    "dq_max", {0.4, 0.8, 0.07, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

  dt_ = 1.0 / rate_hz;

  if (!urdf_xml.empty()) {
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
  } else if (!urdf_path.empty()) {
    pinocchio::urdf::buildModel(urdf_path, model_);
  } else {
    throw std::runtime_error("Either 'robot_description' XML or 'urdf_path' must be provided.");
  }

  data_ = pinocchio::Data(model_);
  left_ee_id_ = model_.getFrameId(left_ee_name);
  right_ee_id_ = model_.getFrameId(right_ee_name);

  if (left_ee_id_ >= static_cast<pinocchio::FrameIndex>(model_.nframes) ||
      right_ee_id_ >= static_cast<pinocchio::FrameIndex>(model_.nframes)) {
    throw std::runtime_error("End-effector frame not found in URDF model.");
  }

  joint_names_.reserve(kUpperJoints);
  joint_names_.push_back("torso_lift_joint");
  for (int i = 1; i <= 7; ++i) {
    const std::string name = "arm_left_" + std::to_string(i) + "_joint";
    left_joint_names_.push_back(name);
    joint_names_.push_back(name);
  }
  for (int i = 1; i <= 7; ++i) {
    const std::string name = "arm_right_" + std::to_string(i) + "_joint";
    right_joint_names_.push_back(name);
    joint_names_.push_back(name);
  }

  for (std::size_t i = 0; i < kUpperJoints; ++i) {
    const auto jid = model_.getJointId(joint_names_[i]);
    if (jid == 0) {
      throw std::runtime_error("Joint not found in URDF: " + joint_names_[i]);
    }
    q_idx_[i] = model_.joints[jid].idx_q();
    v_idx_[i] = model_.joints[jid].idx_v();
    joint_name_to_upper_idx_[joint_names_[i]] = i;
  }

  q_current_ = pinocchio::neutral(model_);
  q_cmd_upper_.setZero();
  // Mirrored inward-elbow nominal posture at Level 3 torso height (0.28 m)
  q_nominal_ << 0.28,
    0.35, -0.55, -0.35, 1.65, -1.57, 0.15, 0.0,
   -0.35, -0.55,  0.35, 1.65,  1.57, 0.15, 0.0;

  W_task_.setZero();
  for (int i = 0; i < kTaskDim; ++i) {
    W_task_(i, i) = w_task_vec[static_cast<std::size_t>(i)];
  }

  W_reg_.setZero();
  for (int i = 0; i < kNumDofs; ++i) {
    W_reg_(i, i) = w_reg_vec[static_cast<std::size_t>(i)];
    dq_max_[i] = dq_max_vec[static_cast<std::size_t>(i)];
    dq_min_[i] = -dq_max_[i];
  }

  // Pre-allocate OSQP workspace once
  P_sparse_.resize(kNumDofs, kNumDofs);
  A_sparse_.resize(kNumDofs, kNumDofs);
  A_sparse_.setIdentity();
  H_dense_ = Matrix17d::Identity();
  P_sparse_ = H_dense_.sparseView();
  q_vec_.setZero();
  l_bound_ = dq_min_;
  u_bound_ = dq_max_;

  solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setPolish(true);
  solver_.data()->setNumberOfVariables(kNumDofs);
  solver_.data()->setNumberOfConstraints(kNumDofs);

  if (!solver_.data()->setHessianMatrix(P_sparse_) ||
      !solver_.data()->setGradient(q_vec_) ||
      !solver_.data()->setLinearConstraintsMatrix(A_sparse_) ||
      !solver_.data()->setLowerBound(l_bound_) ||
      !solver_.data()->setUpperBound(u_bound_) ||
      !solver_.initSolver()) {
    throw std::runtime_error("Failed to initialize OSQP-Eigen solver.");
  }
  solver_initialized_ = true;

  // TF2 Buffer & Listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ROS 2 Subscribers & Publishers
  enable_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/wbc/enable", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data && !wbc_enabled_) {
        for (std::size_t i = 0; i < kUpperJoints; ++i) {
          q_cmd_upper_[static_cast<Eigen::Index>(i)] = q_current_[q_idx_[i]];
        }
        RCLCPP_INFO(get_logger(), "QP-WBC ENABLED (synced to current joint positions).");
      } else if (!msg->data && wbc_enabled_) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        RCLCPP_INFO(get_logger(), "QP-WBC PAUSED.");
      }
      wbc_enabled_ = msg->data;
    });

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&TiagoDualArmWholeBodyController::jointCallback, this, std::placeholders::_1));
  left_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/wbc/left_ee_target", 10, std::bind(&TiagoDualArmWholeBodyController::leftTargetCallback, this, std::placeholders::_1));
  right_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/wbc/right_ee_target", 10, std::bind(&TiagoDualArmWholeBodyController::rightTargetCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/mobile_base_controller/cmd_vel", 10);
  torso_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/torso_controller/joint_trajectory", 10);
  left_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_left_controller/joint_trajectory", 10);
  right_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_right_controller/joint_trajectory", 10);
  telemetry_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wbc/telemetry", 10);

  const auto period = std::chrono::duration<double>(dt_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TiagoDualArmWholeBodyController::controlLoop, this));

  RCLCPP_INFO(
    get_logger(), "C++ 17-DOF Dual-Arm QP-WBC initialized at %.1f Hz (start_enabled=%s).",
    rate_hz, wbc_enabled_ ? "true" : "false");
}

void TiagoDualArmWholeBodyController::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const std::size_t n = std::min(msg->name.size(), msg->position.size());
  for (std::size_t i = 0; i < n; ++i) {
    const auto it = joint_name_to_upper_idx_.find(msg->name[i]);
    if (it != joint_name_to_upper_idx_.end()) {
      q_current_[q_idx_[it->second]] = msg->position[i];
      if (!cmd_initialized_) {
        q_cmd_upper_[static_cast<Eigen::Index>(it->second)] = msg->position[i];
      }
    }
  }
  has_joint_state_ = true;
  cmd_initialized_ = true;
}

pinocchio::SE3 TiagoDualArmWholeBodyController::poseMsgToSE3(const geometry_msgs::msg::PoseStamped & msg)
{
  const auto & p = msg.pose.position;
  const auto & o = msg.pose.orientation;
  Eigen::Quaterniond quat(o.w, o.x, o.y, o.z);
  quat.normalize();
  return pinocchio::SE3(quat.toRotationMatrix(), Eigen::Vector3d(p.x, p.y, p.z));
}

void TiagoDualArmWholeBodyController::leftTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  left_target_msg_ = *msg;
  if (left_target_msg_.header.frame_id.empty()) {
    left_target_msg_.header.frame_id = "base_footprint";
  }
  has_left_target_ = true;
}

void TiagoDualArmWholeBodyController::rightTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  right_target_msg_ = *msg;
  if (right_target_msg_.header.frame_id.empty()) {
    right_target_msg_.header.frame_id = "base_footprint";
  }
  has_right_target_ = true;
}

Eigen::Matrix<double, 6, 2> TiagoDualArmWholeBodyController::computeBaseJacobian(const pinocchio::SE3 & ee_pose)
{
  Eigen::Matrix<double, 6, 2> J_base;
  J_base.setZero();
  J_base(0, 0) = 1.0;
  J_base(0, 1) = -ee_pose.translation().y();
  J_base(1, 1) = ee_pose.translation().x();
  J_base(5, 1) = 1.0;
  return J_base;
}

TiagoDualArmWholeBodyController::Vector6d TiagoDualArmWholeBodyController::computeTwistError(
  const pinocchio::SE3 & current_pose, const pinocchio::SE3 & target_pose) const
{
  const auto err_se3 = pinocchio::log6(current_pose.actInv(target_pose)).toVector();
  const Eigen::Matrix3d & R = current_pose.rotation();
  Vector6d v_des;
  v_des.head<3>() = kp_lin_ * (R * err_se3.head<3>());
  v_des.tail<3>() = kp_ang_ * (R * err_se3.tail<3>());
  return v_des;
}

void TiagoDualArmWholeBodyController::publishSinglePointTrajectory(
  const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & pub,
  const std::vector<std::string> & names,
  const double * pos_ptr,
  const double * vel_ptr) const
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = names;

  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions.assign(pos_ptr, pos_ptr + names.size());
  pt.velocities.assign(vel_ptr, vel_ptr + names.size());
  pt.time_from_start = rclcpp::Duration::from_seconds(dt_);

  traj.points.push_back(std::move(pt));
  pub->publish(traj);
}

void TiagoDualArmWholeBodyController::controlLoop()
{
  if (!wbc_enabled_ || !has_joint_state_ || !has_left_target_ || !has_right_target_ || !solver_initialized_) {
    return;
  }

  const auto t_start = std::chrono::steady_clock::now();

  geometry_msgs::msg::PoseStamped left_in_base = left_target_msg_;
  geometry_msgs::msg::PoseStamped right_in_base = right_target_msg_;
  left_in_base.header.stamp = rclcpp::Time(0);
  right_in_base.header.stamp = rclcpp::Time(0);

  try {
    if (left_in_base.header.frame_id != "base_footprint") {
      left_in_base = tf_buffer_->transform(left_in_base, "base_footprint", tf2::durationFromSec(0.02));
    }
    if (right_in_base.header.frame_id != "base_footprint") {
      right_in_base = tf_buffer_->transform(right_in_base, "base_footprint", tf2::durationFromSec(0.02));
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF2 lookup failed: %s", ex.what());
    return;
  }

  const pinocchio::SE3 left_target = poseMsgToSE3(left_in_base);
  const pinocchio::SE3 right_target = poseMsgToSE3(right_in_base);

  pinocchio::forwardKinematics(model_, data_, q_current_);
  pinocchio::updateFramePlacements(model_, data_);

  const pinocchio::SE3 & left_pose = data_.oMf[left_ee_id_];
  const pinocchio::SE3 & right_pose = data_.oMf[right_ee_id_];

  Eigen::MatrixXd J_full_L(6, model_.nv);
  Eigen::MatrixXd J_full_R(6, model_.nv);
  J_full_L.setZero();
  J_full_R.setZero();

  pinocchio::computeFrameJacobian(
    model_, data_, q_current_, left_ee_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_L);
  pinocchio::computeFrameJacobian(
    model_, data_, q_current_, right_ee_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_R);

  J_wb_.setZero();
  J_wb_.block<6, 2>(0, 0) = computeBaseJacobian(left_pose);
  J_wb_.block<6, 2>(6, 0) = computeBaseJacobian(right_pose);

  for (int k = 0; k < kUpperJoints; ++k) {
    J_wb_.block<6, 1>(0, k + 2) = J_full_L.col(v_idx_[static_cast<std::size_t>(k)]);
    J_wb_.block<6, 1>(6, k + 2) = J_full_R.col(v_idx_[static_cast<std::size_t>(k)]);
  }

  Vector12d v_des;
  v_des.head<6>() = computeTwistError(left_pose, left_target);
  v_des.tail<6>() = computeTwistError(right_pose, right_target);

  Vector17d dq_null;
  dq_null.setZero();
  for (int k = 0; k < kUpperJoints; ++k) {
    dq_null[k + 2] = k_null_ * (q_nominal_[k] - q_current_[q_idx_[static_cast<std::size_t>(k)]]);
  }

  H_dense_.noalias() = J_wb_.transpose() * W_task_ * J_wb_ + W_reg_;
  q_vec_.noalias() = -(J_wb_.transpose() * W_task_ * v_des + W_reg_ * dq_null);

  for (int r = 0; r < kNumDofs; ++r) {
    for (int c = 0; c < kNumDofs; ++c) {
      P_sparse_.coeffRef(r, c) = H_dense_(r, c);
    }
  }

  l_bound_ = dq_min_;
  u_bound_ = dq_max_;

  // Shelf standoff barrier: prevent forward base motion once target is within arm reach
  const double min_target_x = std::min(left_target.translation().x(), right_target.translation().x());
  if (min_target_x > 0.0 && min_target_x <= base_standoff_x_) {
    u_bound_[0] = 0.0;
  } else if (min_target_x > base_standoff_x_) {
    const double max_approach_vx = 1.5 * (min_target_x - base_standoff_x_);
    u_bound_[0] = std::min(u_bound_[0], max_approach_vx);
  }

  for (int k = 0; k < kUpperJoints; ++k) {
    const int idx = q_idx_[static_cast<std::size_t>(k)];
    const double lower_lim = model_.lowerPositionLimit[idx];
    const double upper_lim = model_.upperPositionLimit[idx];

    if (std::isfinite(lower_lim) && std::isfinite(upper_lim) && (upper_lim - lower_lim) > 0.04) {
      const double margin = 0.01;
      const double q_min = lower_lim + margin;
      const double q_max = upper_lim - margin;

      const double v_low_lim = (q_min - q_current_[idx]) / dt_;
      const double v_high_lim = (q_max - q_current_[idx]) / dt_;

      l_bound_[k + 2] = std::clamp(v_low_lim, dq_min_[k + 2], dq_max_[k + 2]);
      u_bound_[k + 2] = std::clamp(v_high_lim, dq_min_[k + 2], dq_max_[k + 2]);

      if (l_bound_[k + 2] > u_bound_[k + 2]) {
        std::swap(l_bound_[k + 2], u_bound_[k + 2]);
      }
    }
  }

  solver_.updateHessianMatrix(P_sparse_);
  solver_.updateGradient(q_vec_);
  solver_.updateBounds(l_bound_, u_bound_);

  if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "OSQP solve failed; holding position.");
    return;
  }

  const Eigen::VectorXd & dq_opt = solver_.getSolution();

  for (int k = 0; k < kUpperJoints; ++k) {
    const int idx = q_idx_[static_cast<std::size_t>(k)];
    const double lower_lim = model_.lowerPositionLimit[idx];
    const double upper_lim = model_.upperPositionLimit[idx];

    double q_next = q_cmd_upper_[k] + dq_opt[k + 2] * dt_;
    if (std::isfinite(lower_lim) && std::isfinite(upper_lim) && (upper_lim - lower_lim) > 0.04) {
      q_next = std::clamp(q_next, lower_lim + 0.01, upper_lim - 0.01);
    }
    q_cmd_upper_[k] = q_next;
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = dq_opt[0];
  twist.angular.z = dq_opt[1];
  cmd_vel_pub_->publish(twist);

  publishSinglePointTrajectory(torso_traj_pub_, torso_joint_names_, q_cmd_upper_.data(), dq_opt.data() + 2);
  publishSinglePointTrajectory(left_traj_pub_, left_joint_names_, q_cmd_upper_.data() + 1, dq_opt.data() + 3);
  publishSinglePointTrajectory(right_traj_pub_, right_joint_names_, q_cmd_upper_.data() + 8, dq_opt.data() + 10);

  const double solve_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
  std_msgs::msg::Float64MultiArray telem;
  telem.data = {
    solve_ms,
    v_des.head<3>().norm() / kp_lin_,
    v_des.segment<3>(6).norm() / kp_lin_
  };
  telemetry_pub_->publish(telem);
}

}  // namespace tiago_wbc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tiago_wbc::TiagoDualArmWholeBodyController>());
  rclcpp::shutdown();
  return 0;
}