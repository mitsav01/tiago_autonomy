#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace tiago_wbc
{

class TiagoDualArmWholeBodyController : public rclcpp::Node
{
public:
  explicit TiagoDualArmWholeBodyController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  static constexpr int kNumDofs = 17;      // [vx, wz, torso(1), left_arm(7), right_arm(7)]
  static constexpr int kUpperJoints = 15;  // [torso(1), left_arm(7), right_arm(7)]
  static constexpr int kTaskDim = 12;      // [left_ee(6), right_ee(6)]

  using Matrix12x17d = Eigen::Matrix<double, kTaskDim, kNumDofs>;
  using Matrix17d = Eigen::Matrix<double, kNumDofs, kNumDofs>;
  using Matrix12d = Eigen::Matrix<double, kTaskDim, kTaskDim>;
  using Vector17d = Eigen::Matrix<double, kNumDofs, 1>;
  using Vector15d = Eigen::Matrix<double, kUpperJoints, 1>;
  using Vector12d = Eigen::Matrix<double, kTaskDim, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void leftTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void rightTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void controlLoop();

  void publishSinglePointTrajectory(
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & pub,
    const std::vector<std::string> & names,
    const double * pos_ptr,
    const double * vel_ptr) const;

  static pinocchio::SE3 poseMsgToSE3(const geometry_msgs::msg::PoseStamped & msg);
  static Eigen::Matrix<double, 6, 2> computeBaseJacobian(const pinocchio::SE3 & ee_pose);
  Vector6d computeTwistError(const pinocchio::SE3 & current_pose, const pinocchio::SE3 & target_pose) const;

  // Pinocchio model & state
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex left_ee_id_;
  pinocchio::FrameIndex right_ee_id_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> torso_joint_names_{"torso_lift_joint"};
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;

  std::unordered_map<std::string, std::size_t> joint_name_to_upper_idx_;
  std::array<int, kUpperJoints> q_idx_{};
  std::array<int, kUpperJoints> v_idx_{};

  Eigen::VectorXd q_current_;
  Vector15d q_cmd_upper_;
  Vector15d q_nominal_;

  // Stored target messages (can be in odom, map, or base_footprint)
  geometry_msgs::msg::PoseStamped left_target_msg_;
  geometry_msgs::msg::PoseStamped right_target_msg_;
  bool has_left_target_{false};
  bool has_right_target_{false};
  bool has_joint_state_{false};
  bool cmd_initialized_{false};
  bool wbc_enabled_{false};

  // Control parameters & pre-allocated QP matrices
  double dt_{0.01};
  double kp_lin_{3.0};
  double kp_ang_{3.0};
  double k_null_{1.5};
  double base_standoff_x_{0.85};

  Matrix12d W_task_;
  Matrix17d W_reg_;
  Vector17d dq_min_;
  Vector17d dq_max_;

  Matrix12x17d J_wb_;
  Matrix17d H_dense_;
  Eigen::SparseMatrix<double> P_sparse_;
  Eigen::SparseMatrix<double> A_sparse_;
  Vector17d q_vec_;
  Vector17d l_bound_;
  Vector17d u_bound_;

  OsqpEigen::Solver solver_;
  bool solver_initialized_{false};

  // TF2 for world-frame target resolution
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS 2 communication
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr torso_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr telemetry_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tiago_wbc