#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class ShelfWholeBodyPickDemo : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;

  ShelfWholeBodyPickDemo()
  : Node("shelf_wbc_pick_demo"), start_time_(now())
  {
    wbc_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/wbc/enable", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/mobile_base_controller/cmd_vel", 10);
    torso_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/torso_controller/joint_trajectory", 10);
    left_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_left_controller/joint_trajectory", 10);
    right_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_right_controller/joint_trajectory", 10);

    left_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/wbc/left_ee_target", 10);
    right_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/wbc/right_ee_target", 10);

    left_gripper_ = rclcpp_action::create_client<GripperCommand>(
      this, "/left_robotiq_gripper_controller/gripper_cmd");
    right_gripper_ = rclcpp_action::create_client<GripperCommand>(
      this, "/right_robotiq_gripper_controller/gripper_cmd");

    telem_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/wbc/telemetry", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
          err_L_ = msg->data[1];
          err_R_ = msg->data[2];
        }
      });


    // Shelf 2 Level 3 bay opening: x in [1.32, 2.18]
    // Left gripper  -> blue Finish bag   (x = 1.54, y = -1.30, z = 1.13)
    // Right gripper -> yellow Balea bottle (x = 1.70, y = -1.30, z = 1.13)
    waypoints_ = {
      {"DRIVE_TUCKED", {1.560, -0.55, 1.15}, {1.720, -0.55, 1.15}, 0.00, 3.0, 0.8},
      {"PRE_GRASP",    {1.540, -1.04, 1.13}, {1.700, -1.04, 1.13}, 0.00, 2.5, 0.8},
      {"REACH_IN",     {1.540, -1.30, 1.13}, {1.700, -1.30, 1.13}, 0.00, 2.0, 0.8},
      {"CLOSE_GRIP",   {1.540, -1.30, 1.13}, {1.700, -1.30, 1.13}, 0.78, 0.2, 1.5},
      {"LIFT_CLEAR",   {1.540, -1.30, 1.19}, {1.700, -1.30, 1.19}, 0.78, 0.8, 0.8},
      {"RETRACT",      {1.540, -0.80, 1.19}, {1.700, -0.80, 1.19}, 0.78, 2.5, 2.0}
    };

    curr_left_ = waypoints_[0].left_xyz;
    curr_right_ = waypoints_[0].right_xyz;

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ShelfWholeBodyPickDemo::controlStep, this));

    RCLCPP_INFO(get_logger(), "Phase 0: Backing up into aisle and tucking arms in joint space...");
  }

private:
  struct Waypoint
  {
    std::string name;
    std::array<double, 3> left_xyz;
    std::array<double, 3> right_xyz;
    double gripper_pos;
    double move_sec;
    double hold_sec;
  };

  static std::array<double, 3> lerp3(
    const std::array<double, 3> & a, const std::array<double, 3> & b, double alpha)
  {
    const double t = std::clamp(alpha, 0.0, 1.0);
    const double s = t * t * (3.0 - 2.0 * t);
    return {
      a[0] + s * (b[0] - a[0]),
      a[1] + s * (b[1] - a[1]),
      a[2] + s * (b[2] - a[2])
    };
  }

  void sendJointTuck(double duration_sec)
  {
    trajectory_msgs::msg::JointTrajectory torso;
    torso.joint_names = {"torso_lift_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint pt_t;
    pt_t.positions = {0.28};
    pt_t.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    torso.points.push_back(pt_t);
    torso_traj_pub_->publish(torso);

    trajectory_msgs::msg::JointTrajectory left;
    left.joint_names = {
      "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint",
      "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"
    };
    trajectory_msgs::msg::JointTrajectoryPoint pt_l;
    pt_l.positions = {0.35, -0.55, -0.35, 1.65, -1.57, 0.15, 0.0};
    pt_l.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    left.points.push_back(pt_l);
    left_traj_pub_->publish(left);

    trajectory_msgs::msg::JointTrajectory right;
    right.joint_names = {
      "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
      "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"
    };
    trajectory_msgs::msg::JointTrajectoryPoint pt_r;
    pt_r.positions = {-0.35, -0.55, 0.35, 1.65, 1.57, 0.15, 0.0};
    pt_r.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    right.points.push_back(pt_r);
    right_traj_pub_->publish(right);
  }

  geometry_msgs::msg::PoseStamped makeOdomPose(const std::array<double, 3> & xyz)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.pose.position.x = xyz[0];
    msg.pose.position.y = xyz[1];
    msg.pose.position.z = xyz[2];

    const double yaw = -M_PI_2;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = std::sin(yaw * 0.5);
    msg.pose.orientation.w = std::cos(yaw * 0.5);
    return msg;
  }

  void sendGripperGoal(double pos)
  {
    GripperCommand::Goal goal;
    goal.command.position = pos;
    goal.command.max_effort = 40.0;

    if (left_gripper_->action_server_is_ready()) {
      left_gripper_->async_send_goal(goal);
    }
    if (right_gripper_->action_server_is_ready()) {
      right_gripper_->async_send_goal(goal);
    }
  }

  void controlStep()
  {
    const double t_since_start = (now() - start_time_).seconds();

    // PHASE 0 (0.0s -> 3.2s): Joint-space tuck + back up 0.55 m into aisle with WBC disabled
    if (!wbc_handed_off_) {
      std_msgs::msg::Bool en;
      en.data = false;
      wbc_enable_pub_->publish(en);

      if (!tuck_sent_ && t_since_start > 0.2) {
        sendJointTuck(2.6);
        sendGripperGoal(0.0);
        tuck_sent_ = true;
      }

      geometry_msgs::msg::Twist back_twist;
      back_twist.linear.x = (t_since_start < 2.2) ? -0.25 : 0.0;
      cmd_vel_pub_->publish(back_twist);

      if (t_since_start >= 3.2) {
        wbc_handed_off_ = true;
        en.data = true;
        wbc_enable_pub_->publish(en);
        RCLCPP_INFO(get_logger(), "Phase 0 complete. Handing off to 17-DOF QP-WBC...");
      }
      return;
    }

    // PHASE 1..6: 17-DOF Whole-Body Controller Cartesian execution
    if (step_idx_ >= waypoints_.size()) {
      return;
    }

    const auto & wp = waypoints_[step_idx_];

    if (gripper_sent_step_ != static_cast<int>(step_idx_)) {
      sendGripperGoal(wp.gripper_pos);
      gripper_sent_step_ = static_cast<int>(step_idx_);
      seg_start_left_ = curr_left_;
      seg_start_right_ = curr_right_;
      seg_start_time_ = now();
      RCLCPP_INFO(get_logger(), "Phase [%zu/%zu]: %s", step_idx_ + 1, waypoints_.size(), wp.name.c_str());
    }

    const double elapsed = (now() - seg_start_time_).seconds();
    const double alpha = (wp.move_sec > 1e-3) ? (elapsed / wp.move_sec) : 1.0;

    curr_left_ = lerp3(seg_start_left_, wp.left_xyz, alpha);
    curr_right_ = lerp3(seg_start_right_, wp.right_xyz, alpha);

    left_pub_->publish(makeOdomPose(curr_left_));
    right_pub_->publish(makeOdomPose(curr_right_));

    if (alpha >= 1.0 && err_L_ < 0.035 && err_R_ < 0.035) {
      if (!in_hold_) {
        hold_start_ = now();
        in_hold_ = true;
      } else if ((now() - hold_start_).seconds() >= wp.hold_sec) {
        ++step_idx_;
        in_hold_ = false;
      }
    }
  }

  rclcpp::Time start_time_;
  bool tuck_sent_{false};
  bool wbc_handed_off_{false};

  std::vector<Waypoint> waypoints_;
  std::size_t step_idx_{0};
  int gripper_sent_step_{-1};
  bool in_hold_{false};
  rclcpp::Time seg_start_time_;
  rclcpp::Time hold_start_;

  std::array<double, 3> seg_start_left_{};
  std::array<double, 3> seg_start_right_{};
  std::array<double, 3> curr_left_{};
  std::array<double, 3> curr_right_{};

  double err_L_{1.0};
  double err_R_{1.0};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wbc_enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr torso_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_traj_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr telem_sub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr left_gripper_;
  rclcpp_action::Client<GripperCommand>::SharedPtr right_gripper_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShelfWholeBodyPickDemo>());
  rclcpp::shutdown();
  return 0;
}