#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ObjectGraspPlanner : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;

  ObjectGraspPlanner()
  : Node("object_grasp_planner")
  {
    pre_grasp_dist_ = declare_parameter<double>("pre_grasp_distance", 0.34);
    lift_height_ = declare_parameter<double>("lift_height", 0.06);
    retract_dist_ = declare_parameter<double>("retract_distance", 0.40);
    approach_speed_ = declare_parameter<double>("approach_speed_mps", 0.12);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    wbc_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/wbc/enable", 10);
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

    grasp_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/wbc/grasp_object", 10,
      std::bind(&ObjectGraspPlanner::onGraspObjectGoal, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ObjectGraspPlanner::controlStep, this));

    RCLCPP_INFO(
      get_logger(),
      "Object Grasp Planner ready (SLERP + linear path). Publish PoseStamped to /wbc/grasp_object.");
  }

private:
  struct CartesianPhase
  {
    std::string name;
    std::array<double, 3> target_xyz;
    Eigen::Quaterniond target_quat;
    double gripper_pos;
    double move_sec;
    double hold_sec;
  };

  static double smoothstep(double alpha)
  {
    const double t = std::clamp(alpha, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
  }

  static std::array<double, 3> lerp3(
    const std::array<double, 3> & a, const std::array<double, 3> & b, double s)
  {
    return {
      a[0] + s * (b[0] - a[0]),
      a[1] + s * (b[1] - a[1]),
      a[2] + s * (b[2] - a[2])
    };
  }

  static double dist3(const std::array<double, 3> & a, const std::array<double, 3> & b)
  {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  }

  void onGraspObjectGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped obj_odom = *msg;
    if (obj_odom.header.frame_id.empty()) {
      obj_odom.header.frame_id = "odom";
    }
    obj_odom.header.stamp = rclcpp::Time(0);

    geometry_msgs::msg::PoseStamped obj_base;
    try {
      obj_base = tf_buffer_->transform(obj_odom, "base_footprint", tf2::durationFromSec(0.05));
      if (obj_odom.header.frame_id != "odom") {
        obj_odom = tf_buffer_->transform(obj_odom, "odom", tf2::durationFromSec(0.05));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed for grasp goal: %s", ex.what());
      return;
    }

    use_left_arm_ = (obj_base.pose.position.y >= 0.0);

    const std::string active_ee_frame =
      use_left_arm_ ? "gripper_left_grasping_frame" : "gripper_right_grasping_frame";
    std::array<double, 3> start_ee_xyz{};
    Eigen::Quaterniond start_ee_quat = Eigen::Quaterniond::Identity();
    try {
      const auto tf_ee = tf_buffer_->lookupTransform("odom", active_ee_frame, tf2::TimePointZero);
      start_ee_xyz = {
        tf_ee.transform.translation.x,
        tf_ee.transform.translation.y,
        tf_ee.transform.translation.z
      };
      start_ee_quat = Eigen::Quaterniond(
        tf_ee.transform.rotation.w,
        tf_ee.transform.rotation.x,
        tf_ee.transform.rotation.y,
        tf_ee.transform.rotation.z).normalized();
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Could not lookup %s in odom: %s", active_ee_frame.c_str(), ex.what());
      return;
    }

    // Freeze idle arm at its EXACT current pose in base_footprint so it never swings!
    const std::string idle_ee_frame =
      use_left_arm_ ? "gripper_right_grasping_frame" : "gripper_left_grasping_frame";
    try {
      const auto tf_idle = tf_buffer_->lookupTransform("base_footprint", idle_ee_frame, tf2::TimePointZero);
      idle_hold_pose_.header.frame_id = "base_footprint";
      idle_hold_pose_.pose.position.x = tf_idle.transform.translation.x;
      idle_hold_pose_.pose.position.y = tf_idle.transform.translation.y;
      idle_hold_pose_.pose.position.z = tf_idle.transform.translation.z;
      idle_hold_pose_.pose.orientation = tf_idle.transform.rotation;
      has_idle_hold_ = true;
    } catch (const tf2::TransformException &) {
      has_idle_hold_ = false;
    }

    Eigen::Quaterniond q(
      obj_odom.pose.orientation.w,
      obj_odom.pose.orientation.x,
      obj_odom.pose.orientation.y,
      obj_odom.pose.orientation.z);

    const bool is_default_orientation =
      (q.norm() < 1e-3) ||
      (std::abs(q.x()) < 1e-3 && std::abs(q.y()) < 1e-3 && std::abs(q.z()) < 1e-3);

    if (is_default_orientation) {
      q = Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()));
    } else {
      q.normalize();
    }
    grasp_quat_ = q;

    const Eigen::Vector3d approach_dir = q * Eigen::Vector3d::UnitX();
    const std::array<double, 3> obj_xyz = {
      obj_odom.pose.position.x,
      obj_odom.pose.position.y,
      obj_odom.pose.position.z
    };

    const std::array<double, 3> pre_grasp_xyz = {
      obj_xyz[0] - pre_grasp_dist_ * approach_dir.x(),
      obj_xyz[1] - pre_grasp_dist_ * approach_dir.y(),
      obj_xyz[2] - pre_grasp_dist_ * approach_dir.z()
    };

    geometry_msgs::msg::PoseStamped chest_base;
    chest_base.header.stamp = rclcpp::Time(0);
    chest_base.header.frame_id = "base_footprint";
    chest_base.pose.position.x = 0.42;
    chest_base.pose.position.y = use_left_arm_ ? 0.12 : -0.12;
    chest_base.pose.position.z = obj_base.pose.position.z;
    chest_base.pose.orientation.w = 1.0;

    std::array<double, 3> staging_xyz = pre_grasp_xyz;
    try {
      const auto chest_odom = tf_buffer_->transform(chest_base, "odom", tf2::durationFromSec(0.05));
      staging_xyz = {
        chest_odom.pose.position.x,
        chest_odom.pose.position.y,
        chest_odom.pose.position.z
      };
    } catch (const tf2::TransformException &) {
    }

    const std::array<double, 3> lift_xyz = {
      obj_xyz[0],
      obj_xyz[1],
      obj_xyz[2] + lift_height_
    };

    const std::array<double, 3> retract_xyz = {
      lift_xyz[0] - retract_dist_ * approach_dir.x(),
      lift_xyz[1] - retract_dist_ * approach_dir.y(),
      lift_xyz[2] - retract_dist_ * approach_dir.z()
    };

    const double d_stage = dist3(start_ee_xyz, staging_xyz);
    const double d_pre = dist3(staging_xyz, pre_grasp_xyz);

    // Phase 1 (CENTER_STAGE): Translate in a straight line to chest front WITHOUT rotating wrist yet!
    // Phase 2 (PRE_GRASP):    SLERP wrist orientation to face into shelf while aligning with object.
    // Phase 3 (REACH_IN):     Pure linear insertion along approach vector into shelf.
    phases_ = {
      {"CENTER_STAGE", staging_xyz,   start_ee_quat, 0.00, std::max(2.0, d_stage / approach_speed_), 0.3},
      {"PRE_GRASP",    pre_grasp_xyz, grasp_quat_,   0.00, std::max(1.8, d_pre / approach_speed_),   0.5},
      {"REACH_IN",     obj_xyz,       grasp_quat_,   0.00, std::max(1.8, pre_grasp_dist_ / approach_speed_), 0.6},
      {"CLOSE_GRIP",   obj_xyz,       grasp_quat_,   0.78, 0.2, 1.4},
      {"LIFT_CLEAR",   lift_xyz,      grasp_quat_,   0.78, 0.8, 0.6},
      {"RETRACT",      retract_xyz,   grasp_quat_,   0.78, std::max(2.0, retract_dist_ / approach_speed_), 1.5}
    };

    curr_xyz_ = start_ee_xyz;
    curr_quat_ = start_ee_quat;
    phase_idx_ = 0;
    gripper_sent_phase_ = -1;
    in_hold_ = false;
    active_ = true;

    std_msgs::msg::Bool en;
    en.data = true;
    wbc_enable_pub_->publish(en);

    RCLCPP_INFO(
      get_logger(),
      "Planned %s-arm SLERP+linear grasp for object at odom (%.3f, %.3f, %.3f).",
      use_left_arm_ ? "LEFT" : "RIGHT", obj_xyz[0], obj_xyz[1], obj_xyz[2]);
  }

  geometry_msgs::msg::PoseStamped makeOdomPose(
    const std::array<double, 3> & xyz, const Eigen::Quaterniond & quat)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.pose.position.x = xyz[0];
    msg.pose.position.y = xyz[1];
    msg.pose.position.z = xyz[2];
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    return msg;
  }

  void sendActiveGripper(double pos)
  {
    GripperCommand::Goal goal;
    goal.command.position = pos;
    goal.command.max_effort = 40.0;

    auto & client = use_left_arm_ ? left_gripper_ : right_gripper_;
    if (client->action_server_is_ready()) {
      client->async_send_goal(goal);
    }
  }

  void controlStep()
  {
    if (!active_ || phase_idx_ >= phases_.size()) {
      return;
    }

    const auto & ph = phases_[phase_idx_];

    if (gripper_sent_phase_ != static_cast<int>(phase_idx_)) {
      sendActiveGripper(ph.gripper_pos);
      gripper_sent_phase_ = static_cast<int>(phase_idx_);
      seg_start_xyz_ = curr_xyz_;
      seg_start_quat_ = curr_quat_;
      seg_start_time_ = now();
      RCLCPP_INFO(get_logger(), "Executing [%zu/%zu]: %s", phase_idx_ + 1, phases_.size(), ph.name.c_str());
    }

    const double elapsed = (now() - seg_start_time_).seconds();
    const double alpha = (ph.move_sec > 1e-3) ? (elapsed / ph.move_sec) : 1.0;
    const double s = smoothstep(alpha);

    curr_xyz_ = lerp3(seg_start_xyz_, ph.target_xyz, s);
    curr_quat_ = seg_start_quat_.slerp(s, ph.target_quat);

    auto active_msg = makeOdomPose(curr_xyz_, curr_quat_);
    if (has_idle_hold_) {
      idle_hold_pose_.header.stamp = now();
    }

    if (use_left_arm_) {
      left_pub_->publish(active_msg);
      if (has_idle_hold_) {
        right_pub_->publish(idle_hold_pose_);
      }
    } else {
      right_pub_->publish(active_msg);
      if (has_idle_hold_) {
        left_pub_->publish(idle_hold_pose_);
      }
    }

    const double active_err = use_left_arm_ ? err_L_ : err_R_;
    if (alpha >= 1.0 && active_err < 0.035) {
      if (!in_hold_) {
        hold_start_ = now();
        in_hold_ = true;
      } else if ((now() - hold_start_).seconds() >= ph.hold_sec) {
        ++phase_idx_;
        in_hold_ = false;
        if (phase_idx_ >= phases_.size()) {
          active_ = false;
          RCLCPP_INFO(get_logger(), "Grasp sequence complete! Ready for next /wbc/grasp_object goal.");
        }
      }
    }
  }

  double pre_grasp_dist_{0.34};
  double lift_height_{0.06};
  double retract_dist_{0.40};
  double approach_speed_{0.12};

  bool active_{false};
  bool use_left_arm_{false};
  bool has_idle_hold_{false};
  geometry_msgs::msg::PoseStamped idle_hold_pose_;
  Eigen::Quaterniond grasp_quat_{Eigen::Quaterniond::Identity()};

  std::vector<CartesianPhase> phases_;
  std::size_t phase_idx_{0};
  int gripper_sent_phase_{-1};
  bool in_hold_{false};
  rclcpp::Time seg_start_time_;
  rclcpp::Time hold_start_;

  std::array<double, 3> seg_start_xyz_{};
  std::array<double, 3> curr_xyz_{};
  Eigen::Quaterniond seg_start_quat_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond curr_quat_{Eigen::Quaterniond::Identity()};

  double err_L_{1.0};
  double err_R_{1.0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wbc_enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr telem_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_goal_sub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr left_gripper_;
  rclcpp_action::Client<GripperCommand>::SharedPtr right_gripper_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectGraspPlanner>());
  rclcpp::shutdown();
  return 0;
}