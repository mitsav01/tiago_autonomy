#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <sys/mman.h>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

struct Transform4 {
  std::array<double, 9> R; // Row-major 3x3 rotation matrix
  std::array<double, 3> p; // 3x1 translation vector

  static Transform4 Identity() {
    Transform4 t;
    t.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    t.p = {0, 0, 0};
    return t;
  }

  static Transform4 FromRPY(double roll, double pitch, double yaw, double x, double y, double z) {
    Transform4 t;
    t.p = {x, y, z};
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);

    t.R[0] = cy * cp;
    t.R[1] = cy * sp * sr - sy * cr;
    t.R[2] = cy * sp * cr + sy * sr;

    t.R[3] = sy * cp;
    t.R[4] = sy * sp * sr + cy * cr;
    t.R[5] = sy * sp * cr - cy * sr;

    t.R[6] = -sp;
    t.R[7] = cp * sr;
    t.R[8] = cp * cr;
    return t;
  }

  Transform4 operator*(const Transform4& rhs) const {
    Transform4 out;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        out.R[r * 3 + c] = R[r * 3 + 0] * rhs.R[0 * 3 + c] +
                           R[r * 3 + 1] * rhs.R[1 * 3 + c] +
                           R[r * 3 + 2] * rhs.R[2 * 3 + c];
      }
      out.p[r] = R[r * 3 + 0] * rhs.p[0] +
                 R[r * 3 + 1] * rhs.p[1] +
                 R[r * 3 + 2] * rhs.p[2] + p[r];
    }
    return out;
  }

  static Transform4 RotZ(double q) {
    Transform4 t = Identity();
    const double c = std::cos(q), s = std::sin(q);
    t.R[0] = c; t.R[1] = -s;
    t.R[2] = 0;
    t.R[3] = s; t.R[4] = c;
    t.R[5] = 0;
    return t;
  }
};

class TiagoCartesianController : public RTT::TaskContext {
public:
  static constexpr size_t NUM_ARM_JOINTS = 7;

  // URDF soft safety limits from tiago_dual.urdf
  static constexpr std::array<double, NUM_ARM_JOINTS> Q_MIN = {
    -1.108097, -1.108097, -0.715398, -0.322699, -2.074395, -1.393717, -2.074395
  };

  static constexpr std::array<double, NUM_ARM_JOINTS> Q_MAX = {
     1.500796,  1.500796,  3.856991,  2.286194,  2.074395,  1.393717,  2.074395
  };

  // Safe joint velocity limits (30% scale of URDF max velocity limits)
  static constexpr std::array<double, NUM_ARM_JOINTS> MAX_DQ = {
    0.585, 0.585, 0.705, 0.705, 0.585, 0.528, 0.528
  };

  explicit TiagoCartesianController(const std::string& name)
    : RTT::TaskContext(name),
      joint_prefix_("arm_left_"),
      root_frame_("base_link"),
      damping_gain_(2.5),
      is_right_arm_(false),
      has_valid_state_(false),
      has_target_pose_(false),
      indices_cached_(false),
      integrated_initialized_(false),
      torso_index_(-1),
      current_torso_z_(0.20)
  {
    this->addPort("joint_states_in", joint_states_in_)
        .doc("Input port reading full-robot joint states from /joint_states");
    this->addPort("ft_sensor_in", ft_sensor_in_)
        .doc("Input port reading wrist force/torque sensor feedback");
    this->addPort("joint_cmd_out", joint_cmd_out_)
        .doc("Output port streaming 1 kHz JointTrajectoryPoint commands to ros2_control");

    this->addPort("target_pose_in", target_pose_in_)
        .doc("Input port receiving Cartesian target pose for gripper grasping frame");
    this->addPort("twist_cmd_in", twist_cmd_in_)
        .doc("Input port receiving Cartesian feedforward twist (TwistStamped)");
    this->addPort("current_pose_out", current_pose_out_)
        .doc("Output port broadcasting computed Cartesian gripper grasping pose");

    this->addProperty("joint_prefix", joint_prefix_)
        .doc("Prefix for arm joints: 'arm_right_' or 'arm_left_'");
    this->addProperty("root_frame", root_frame_)
        .doc("Reference coordinate frame for Cartesian poses (e.g. 'base_link')");
    this->addProperty("damping_gain", damping_gain_)
        .doc("Cartesian tracking gain / damping parameter");

    cmd_out_.positions.resize(NUM_ARM_JOINTS, 0.0);
    cmd_out_.velocities.resize(NUM_ARM_JOINTS, 0.0);
    cmd_out_.accelerations.resize(NUM_ARM_JOINTS, 0.0);
    cmd_out_.effort.resize(NUM_ARM_JOINTS, 0.0);

    current_q_.resize(NUM_ARM_JOINTS, 0.0);
    integrated_q_.resize(NUM_ARM_JOINTS, 0.0);

    current_pose_out_msg_.header.frame_id = root_frame_;
    current_pose_out_msg_.pose.orientation.w = 1.0;

    joint_indices_.fill(-1);
  }

  bool configureHook() override {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      RTT::log(RTT::Warning) << "[" << this->getName()
                             << "] Failed to lock memory with mlockall()" << RTT::endlog();
    }

    is_right_arm_ = (joint_prefix_.find("left") == std::string::npos);
    current_pose_out_msg_.header.frame_id = root_frame_;

    RTT::log(RTT::Info) << "[" << this->getName()
                        << "] Configured for " << (is_right_arm_ ? "RIGHT" : "LEFT")
                        << " arm with prefix: '" << joint_prefix_
                        << "', root frame: '" << root_frame_ << "'" << RTT::endlog();

    has_valid_state_ = false;
    has_target_pose_ = false;
    indices_cached_ = false;
    integrated_initialized_ = false;
    torso_index_ = -1;
    current_torso_z_ = 0.20;
    joint_indices_.fill(-1);

    return true;
  }

  bool startHook() override {
    RTT::log(RTT::Info) << "[" << this->getName()
                        << "] Real-time execution started." << RTT::endlog();
    return true;
  }

  void updateHook() override {
    sensor_msgs::msg::JointState js;
    if (joint_states_in_.readNewest(js) == RTT::NewData) {
      latest_joint_state_ = js;
      has_valid_state_ = true;

      if (!indices_cached_ && !latest_joint_state_.name.empty()) {
        cacheJointIndices(latest_joint_state_);
      }
    }

    if (!has_valid_state_ || !indices_cached_) {
      return;
    }

    geometry_msgs::msg::WrenchStamped ft_data;
    if (ft_sensor_in_.readNewest(ft_data) == RTT::NewData) {
      latest_wrench_ = ft_data;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    if (target_pose_in_.readNewest(target_pose) == RTT::NewData) {
      latest_target_pose_ = target_pose;
      integrated_q_ = current_q_;
      has_target_pose_ = true;
    }

    geometry_msgs::msg::TwistStamped twist_cmd;
    if (twist_cmd_in_.readNewest(twist_cmd) == RTT::NewData) {
      latest_twist_cmd_ = twist_cmd;
    }

    if (torso_index_ >= 0 && static_cast<size_t>(torso_index_) < latest_joint_state_.position.size()) {
      current_torso_z_ = latest_joint_state_.position[torso_index_];
    }

    for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      const int idx = joint_indices_[i];
      if (idx >= 0 && static_cast<size_t>(idx) < latest_joint_state_.position.size()) {
        current_q_[i] = latest_joint_state_.position[idx];
      }
    }

    if (!integrated_initialized_) {
      integrated_q_ = current_q_;
      integrated_initialized_ = true;
    }

    std::array<std::array<double, 3>, NUM_ARM_JOINTS> joint_positions;
    std::array<std::array<double, 3>, NUM_ARM_JOINTS> joint_axes;
    std::array<double, 3> grasp_pos;

    forwardKinematicsChain(current_q_, current_torso_z_, grasp_pos, joint_positions, joint_axes);

    current_pose_out_msg_.pose.position.x = grasp_pos[0];
    current_pose_out_msg_.pose.position.y = grasp_pos[1];
    current_pose_out_msg_.pose.position.z = grasp_pos[2];

    constexpr double dt = 0.001;
    constexpr double deadband = 0.005;

    if (has_target_pose_) {
      const double ex = latest_target_pose_.pose.position.x - grasp_pos[0];
      const double ey = latest_target_pose_.pose.position.y - grasp_pos[1];
      const double ez = latest_target_pose_.pose.position.z - grasp_pos[2];
      const double err_norm = std::sqrt(ex * ex + ey * ey + ez * ez);

      if (err_norm > deadband) {
        for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
          const double rx = grasp_pos[0] - joint_positions[i][0];
          const double ry = grasp_pos[1] - joint_positions[i][1];
          const double rz = grasp_pos[2] - joint_positions[i][2];

          const double jx = joint_axes[i][1] * rz - joint_axes[i][2] * ry;
          const double jy = joint_axes[i][2] * rx - joint_axes[i][0] * rz;
          const double jz = joint_axes[i][0] * ry - joint_axes[i][1] * rx;

          double dq = damping_gain_ * (jx * ex + jy * ey + jz * ez);
          dq = std::clamp(dq, -MAX_DQ[i], MAX_DQ[i]);

          double next_pos = integrated_q_[i] + dq * dt;

          if (next_pos <= Q_MIN[i]) {
            next_pos = Q_MIN[i];
            dq = std::max(0.0, dq);
          } else if (next_pos >= Q_MAX[i]) {
            next_pos = Q_MAX[i];
            dq = std::min(0.0, dq);
          }

          integrated_q_[i] = next_pos;
          cmd_out_.positions[i] = integrated_q_[i];
          cmd_out_.velocities[i] = dq;
        }
      } else {
        for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
          cmd_out_.positions[i] = integrated_q_[i];
          cmd_out_.velocities[i] = 0.0;
        }
      }
    } else {
      cmd_out_.positions = current_q_;
      std::fill(cmd_out_.velocities.begin(), cmd_out_.velocities.end(), 0.0);
    }

    cmd_out_.time_from_start.sec = 0;
    cmd_out_.time_from_start.nanosec = 1000000;
    joint_cmd_out_.write(cmd_out_);

    current_pose_out_msg_.header.stamp = latest_joint_state_.header.stamp;
    current_pose_out_.write(current_pose_out_msg_);
  }

  void stopHook() override {
    RTT::log(RTT::Info) << "[" << this->getName()
                        << "] Real-time execution stopped." << RTT::endlog();
  }

  void cleanupHook() override {
    has_valid_state_ = false;
    has_target_pose_ = false;
    indices_cached_ = false;
    integrated_initialized_ = false;
    torso_index_ = -1;
  }

private:
  void cacheJointIndices(const sensor_msgs::msg::JointState& js) {
    size_t matched = 0;

    for (size_t i = 1; i <= NUM_ARM_JOINTS; ++i) {
      const std::string target_name = joint_prefix_ + std::to_string(i) + "_joint";
      for (size_t j = 0; j < js.name.size(); ++j) {
        if (js.name[j] == target_name) {
          joint_indices_[i - 1] = static_cast<int>(j);
          matched++;
          break;
        }
      }
    }

    for (size_t j = 0; j < js.name.size(); ++j) {
      if (js.name[j] == "torso_lift_joint") {
        torso_index_ = static_cast<int>(j);
        break;
      }
    }

    if (matched == NUM_ARM_JOINTS) {
      indices_cached_ = true;
      RTT::log(RTT::Info) << "[" << this->getName()
                          << "] Successfully mapped joints for prefix '"
                          << joint_prefix_ << "'" << RTT::endlog();
    }
  }

  void forwardKinematicsChain(const std::vector<double>& q,
                              double torso_lift_val,
                              std::array<double, 3>& grasp_pos,
                              std::array<std::array<double, 3>, NUM_ARM_JOINTS>& j_pos,
                              std::array<std::array<double, 3>, NUM_ARM_JOINTS>& j_axis)
  {
    // Chain from base_link to torso_lift_link:
    Transform4 T_torso = Transform4::Identity();
    T_torso.p = {-0.062, 0.0, 0.193 + 0.597 + torso_lift_val};

    // torso_lift_link to arm_1_link:
    Transform4 T_shoulder;
    if (is_right_arm_) {
      // arm_right_1_joint: rpy="0 0 -pi/2", xyz="0.02556 -0.19 -0.171"
      T_shoulder = Transform4::FromRPY(0.0, 0.0, -1.5707963267948966, 0.02556, -0.19, -0.171);
    } else {
      // arm_left_1_joint: rpy="pi 0 pi/2", xyz="0.02556 0.19 -0.171"
      T_shoulder = Transform4::FromRPY(3.141592653589793, 0.0, 1.5707963267948966, 0.02556, 0.19, -0.171);
    }

    Transform4 T = T_torso * T_shoulder;

    // URDF link offsets extracted from tiago_dual.urdf
    const std::array<Transform4, 7> T_links_left = {
      Transform4::Identity(), // arm_left_1_joint
      Transform4::FromRPY(1.5707963267948966, 0.0, 0.0, 0.125, -0.0195, 0.031), // arm_left_2_joint
      Transform4::FromRPY(-1.5707963267948966, 3.141592653589793, -1.5707963267948966, 0.0895, 0.0, -0.0015), // arm_left_3_joint
      Transform4::FromRPY(1.5707963267948966, -1.5707963267948966, 0.0, -0.02, -0.027, 0.222), // arm_left_4_joint
      Transform4::FromRPY(1.5707963267948966, -1.5707963267948966, -1.5707963267948966, 0.162, -0.02, -0.027), // arm_left_5_joint
      Transform4::FromRPY(0.0, -1.5707963267948966, -1.5707963267948966, 0.0, 0.0, -0.150), // arm_left_6_joint
      Transform4::FromRPY(1.5707963267948966, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0) // arm_left_7_joint
    };

    const std::array<Transform4, 7> T_links_right = {
      Transform4::Identity(), // arm_right_1_joint
      Transform4::FromRPY(-1.5707963267948966, 0.0, 0.0, 0.125, -0.0195, -0.031),
      Transform4::FromRPY(-1.5707963267948966, 0.0, 1.5707963267948966, 0.0895, 0.0, 0.0015),
      Transform4::FromRPY(-1.5707963267948966, -1.5707963267948966, 0.0, -0.02, -0.027, -0.222),
      Transform4::FromRPY(1.5707963267948966, -1.5707963267948966, -1.5707963267948966, -0.162, 0.02, 0.027),
      Transform4::FromRPY(0.0, -1.5707963267948966, -1.5707963267948966, 0.0, 0.0, 0.150),
      Transform4::FromRPY(1.5707963267948966, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0)
    };

    const auto& T_links = is_right_arm_ ? T_links_right : T_links_left;

    for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      T = T * T_links[i];

      j_pos[i] = {T.p[0], T.p[1], T.p[2]};
      // URDF axis is "0 0 1" for all 7 arm joints
      j_axis[i] = {T.R[2], T.R[5], T.R[8]};

      T = T * Transform4::RotZ(q[i]);
    }

    Transform4 T_flange_to_grasp;
    T_flange_to_grasp.R = {
       0.0,  0.0,  1.0,
       0.0,  1.0,  0.0,
      -1.0,  0.0,  0.0
    };
    T_flange_to_grasp.p = {0.0, 0.0, 0.23304};

    T = T * T_flange_to_grasp;

    grasp_pos = {T.p[0], T.p[1], T.p[2]};
  }

  // Real-time Data Ports
  RTT::InputPort<sensor_msgs::msg::JointState> joint_states_in_;
  RTT::InputPort<geometry_msgs::msg::WrenchStamped> ft_sensor_in_;
  RTT::InputPort<geometry_msgs::msg::PoseStamped> target_pose_in_;
  RTT::InputPort<geometry_msgs::msg::TwistStamped> twist_cmd_in_;

  RTT::OutputPort<trajectory_msgs::msg::JointTrajectoryPoint> joint_cmd_out_;
  RTT::OutputPort<geometry_msgs::msg::PoseStamped> current_pose_out_;

  // Properties
  std::string joint_prefix_;
  std::string root_frame_;
  double damping_gain_;
  bool is_right_arm_;

  // State Management
  bool has_valid_state_;
  bool has_target_pose_;
  bool indices_cached_;
  bool integrated_initialized_;
  int torso_index_;
  double current_torso_z_;
  std::array<int, NUM_ARM_JOINTS> joint_indices_;

  std::vector<double> current_q_;
  std::vector<double> integrated_q_;

  sensor_msgs::msg::JointState latest_joint_state_;
  geometry_msgs::msg::WrenchStamped latest_wrench_;
  geometry_msgs::msg::PoseStamped latest_target_pose_;
  geometry_msgs::msg::TwistStamped latest_twist_cmd_;

  trajectory_msgs::msg::JointTrajectoryPoint cmd_out_;
  geometry_msgs::msg::PoseStamped current_pose_out_msg_;
};

ORO_CREATE_COMPONENT(TiagoCartesianController)