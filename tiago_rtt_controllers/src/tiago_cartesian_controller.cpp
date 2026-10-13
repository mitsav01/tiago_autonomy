#pragma once

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
#include <array>
#include <cmath>
#include <algorithm>

struct Transform4 {
  std::array<double, 9> R;
  std::array<double, 3> p;

  static constexpr Transform4 Identity() {
    return Transform4{
      .R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
      .p = {0.0, 0.0, 0.0}
    };
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
    Transform4 out{};
    for (size_t r = 0; r < 3; ++r) {
      for (size_t c = 0; c < 3; ++c) {
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
    t.R[0] = c;  t.R[1] = -s;
    t.R[3] = s;  t.R[4] = c;
    return t;
  }
};

class TiagoCartesianController : public RTT::TaskContext {
public:
  static constexpr size_t NUM_ARM_JOINTS = 7;
  static constexpr size_t MAX_MSG_JOINTS = 64; // Bounds for robot-wide joint state

  static constexpr std::array<double, NUM_ARM_JOINTS> Q_MIN = {
    -1.108097, -1.108097, -0.715398, -0.322699, -2.074395, -1.393717, -2.074395
  };

  static constexpr std::array<double, NUM_ARM_JOINTS> Q_MAX = {
     1.500796,  1.500796,  3.856991,  2.286194,  2.074395,  1.393717,  2.074395
  };

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
      torso_index_(-1),
      current_torso_z_(0.20)
  {
    // Register Ports
    this->addPort("joint_states_in", joint_states_in_)
        .doc("Input port reading full-robot joint states from /joint_states");
    this->addPort("ft_sensor_in", ft_sensor_in_)
        .doc("Input port reading wrist force/torque sensor feedback");
    this->addPort("joint_cmd_out", joint_cmd_out_)
        .doc("Output port streaming 1 kHz JointTrajectoryPoint commands to ros2_control");
    this->addPort("target_pose_in", target_pose_in_)
        .doc("Input port receiving Cartesian target pose");
    this->addPort("twist_cmd_in", twist_cmd_in_)
        .doc("Input port receiving Cartesian feedforward twist");
    this->addPort("current_pose_out", current_pose_out_)
        .doc("Output port broadcasting computed Cartesian gripper grasping pose");

    // Register Properties
    this->addProperty("joint_prefix", joint_prefix_)
        .doc("Prefix for arm joints: 'arm_right_' or 'arm_left_'");
    this->addProperty("root_frame", root_frame_)
        .doc("Reference coordinate frame for Cartesian poses");
    this->addProperty("damping_gain", damping_gain_)
        .doc("Cartesian tracking gain / damping parameter");

    // Pre-allocate outgoing trajectory point arrays to fixed sizes
    cmd_out_.positions.resize(NUM_ARM_JOINTS, 0.0);
    cmd_out_.velocities.resize(NUM_ARM_JOINTS, 0.0);
    cmd_out_.accelerations.resize(NUM_ARM_JOINTS, 0.0);
    cmd_out_.effort.resize(NUM_ARM_JOINTS, 0.0);

    // Pre-allocate incoming JointState message buffer to avoid runtime reallocation
    in_joint_state_.name.reserve(MAX_MSG_JOINTS);
    in_joint_state_.position.reserve(MAX_MSG_JOINTS);
    in_joint_state_.velocity.reserve(MAX_MSG_JOINTS);
    in_joint_state_.effort.reserve(MAX_MSG_JOINTS);

    current_pose_out_msg_.header.frame_id = root_frame_;
    current_pose_out_msg_.pose.orientation.w = 1.0;

    current_q_.fill(0.0);
    integrated_q_.fill(0.0);
    joint_indices_.fill(-1);
  }

  bool configureHook() override {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      RTT::log(RTT::Warning) << "[" << this->getName()
                             << "] Failed to lock memory with mlockall()" << RTT::endlog();
    }

    is_right_arm_ = (joint_prefix_.find("left") == std::string::npos);
    current_pose_out_msg_.header.frame_id = root_frame_;

    // Pre-compute expected joint names in configureHook()
    for (size_t i = 1; i <= NUM_ARM_JOINTS; ++i) {
      expected_joint_names_[i - 1] = joint_prefix_ + std::to_string(i) + "_joint";
    }

    has_valid_state_ = false;
    has_target_pose_ = false;
    indices_cached_ = false;
    torso_index_ = -1;
    current_torso_z_ = 0.20;
    joint_indices_.fill(-1);

    RTT::log(RTT::Info) << "[" << this->getName()
                        << "] Configured for " << (is_right_arm_ ? "RIGHT" : "LEFT")
                        << " arm with prefix: '" << joint_prefix_ << "'" << RTT::endlog();
    return true;
  }

  bool startHook() override {
    RTT::log(RTT::Info) << "[" << this->getName()
                        << "] Real-time execution started." << RTT::endlog();
    return true;
  }

  void updateHook() override {
    // 1. Read input state into pre-allocated member buffers (zero runtime heap alloc)
    if (joint_states_in_.readNewest(in_joint_state_) == RTT::NewData) {
      has_valid_state_ = true;

      // Lock-free index mapping check without allocations or log I/O
      if (__builtin_expect(!indices_cached_, 0)) {
        cacheJointIndicesRT();
      }
    }

    if (!has_valid_state_ || !indices_cached_) {
      return;
    }

    ft_sensor_in_.readNewest(in_wrench_);
    twist_cmd_in_.readNewest(in_twist_cmd_);

    if (target_pose_in_.readNewest(in_target_pose_) == RTT::NewData) {
      integrated_q_ = current_q_;
      has_target_pose_ = true;
    }

    // 2. Extract state without dynamic lookups
    const auto& pos_vec = in_joint_state_.position;
    const size_t pos_size = pos_vec.size();

    if (torso_index_ >= 0 && static_cast<size_t>(torso_index_) < pos_size) {
      current_torso_z_ = pos_vec[torso_index_];
    }

    for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      const int idx = joint_indices_[i];
      if (idx >= 0 && static_cast<size_t>(idx) < pos_size) {
        current_q_[i] = pos_vec[idx];
      }
    }

    // 3. Forward Kinematics (all stack arrays)
    std::array<std::array<double, 3>, NUM_ARM_JOINTS> joint_positions{};
    std::array<std::array<double, 3>, NUM_ARM_JOINTS> joint_axes{};
    std::array<double, 3> grasp_pos{};

    forwardKinematicsChain(current_q_, current_torso_z_, grasp_pos, joint_positions, joint_axes);

    // 4. Closed-loop Cartesian velocity & numerical integration
    constexpr double dt = 0.001; // 1 kHz cycle period
    constexpr double deadband_sq = 0.005 * 0.005;

    if (has_target_pose_) {
      const double ex = in_target_pose_.pose.position.x - grasp_pos[0];
      const double ey = in_target_pose_.pose.position.y - grasp_pos[1];
      const double ez = in_target_pose_.pose.position.z - grasp_pos[2];
      const double err_sq = ex * ex + ey * ey + ez * ez;

      if (err_sq > deadband_sq) {
        for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
          const double rx = grasp_pos[0] - joint_positions[i][0];
          const double ry = grasp_pos[1] - joint_positions[i][1];
          const double rz = grasp_pos[2] - joint_positions[i][2];

          // Geometric Jacobian column: J_v = axis x r
          const double jx = joint_axes[i][1] * rz - joint_axes[i][2] * ry;
          const double jy = joint_axes[i][2] * rx - joint_axes[i][0] * rz;
          const double jz = joint_axes[i][0] * ry - joint_axes[i][1] * rx;

          double dq = damping_gain_ * (jx * ex + jy * ey + jz * ez);
          dq = std::clamp(dq, -MAX_DQ[i], MAX_DQ[i]);

          double next_pos = integrated_q_[i] + dq * dt;

          // Joint position limits enforcement
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
      for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
        cmd_out_.positions[i] = current_q_[i];
        cmd_out_.velocities[i] = 0.0;
      }
    }

    // 5. Output write without vector reallocation
    cmd_out_.time_from_start.sec = 0;
    cmd_out_.time_from_start.nanosec = 1000000; // 1 ms lookahead
    joint_cmd_out_.write(cmd_out_);

    current_pose_out_msg_.header.stamp = in_joint_state_.header.stamp;
    current_pose_out_msg_.pose.position.x = grasp_pos[0];
    current_pose_out_msg_.pose.position.y = grasp_pos[1];
    current_pose_out_msg_.pose.position.z = grasp_pos[2];
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
    torso_index_ = -1;
  }

private:
  void cacheJointIndicesRT() noexcept {
    size_t matched = 0;
    const size_t num_names = in_joint_state_.name.size();

    for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      for (size_t j = 0; j < num_names; ++j) {
        // String comparison without heap allocation
        if (in_joint_state_.name[j] == expected_joint_names_[i]) {
          joint_indices_[i] = static_cast<int>(j);
          matched++;
          break;
        }
      }
    }

    for (size_t j = 0; j < num_names; ++j) {
      if (in_joint_state_.name[j] == "torso_lift_joint") {
        torso_index_ = static_cast<int>(j);
        break;
      }
    }

    if (matched == NUM_ARM_JOINTS) {
      indices_cached_ = true;
    }
  }

  void forwardKinematicsChain(const std::array<double, NUM_ARM_JOINTS>& q,
                              double torso_lift_val,
                              std::array<double, 3>& grasp_pos,
                              std::array<std::array<double, 3>, NUM_ARM_JOINTS>& j_pos,
                              std::array<std::array<double, 3>, NUM_ARM_JOINTS>& j_axis) const noexcept
  {
    Transform4 T_torso = Transform4::Identity();
    T_torso.p = {-0.062, 0.0, 0.193 + 0.597 + torso_lift_val};

    const Transform4 T_shoulder = is_right_arm_
      ? Transform4::FromRPY(0.0, 0.0, -1.5707963267948966, 0.02556, -0.19, -0.171)
      : Transform4::FromRPY(3.141592653589793, 0.0, 1.5707963267948966, 0.02556, 0.19, -0.171);

    Transform4 T = T_torso * T_shoulder;

    static const std::array<Transform4, NUM_ARM_JOINTS> T_links_left = {
      Transform4::Identity(),
      Transform4::FromRPY(1.5707963267948966, 0.0, 0.0, 0.125, -0.0195, 0.031),
      Transform4::FromRPY(-1.5707963267948966, 3.141592653589793, -1.5707963267948966, 0.0895, 0.0, -0.0015),
      Transform4::FromRPY(1.5707963267948966, -1.5707963267948966, 0.0, -0.02, -0.027, 0.222),
      Transform4::FromRPY(1.5707963267948966, -1.5707963267948966, -1.5707963267948966, 0.162, -0.02, -0.027),
      Transform4::FromRPY(0.0, -1.5707963267948966, -1.5707963267948966, 0.0, 0.0, -0.150),
      Transform4::FromRPY(1.5707963267948966, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0)
    };

    static const std::array<Transform4, NUM_ARM_JOINTS> T_links_right = {
      Transform4::Identity(),
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
      j_axis[i] = {T.R[2], T.R[5], T.R[8]};
      T = T * Transform4::RotZ(q[i]);
    }

    static constexpr Transform4 T_flange_to_grasp = {
      .R = { 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0 },
      .p = { 0.0, 0.0, 0.23304 }
    };

    T = T * T_flange_to_grasp;
    grasp_pos = {T.p[0], T.p[1], T.p[2]};
  }

  // Real-time Ports
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

  // Pre-allocated Message Buffers
  sensor_msgs::msg::JointState in_joint_state_;
  geometry_msgs::msg::WrenchStamped in_wrench_;
  geometry_msgs::msg::PoseStamped in_target_pose_;
  geometry_msgs::msg::TwistStamped in_twist_cmd_;
  trajectory_msgs::msg::JointTrajectoryPoint cmd_out_;
  geometry_msgs::msg::PoseStamped current_pose_out_msg_;

  // Pre-computed Name Cache (Zero dynamic allocations at runtime)
  std::array<std::string, NUM_ARM_JOINTS> expected_joint_names_;

  // Real-time Fixed Sized State
  bool has_valid_state_;
  bool has_target_pose_;
  bool indices_cached_;
  int torso_index_;
  double current_torso_z_;
  std::array<int, NUM_ARM_JOINTS> joint_indices_;

  std::array<double, NUM_ARM_JOINTS> current_q_;
  std::array<double, NUM_ARM_JOINTS> integrated_q_;
};

ORO_CREATE_COMPONENT(TiagoCartesianController)