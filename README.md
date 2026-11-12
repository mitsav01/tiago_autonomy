# TIAGo Autonomous Mobile Manipulation

A ROS 2 robotics portfolio project focused on autonomous navigation, motion planning, sensor fusion, real-time control experiments, and whole-body mobile manipulation using the PAL TIAGo dual-arm robot in Gazebo Harmonic.

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![Navigation](https://img.shields.io/badge/Navigation-Nav2-green)
![Motion Planning](https://img.shields.io/badge/Motion%20Planning-MoveIt%202-purple)
![Control](https://img.shields.io/badge/Control-Whole--Body%20Control-red)
![Status](https://img.shields.io/badge/Project-Active%20Development-yellow)

---

## Demonstration

<p align="center">
  <video
    src="media/tiago_autonomy_demo.mp4"
    autoplay
    loop
    muted
    playsinline
    controls
    width="850">
  </video>
</p>

<p align="center">
  <em>TIAGo autonomous navigation and mobile manipulation in Gazebo Harmonic.</em>
</p>

The demonstration video will showcase the implemented navigation, planning, and manipulation capabilities of the simulated robot.

---

## 1. Project Overview

This project develops an integrated robotics software stack for the PAL TIAGo dual-arm mobile manipulator.

The objective is to bring together mobile robot navigation, manipulation planning, sensor fusion, and coordinated whole-body control within a ROS 2 architecture.

The project combines:

- **Mobile robot navigation:** Nav2, SLAM, AMCL, and differential-drive control.
- **State estimation:** Wheel odometry, IMU integration, and EKF-based sensor fusion.
- **Motion planning:** MoveIt 2, OMPL, and robot kinematic configuration.
- **Whole-body control:** Custom 17-DOF C++ quadratic-programming controller using Pinocchio and OSQP-Eigen.
- **Manipulation:** Object-centric grasp planning and dual-arm trajectory generation.
- **Robot control:** Custom `ros2_control` controllers and OROCOS RTT control experiments.
- **Engineering tools:** Health monitoring, jitter benchmarking, metrics collection, and automated tests.

The software is developed and evaluated in **Gazebo Harmonic simulation**. The project is intended as an ongoing portfolio and robotics systems integration effort.

## 2. System Architecture

The architecture is organized into four main subsystems.

### 2.1 Simulation and robot integration

- Gazebo Harmonic simulation environment.
- TIAGo dual-arm robot model.
- Custom simulation environment using `dm_room.sdf` and `dm_room.urdf`.
- ROS 2 integration through `ros_gz_sim` and `ros_gz_bridge`.
- Robot visualization and coordinate-frame management using RViz 2 and TF2.

### 2.2 Autonomous navigation

- Differential-drive mobile base.
- Wheel odometry and IMU-based state estimation.
- EKF sensor fusion using `robot_localization`.
- SLAM Toolbox and AMCL localization.
- Global and local path planning using Nav2.

### 2.3 Motion planning and manipulation

- MoveIt 2 configuration for the TIAGo dual-arm robot.
- OMPL-based motion planning.
- Custom manipulation and pick-and-place demonstration scripts.
- Object-centric grasp planning.
- Robotiq 2F gripper action execution.

### 2.4 Whole-body and real-time control

- Custom C++ 17-DOF quadratic-programming whole-body controller.
- Pinocchio-based forward kinematics and Jacobian computation.
- OSQP-Eigen optimization.
- TCP-level end-effector tracking.
- Custom joint-point and Cartesian controllers.
- OROCOS RTT-based Cartesian control experiments.

## 3. Whole-Body Control

One of the main components of this project is a custom **17-DOF velocity-level whole-body controller**.

The controller coordinates the mobile base, torso, and both seven-DOF arms within a unified optimization framework.

### Controlled degrees of freedom

| Component | Controlled DOFs |
|---|---:|
| Differential-drive base (`vx`, `wz`) | 2 |
| Torso lift | 1 |
| Left arm | 7 |
| Right arm | 7 |
| **Total** | **17** |

Head and gripper actuation are handled separately and are not included in the 17 controlled velocity variables.

### Controller architecture

The whole-body controller uses:

- **Pinocchio:** Rigid-body forward kinematics and end-effector Jacobians.
- **Eigen3:** Matrix operations and quaternion interpolation.
- **OSQP-Eigen:** Sparse quadratic-programming optimization.
- **ROS 2:** Command interfaces, state feedback, and trajectory communication.

The optimization combines end-effector tracking with regularization and velocity constraints.

The generalized velocity vector is:

\[
\dot q =
[v_x,\omega_z,\dot q_{\mathrm{torso}},
\dot q_{\mathrm{left}}^T,
\dot q_{\mathrm{right}}^T]^T
\in \mathbb{R}^{17}
\]

The controller solves a constrained optimization problem:

\[
\min_{\dot q}
\frac{1}{2}\|J(q)\dot q-v_{\mathrm{des}}\|^2_{W}
+
\frac{1}{2}\|\dot q-\dot q_{\mathrm{null}}\|^2_{W_{\mathrm{reg}}}
\]

subject to velocity limits and base-motion constraints.

### ObjectGraspPlanner

The custom `ObjectGraspPlanner` provides an object-centric interface for manipulation.

It:

1. Receives an object pose goal through `/wbc/grasp_object`.
2. Determines the appropriate arm for the target.
3. Retrieves the current end-effector pose using TF2.
4. Generates Cartesian motion targets.
5. Coordinates the grasp sequence with gripper action execution.
6. Publishes end-effector targets for the whole-body controller.

The current grasp sequence consists of:

`CENTER_STAGE → PRE_GRASP → REACH_IN → CLOSE_GRIP → LIFT_CLEAR → RETRACT`

The implementation uses Cartesian trajectory profiling and quaternion SLERP for smooth end-effector orientation transitions.

### TCP-level tracking

The controller tracks the gripper grasping frames:

- `gripper_left_grasping_frame`
- `gripper_right_grasping_frame`

This places the controlled end-effector reference at the gripper grasping point rather than the wrist flange.

## 4. Navigation and State Estimation

The navigation stack combines odometry, IMU measurements, localization, and Nav2 planning.

### State estimation

`robot_localization` is used for EKF-based fusion of available odometry and IMU measurements.

TF2 maintains the robot's coordinate-frame relationships, supporting navigation and manipulation components.

### Navigation pipeline

The navigation subsystem integrates:

- Differential-drive base control.
- Wheel odometry.
- IMU measurements.
- EKF state estimation.
- SLAM Toolbox.
- AMCL localization.
- Nav2 global and local planning.

The navigation configuration is maintained in `tiago_navigation`.

## 5. MoveIt 2 Integration

The `tiago_moveit_config` package contains the MoveIt 2 configuration for the TIAGo dual-arm robot.

It includes:

- Dual-arm URDF/Xacro and SRDF configuration.
- Kinematic solver configuration.
- OMPL planning configuration.
- Joint limits.
- Initial joint positions.
- Pilz Cartesian limits.
- MoveIt controller configuration.
- Launch files for MoveIt and RViz integration.

This provides a motion-planning framework alongside the custom whole-body controller.

## 6. Custom Controllers and Control Experiments

The project includes separate packages for different control approaches.

| Package | Purpose |
|---|---|
| `tiago_joint_point_controllers` | Custom joint-point command controller |
| `tiago_whole_body_controller` | 17-DOF C++ QP whole-body control |
| `tiago_rtt_controllers` | OROCOS RTT-based Cartesian control experiments |

The RTT controller work explores a different control execution architecture and provides a basis for comparing ROS 2-based and OROCOS RTT-based control approaches.

A benchmark visualization is available at `media/rtt_vs_ros2_benchmark.png`.

## 7. Robotics Field Tools

The `tiago_field_tools` package provides supporting tools for testing and evaluating the robotics software stack.

| Tool | Purpose |
|---|---|
| `health_monitor.py` | Robot and ROS graph health monitoring |
| `jitter_benchmark.py` | Timing and execution jitter measurements |
| `metrics_collector.py` | Collection of system performance metrics |
| `report_generator.py` | Generation of test and performance reports |
| `test_runner.py` | Execution of configured acceptance tests |

The package also contains automated tests for health monitoring, metrics, and schema validation.

## 8. Repository Structure

```text
tiago_autonomy/
├── LICENSE
├── README.md
├── media/
│   ├── forward_docking.gif
│   ├── forward_docking.webm
│   └── rtt_vs_ros2_benchmark.png
│
├── tiago_autonomy_bringup/
│   ├── config/
│   ├── launch/
│   ├── scripts/
│   │   ├── base_trajectory_action_server.py
│   │   ├── pick_and_place_demo.py
│   │   └── retail_pick_and_place.py
│   └── urdf/
│       ├── dm_room.sdf
│       ├── dm_room.urdf
│       ├── tiago_autonomy.urdf
│       └── urdf_to_map.py
│
├── tiago_field_tools/
│   ├── config/
│   ├── scripts/
│   └── tests/
│
├── tiago_joint_point_controllers/
│   ├── include/
│   ├── src/
│   └── tiago_joint_point_controllers_plugins.xml
│
├── tiago_moveit_config/
│   ├── config/
│   └── launch/
│
├── tiago_navigation/
│   ├── config/
│   ├── launch/
│   ├── maps/
│   └── scripts/
│
├── tiago_rtt_controllers/
│   ├── config/
│   ├── include/
│   ├── launch/
│   └── src/
│
└── tiago_whole_body_controller/
    ├── config/
    ├── include/
    ├── launch/
    └── src/
        ├── demo_trajectory_pub.cpp
        ├── object_grasp_planner.cpp
        ├── shelf_pick_demo.cpp
        └── tiago_dual_wbc_node.cpp
```

## 9. Dependencies and Technology Stack

| Category | Technologies |
|---|---|
| Middleware | ROS 2 Jazzy |
| Simulation | Gazebo Harmonic |
| Programming | C++17, Python |
| Navigation | Nav2, SLAM Toolbox, AMCL |
| State estimation | `robot_localization`, EKF |
| Motion planning | MoveIt 2, OMPL |
| Robot control | `ros2_control` |
| Whole-body control | Pinocchio, OSQP, OSQP-Eigen |
| Mathematical libraries | Eigen3 |
| Visualization | RViz 2, TF2 |
| Real-time control experiments | OROCOS RTT |
| Testing and benchmarking | Custom Python tools and automated tests |

## 10. Build and Installation

### Prerequisites

- Ubuntu with ROS 2 Jazzy installed.
- Gazebo Harmonic.
- MoveIt 2.
- Nav2.
- `ros2_control` and `ros2_controllers`.
- Pinocchio.
- Eigen3.
- OSQP and OSQP-Eigen, where required by the whole-body controller.

Install the relevant ROS 2 dependencies using `rosdep`:

```bash
cd ~/portfolio

rosdep install \
  --from-paths src/tiago_autonomy \
  --ignore-src \
  -r -y
```

Build the workspace:

```bash
cd ~/portfolio

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Source the workspace:

```bash
source install/setup.bash
```

## 11. Running the Project

The following commands are examples of the project launch entry points. Launch dependencies and required environment configuration should be satisfied before starting each subsystem.

### Robot simulation and bringup

```bash
ros2 launch tiago_autonomy_bringup \
  tiago_autonomy_bringup.launch.py
```

### MoveIt 2 integration

```bash
ros2 launch tiago_autonomy_bringup \
  tiago_autonomy_moveit_bringup.launch.py
```

### Navigation

```bash
ros2 launch tiago_navigation \
  tiago_navigation.launch.py
```

### Whole-body controller

```bash
ros2 launch tiago_whole_body_controller \
  tiago_wbc.launch.py
```

### Object grasp planner

```bash
ros2 run tiago_whole_body_controller \
  object_grasp_planner
```

### Example object pose command

```bash
ros2 topic pub --once \
  /wbc/grasp_object \
  geometry_msgs/msg/PoseStamped \
  "{
    header: {frame_id: 'odom'},
    pose: {
      position: {x: 1.70, y: -1.30, z: 1.13}
    }
  }"
```

The example pose must be adapted to the object location and coordinate frame in the running simulation.

## 12. Debugging and Monitoring

### Whole-body control

```bash
ros2 topic echo /wbc/telemetry

ros2 param get \
  /tiago_dual_wbc_controller \
  left_ee_frame

ros2 run tf2_ros tf2_echo \
  odom gripper_right_grasping_frame
```

Enable the whole-body controller:

```bash
ros2 topic pub --once \
  /wbc/enable \
  std_msgs/msg/Bool \
  "{data: true}"
```

### Navigation and TF2

```bash
ros2 run tf2_ros tf2_echo \
  map base_footprint

ros2 topic echo /scan

ros2 topic echo /odom
```

## 13. Engineering Challenges

The project involves several practical robotics software and control challenges.

**1. Simulation contact stability**

High-complexity collision geometry can introduce unstable contact behaviour in simulation. Simplifying collision geometries helps improve contact stability and mobile-base motion.

**2. End-effector kinematics**

Using the wrist flange as the end-effector reference does not directly represent the gripper's grasping point. TCP-level Jacobian tracking provides a more suitable reference for manipulation.

**3. Whole-body coordination**

Coordinating a non-holonomic mobile base, torso, and two seven-DOF arms requires a unified representation of task-space motion and joint-space constraints.

**4. Manipulation near shelving**

Shelf manipulation requires careful trajectory generation, staging poses, and collision-aware arm motion to avoid unwanted contact with shelf geometry.

**5. Control execution and timing**

The project explores controller execution behaviour, timing jitter, and performance measurement through ROS 2 and OROCOS RTT-based implementations.

## 14. Development Roadmap

- [x] Gazebo Harmonic TIAGo simulation integration
- [x] Custom simulation environment
- [x] ROS 2 control integration
- [x] Differential-drive base control
- [x] EKF state estimation
- [x] SLAM and AMCL localization
- [x] Nav2 integration
- [x] MoveIt 2 configuration
- [x] Custom joint-point controller
- [x] C++ whole-body QP controller
- [x] TCP-level end-effector tracking
- [x] Object-centric grasp planner implementation
- [x] RTT control experiments
- [x] Field monitoring and benchmarking tools
- [ ] Further shelf-approach and collision tuning
- [ ] Complete autonomous pick-and-place mission
- [ ] Integrate perception-driven object pose generation
- [ ] High-level autonomous mission execution

## 15. Project Scope

This is an actively developed simulation-based robotics project. The navigation, motion-planning, and manipulation components are developed for the simulated TIAGo platform in Gazebo Harmonic. The repository also contains control experiments and engineering tools for evaluating system behaviour.

The presence of configuration files or experimental components does not imply that every subsystem has been fully validated as an autonomous end-to-end workflow.

## Author

**Mitesh Savsaviya**

Robotics | Autonomous Systems | Control Engineering

GitHub: [@mitsav01](https://github.com/mitsav01)

---

*The long-term objective is to develop an integrated mobile manipulation system capable of navigating an environment, identifying objects, executing coordinated manipulation tasks, and completing autonomous missions.*