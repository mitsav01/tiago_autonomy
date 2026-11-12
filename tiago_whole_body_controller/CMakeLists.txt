cmake_minimum_required(VERSION 3.16)
project(tiago_whole_body_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

include(FetchContent)

# 1. Fetch OSQP C library (v0.6.3) if not installed on system
find_package(osqp QUIET)
if(NOT osqp_FOUND)
  set(UNITTESTS OFF CACHE BOOL "" FORCE)
  set(ENABLE_MKL_PARDISO OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    osqp
    GIT_REPOSITORY https://github.com/osqp/osqp.git
    GIT_TAG v0.6.3
  )
  FetchContent_MakeAvailable(osqp)
endif()

# 2. Fetch OsqpEigen C++ wrapper (v0.8.1) if not installed on system
find_package(OsqpEigen QUIET)
if(NOT OsqpEigen_FOUND)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    OsqpEigen
    GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
    GIT_TAG v0.8.1
  )
  FetchContent_MakeAvailable(OsqpEigen)
endif()

include_directories(include)

# 1. Main 17-DOF Dual-Arm QP Whole-Body Controller Node
add_executable(tiago_whole_body_controller_node src/tiago_dual_wbc_node.cpp)
ament_target_dependencies(tiago_whole_body_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(tiago_whole_body_controller_node
  pinocchio::pinocchio
  OsqpEigen::OsqpEigen
  Eigen3::Eigen
)

# 2. Bimanual Figure-Eight Trajectory Generator
add_executable(demo_trajectory_pub src/demo_trajectory_pub.cpp)
ament_target_dependencies(demo_trajectory_pub
  rclcpp
  geometry_msgs
)

# 3. Shelf Pick-and-Retract Manipulation State Machine (Joint-Space Tuck + WBC + Robotiq)
add_executable(shelf_pick_demo src/shelf_pick_demo.cpp)
ament_target_dependencies(shelf_pick_demo
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  trajectory_msgs
  control_msgs
)

# 4. Semantic Object-Coordinate Grasp Planner (/wbc/grasp_object)
add_executable(object_grasp_planner src/object_grasp_planner.cpp)
ament_target_dependencies(object_grasp_planner
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(object_grasp_planner
  Eigen3::Eigen
)

install(TARGETS
  tiago_whole_body_controller_node
  demo_trajectory_pub
  shelf_pick_demo
  object_grasp_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()