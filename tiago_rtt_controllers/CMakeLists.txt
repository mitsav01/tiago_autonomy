cmake_minimum_required(VERSION 3.16)
project(tiago_rtt_controllers)

# 1. Compiler Flags for Real-Time C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 2. Dependencies
find_package(ament_cmake REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(rtt_ros2_node REQUIRED)
find_package(rtt_ros2_topics REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rtt_ros2_std_msgs REQUIRED)
find_package(rtt_ros2_sensor_msgs REQUIRED)
find_package(rtt_ros2_geometry_msgs REQUIRED)
find_package(rtt_ros2_trajectory_msgs REQUIRED)

# 3. Orocos Component Target (matching package name)
orocos_component(tiago_rtt_controllers src/tiago_cartesian_controller.cpp)

target_link_libraries(tiago_rtt_controllers
  std_msgs::std_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  trajectory_msgs::trajectory_msgs__rosidl_typesupport_cpp
)

# 4. Install Configuration and Launch Files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# 5. Orocos and Ament Packaging (strictly at the bottom in this order)
orocos_generate_package()
ament_package()