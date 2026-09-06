cmake_minimum_required(VERSION 3.8)
project(tiago_navigation)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(sensor_msg REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
scripts/dock_pose_publisher.py
DESTINATION lib/${PROJECT_NAME}
)
ament_package()
