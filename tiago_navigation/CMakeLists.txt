cmake_minimum_required(VERSION 3.8)
project(tiago_navigation)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(sensor_msg REQUIRED)

add_executable(tiago_navigation src/tiago_navigation.cpp)

ament_target_dependencies(
  tiago_navigation
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
