cmake_minimum_required(VERSION 3.8)
project(tiago_autonomy_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(iai_refills_lab REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Install configuration, asset, and launch directories
install(DIRECTORY launch rviz2 urdf config
  DESTINATION share/${PROJECT_NAME}/
)

# Install executable Python scripts
install(PROGRAMS
  scripts/pick_and_place_demo.py
  scripts/retail_pick_and_place.py
  scripts/base_trajectory_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()