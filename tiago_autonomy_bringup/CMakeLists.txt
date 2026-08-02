cmake_minimum_required(VERSION 3.8)
project(tiago_autonomy_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(iai_apartment REQUIRED)

install(DIRECTORY launch rviz2 urdf config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
