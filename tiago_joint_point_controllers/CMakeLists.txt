cmake_minimum_required(VERSION 3.8)
project(tiago_joint_point_controllers)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/joint_point_command_controller.cpp)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools trajectory_msgs)

pluginlib_export_plugin_description_file(controller_interface tiago_joint_point_controllers_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(${PROJECT_NAME})
ament_package()