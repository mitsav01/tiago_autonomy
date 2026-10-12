cmake_minimum_required(VERSION 3.8)
project(tiago_field_tools)

find_package(ament_cmake REQUIRED)

# Install Python executables to lib/tiago_field_tools
install(PROGRAMS
  scripts/health_monitor.py
  scripts/test_runner.py
  scripts/metrics_collector.py
  scripts/report_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config assets to share/tiago_field_tools/config
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()