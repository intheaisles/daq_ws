cmake_minimum_required(VERSION 3.8)
project(daq_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(image_rate_throttle src/image_rate_throttle.cpp)
ament_target_dependencies(image_rate_throttle rclcpp sensor_msgs)

install(
  TARGETS image_rate_throttle
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/topic_rate_monitor.py
    scripts/topic_ready_check.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
