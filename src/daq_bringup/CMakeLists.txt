cmake_minimum_required(VERSION 3.8)
project(daq_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
add_library(radar_panel SHARED src/radar_panel.cpp include/daq_bringup/radar_panel.hpp)
target_include_directories(radar_panel PRIVATE include)
target_link_libraries(radar_panel Qt5::Widgets)
ament_target_dependencies(radar_panel rclcpp std_msgs rviz_common pluginlib)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)
install(TARGETS radar_panel LIBRARY DESTINATION lib)

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
