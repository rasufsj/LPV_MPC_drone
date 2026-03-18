cmake_minimum_required(VERSION 3.8)
project(drone_custom_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(drone_command_filter src/drone_command_filter.cpp)
ament_target_dependencies(drone_command_filter rclcpp geometry_msgs mavros_msgs std_msgs)

install(TARGETS
  drone_command_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()