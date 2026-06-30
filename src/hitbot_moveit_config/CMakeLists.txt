cmake_minimum_required(VERSION 3.5)
project(hitbot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(test_moveit_sequence src/test_moveit_sequence.cpp)
ament_target_dependencies(test_moveit_sequence rclcpp moveit_ros_planning_interface)

install(TARGETS test_moveit_sequence
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
