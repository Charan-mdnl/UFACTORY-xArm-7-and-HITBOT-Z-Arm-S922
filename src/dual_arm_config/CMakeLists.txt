cmake_minimum_required(VERSION 3.8)
project(dual_arm_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
