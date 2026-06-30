cmake_minimum_required(VERSION 3.8)
project(hitbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ---------- HITBOT TCP driver (standalone, no ROS deps) ----------
add_library(hitbot_tcp_driver SHARED
  src/hitbot_tcp_driver.cpp
)
target_include_directories(hitbot_tcp_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------- ros2_control hardware interface plugin ----------
add_library(hitbot_hardware_interface SHARED
  src/hitbot_hardware_interface.cpp
)
target_include_directories(hitbot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hitbot_hardware_interface hitbot_tcp_driver)
ament_target_dependencies(hitbot_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface hitbot_hardware_plugin.xml)

# ---------- Install ----------
install(
  TARGETS hitbot_tcp_driver hitbot_hardware_interface
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(hitbot_hardware_interface hitbot_tcp_driver)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

ament_package()
