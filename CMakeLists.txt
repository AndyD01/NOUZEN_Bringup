cmake_minimum_required(VERSION 3.8)
project(nouzen_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create dependencies list
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  controller_manager
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  geometry_msgs
  std_msgs
  sensor_msgs
)

# Hardware interface library
add_library(nouzen_hardware_interface SHARED
  hardware/src/nouzen_hardware_interface.cpp
)

target_compile_features(nouzen_hardware_interface PUBLIC cxx_std_17)

target_include_directories(nouzen_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/nouzen_bringup>
)

ament_target_dependencies(nouzen_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

# Install libraries
install(
  TARGETS 
    nouzen_hardware_interface
  EXPORT nouzen_bringup_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY hardware/include/
  DESTINATION include/nouzen_bringup
)

# Install other directories
install(
  DIRECTORY 
    bringup 
    description
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration files
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export targets for downstream packages  
ament_export_targets(nouzen_bringup_targets HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install scripts
install(PROGRAMS
  scripts/dock_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()