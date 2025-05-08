cmake_minimum_required(VERSION 3.8)
project(exo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)  
find_package(ament_cmake_python REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(exo_hw_interface SHARED
  src/exo_hw_interface.cpp
)

target_include_directories(exo_hw_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(exo_hw_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  exo_hw_interface
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
  "hardware_interface"
  "pluginlib"
)
# pluginlib_export_plugin_description_file(hardware_interface exo_control_plugins.xml)
install(DIRECTORY config launch
DESTINATION share/${PROJECT_NAME}/)

install(
  PROGRAMS
  ${PROJECT_NAME}/virtual_joy.py
  ${PROJECT_NAME}/gait_generator.py
  ${PROJECT_NAME}/balance_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
