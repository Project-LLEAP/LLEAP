cmake_minimum_required(VERSION 3.8)
project(exo_mujoco)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Set MuJoCo paths
set(MUJOCO_PATH "$ENV{MUJOCO_PY_MUJOCO_PATH}")

include_directories(
  include
  ${MUJOCO_PATH}/include
)

link_directories(
  ${MUJOCO_PATH}/bin
)

add_executable(mujoco_simulator src/mujoco_simulator.cpp)
ament_target_dependencies(mujoco_simulator rclcpp sensor_msgs geometry_msgs)

# Link against the MuJoCo library
target_link_libraries(mujoco_simulator ${MUJOCO_PATH}/lib/libmujoco.so.3.1.5)

install(TARGETS mujoco_simulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch 
DESTINATION share/${PROJECT_NAME}/)

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
