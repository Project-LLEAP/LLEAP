cmake_minimum_required(VERSION 3.8)
project(exo_state_estimator)

find_package(ament_cmake REQUIRED)

# Nothing to build – just install Python files ------------------------------
ament_python_install_package(${PROJECT_NAME})

# Install the script with executable bit so ros2 can launch it
install(
  PROGRAMS
    exo_state_estimator/state_estimator_ukf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()