cmake_minimum_required(VERSION 3.5)
project(rhex_control)

# Set Python as language
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    scripts/rhex_gait.py
    scripts/cmd_vel_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
