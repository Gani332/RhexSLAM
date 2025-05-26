cmake_minimum_required(VERSION 3.8)
project(rhex_description)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install folders (only matching necessary file types)
install(
  DIRECTORY
    urdf
    meshes
    config
    launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.xacro"
    PATTERN "*.urdf.xacro"
    PATTERN "*.urdf"
    PATTERN "*.yaml"
    PATTERN "*.stl"
    PATTERN "*.dae"
    PATTERN "*.py"
    PATTERN "*.gazebo"
    PATTERN "*.trans"
)

ament_package()
