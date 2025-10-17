cmake_minimum_required(VERSION 3.5)
project(allegro_hand_description)

# Find ament and Python
find_package(ament_cmake REQUIRED)

# Install xacro, world, and mesh files
install(DIRECTORY description viz config meshes DESTINATION share/${PROJECT_NAME})

# Install Python script(s)
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

# Export the package
ament_package()
