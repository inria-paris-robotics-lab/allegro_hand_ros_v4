cmake_minimum_required(VERSION 3.5)
project(allegro_hand_controllers)

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
# find_package(allegro_hand_driver REQUIRED)
# find_package(libpcan REQUIRED)


# Find ament and Python
find_package(ament_cmake REQUIRED)

# Install xacro, world, and mesh files
install(DIRECTORY launch src config DESTINATION share/${PROJECT_NAME})

# Install Python script(s)
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

# Export the package
ament_package()