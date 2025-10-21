cmake_minimum_required(VERSION 3.10)
project(allegro_hand_interface)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(allegro_hand_driver REQUIRED)


# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# Source files
set(SOURCES
  src/allegro_hand_hardware_interface.cpp 
)

# Library for the hardware interface
add_library(${PROJECT_NAME} SHARED ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
  ament_index_cpp
  class_loader
  allegro_hand_driver
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface allegro_hand_interface.xml)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install shared lib
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES allegro_hand_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  allegro_hand_driver
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
