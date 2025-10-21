cmake_minimum_required(VERSION 3.8)
project(allegro_hand_driver)

if(CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(allegro_hand_driver SHARED
  src/AllegroHandDrv.cpp
  src/socket_can.cpp
)

# this line to exports the library
ament_export_targets(allegro_hand_driver HAS_LIBRARY_TARGET)

# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/allegro_hand_driver
  DESTINATION include
)

ament_target_dependencies(allegro_hand_driver
  rclcpp
  std_msgs
)
# --- INSTALLATION ---
install(
  TARGETS allegro_hand_driver
  EXPORT allegro_hand_driver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# --- TESTS ---
if(BUILD_TESTING)
  add_executable(
    low_level_test
    src/test.cpp
  )
  target_link_libraries(
    low_level_test
    allegro_hand_driver
  )
  install(
    TARGETS low_level_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# --- FIN ---
ament_package()