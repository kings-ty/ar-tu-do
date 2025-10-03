cmake_minimum_required(VERSION 3.8)
project(teleoperation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(drive_msgs REQUIRED)

# Find SDL2 for input handling
find_package(SDL2 REQUIRED)
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)

# Include directories
include_directories(include)

# Create keyboard controller executable  
add_executable(keyboard_controller src/keyboard_controller.cpp)
ament_target_dependencies(keyboard_controller
  rclcpp
  std_msgs
  geometry_msgs
  drive_msgs
)
target_link_libraries(keyboard_controller ${SDL2_LIBRARIES})

# Install executables
install(TARGETS
  keyboard_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

# Install python scripts
install(PROGRAMS
  teleoperation/drive_param_to_ackermann_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
