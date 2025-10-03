cmake_minimum_required(VERSION 3.8)
project(car_control)

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
find_package(drive_msgs REQUIRED)

# Include directories
include_directories(include)

# Declare car controller node executable
add_executable(car_controller src/car_controller.cpp)
ament_target_dependencies(car_controller
  rclcpp
  std_msgs
  drive_msgs
)

# TODO: Migrate other executables later
# add_executable(dms_controller src/dms_controller.cpp)
# add_executable(drive_parameters_multiplexer src/drive_parameters_multiplexer.cpp src/drive_parameters_source.cpp)

# Install executables
install(TARGETS
  car_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  ament_add_gtest(${PROJECT_NAME}-test test/test_car_control.cpp)
  ament_target_dependencies(${PROJECT_NAME}-test rclcpp std_msgs drive_msgs)
endif()

ament_package()
