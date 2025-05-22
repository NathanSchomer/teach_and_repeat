cmake_minimum_required(VERSION 3.8)
project(teach_and_repeat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Find the interface package
find_package(teach_and_repeat_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(teach_server src/teach_server.cpp)
ament_target_dependencies(teach_server
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  nav_msgs
  teach_and_repeat_interfaces
)


# Install targets
install(TARGETS 

  teach_server

  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION lib/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
