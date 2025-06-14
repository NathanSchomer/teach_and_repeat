cmake_minimum_required(VERSION 3.8)
project(teach_and_repeat)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(teach_and_repeat_interfaces REQUIRED)
find_package(dbow2 REQUIRED)
find_package(tf2 REQUIRED)


# Include directories
include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(teach_server
  src/teach_server.cpp
  src/frame.cpp
)

add_executable(repeat_server
  src/repeat_server.cpp
  src/frame.cpp
)

add_executable(create_vocab_server
  src/create_vocab_server.cpp
)

# Link dependencies
ament_target_dependencies(teach_server
  ament_index_cpp
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  nav_msgs
  message_filters
  cv_bridge
  teach_and_repeat_interfaces
  dbow2
  tf2
)

ament_target_dependencies(repeat_server
  ament_index_cpp
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  nav_msgs
  message_filters
  cv_bridge
  teach_and_repeat_interfaces
  dbow2
  tf2
)

# Link dependencies
ament_target_dependencies(create_vocab_server
  ament_index_cpp
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  nav_msgs
  message_filters
  cv_bridge
  teach_and_repeat_interfaces
  dbow2
  tf2
)

target_link_libraries(teach_server ${OpenCV_LIBS})
target_include_directories(teach_server PRIVATE
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(repeat_server ${OpenCV_LIBS})
target_include_directories(repeat_server PRIVATE
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(create_vocab_server ${OpenCV_LIBS})
target_include_directories(create_vocab_server PRIVATE
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


# Install targets
install(TARGETS teach_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS repeat_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS create_vocab_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY vocab/
  DESTINATION share/${PROJECT_NAME}/vocab
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()