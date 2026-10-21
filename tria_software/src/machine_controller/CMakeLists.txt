cmake_minimum_required(VERSION 3.8)
project(machine_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tria_msgs REQUIRED)

add_executable(pid_controller src/pid_controller.cpp)
target_include_directories(pid_controller
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(pid_controller rclcpp tria_msgs)

install(TARGETS
  pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()