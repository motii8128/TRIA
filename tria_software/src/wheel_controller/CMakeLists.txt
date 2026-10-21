cmake_minimum_required(VERSION 3.8)
project(wheel_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tria_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(simple_controller src/simple_controller.cpp)
target_include_directories(simple_controller
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(simple_controller rclcpp tria_msgs geometry_msgs)

install(TARGETS
  simple_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()