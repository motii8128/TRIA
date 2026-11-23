cmake_minimum_required(VERSION 3.8)
project(machine_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tria_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# MotorController
add_executable(cascade_position_controller src/cascade_position_controller.cpp)
target_include_directories(cascade_position_controller
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(cascade_position_controller rclcpp tria_msgs std_msgs)

# MachineMerger
add_executable(machine_merger src/machine_merger.cpp)
target_include_directories(machine_merger
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(machine_merger rclcpp tria_msgs std_msgs)

install(TARGETS
  motor_controller
  machine_merger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()