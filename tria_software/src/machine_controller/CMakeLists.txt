cmake_minimum_required(VERSION 3.8)
project(machine_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tria_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# PidController
add_executable(pid_controller src/pid_controller.cpp)
target_include_directories(pid_controller
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(pid_controller rclcpp tria_msgs)

# MotorController
add_executable(motor_controller src/motor_controller.cpp)
target_include_directories(motor_controller
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(motor_controller rclcpp tria_msgs std_msgs)

# MachineMerger
add_executable(machine_merger src/machine_merger.cpp)
target_include_directories(machine_merger
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(machine_merger rclcpp tria_msgs std_msgs)

install(TARGETS
  pid_controller
  motor_controller
  machine_merger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()