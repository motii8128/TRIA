cmake_minimum_required(VERSION 3.8)
project(tria_auto_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tria_msgs REQUIRED)
find_package(tria_auto_msgs REQUIRED)
find_package(tf2_ros)

# TaskManager
add_executable(task_manager src/task_manager.cpp)

target_include_directories(task_manager
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(task_manager rclcpp tria_msgs geometry_msgs tria_auto_msgs tf2_ros)

# PidPlanner
add_executable(pid_planner src/pid_planner.cpp)

target_include_directories(pid_planner
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(pid_planner rclcpp geometry_msgs tf2_ros)

install(TARGETS
  task_manager
  pid_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
