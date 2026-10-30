cmake_minimum_required(VERSION 3.8)
project(ydlidar_tmini_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


add_library(ydlidar_tmini src/lidar.cpp)
target_include_directories(ydlidar_tmini 
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)

add_executable(tmini_driver src/tmini_driver.cpp)

target_include_directories(tmini_driver
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
target_link_libraries(tmini_driver ydlidar_tmini)
ament_target_dependencies(tmini_driver rclcpp sensor_msgs)

add_executable(tmini_merge_node src/scan_merger_node.cpp)
target_include_directories(tmini_merge_node
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(tmini_merge_node rclcpp sensor_msgs)

install(TARGETS
  tmini_driver
  tmini_merge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
