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

add_executable(single_tmini_driver src/single_tmini_driver.cpp)

target_include_directories(single_tmini_driver
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
target_link_libraries(single_tmini_driver ydlidar_tmini)
ament_target_dependencies(single_tmini_driver rclcpp sensor_msgs)

install(TARGETS
  single_tmini_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
