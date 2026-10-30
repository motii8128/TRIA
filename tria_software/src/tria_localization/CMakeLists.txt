cmake_minimum_required(VERSION 3.8)
project(tria_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros)

# LiDAR Only Odometer
add_executable(lidar_only_odometer src/lidar_only_odometer.cpp)

target_include_directories(lidar_only_odometer
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(lidar_only_odometer rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)

install(TARGETS
  lidar_only_odometer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
