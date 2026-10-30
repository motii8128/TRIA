cmake_minimum_required(VERSION 3.8)
project(device_communicator)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tria_msgs REQUIRED)
find_package(tria_auto_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(geometry_msgs)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/tria.proto)

# Protocol Buffersのライブラリ
add_library(tria_pb STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(tria_pb PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>   # mcu.pb.h の場所
  ${Protobuf_INCLUDE_DIRS})
target_link_libraries(tria_pb PUBLIC ${Protobuf_LIBRARIES})
set_target_properties(tria_pb PROPERTIES POSITION_INDEPENDENT_CODE ON)


# CentralControlBoardCommunicator
add_executable(ccb_communicator src/ccb_communicator.cpp)

target_include_directories(ccb_communicator
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)

target_link_libraries(ccb_communicator tria_pb)
ament_target_dependencies(ccb_communicator rclcpp tria_msgs)


# GameControllerCommunicator
add_executable(game_controller_communicator src/game_controller_communicator.cpp)

target_include_directories(game_controller_communicator
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(game_controller_communicator rclcpp tria_msgs sensor_msgs geometry_msgs)

# GameControllerCommunicator
add_executable(web_ui_connector src/web_ui_connector.cpp)

target_include_directories(web_ui_connector
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
)
ament_target_dependencies(web_ui_connector rclcpp tria_auto_msgs std_msgs)

install(TARGETS
  ccb_communicator
  game_controller_communicator
  web_ui_connector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
