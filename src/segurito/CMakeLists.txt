cmake_minimum_required(VERSION 3.8)
project(segurito)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(include)

set(NODE_SOURCES
  src/motion_sensor_node.cpp
  src/camera_vision_node.cpp
  src/battery_monitor_node.cpp
  src/encoder_node.cpp
  src/odometry_node.cpp
  src/slam_node.cpp
  src/map_server_node.cpp
  src/navigation_node.cpp
  src/motor_controller_node.cpp
  src/alert_manager_node.cpp
)

foreach(source_file ${NODE_SOURCES})
  get_filename_component(node_name ${source_file} NAME_WE)
  add_executable(${node_name} ${source_file})
  ament_target_dependencies(${node_name} rclcpp std_msgs sensor_msgs)
  target_include_directories(${node_name} PRIVATE include)
  install(TARGETS ${node_name} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()