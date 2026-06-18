cmake_minimum_required(VERSION 3.8)
project(lidar_slam2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  OdomWithCloud.msg
)

add_service_files(
  FILES
  SaveMap.srv
  LoadMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_runtime
)
