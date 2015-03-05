cmake_minimum_required(VERSION 2.8.3)
project(laser_altimeter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kr_math
  message_filters
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# generate messages
add_message_files(FILES Height.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_definitions("-Wall -Werror -std=c++11")
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
