cmake_minimum_required(VERSION 2.8.3)
project(gps_odom)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    kr_math
    message_filters
    nav_msgs
    roscpp
    roslib
    sensor_msgs
    std_msgs
    ublox_msgs
    pressure_altimeter
    rviz_helper
    visualization_msgs
    )
find_package(GeographicLib 1.35 REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
    include
    )

add_definitions(${GeographicLib_DEFINITIONS})

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/node.cpp
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GeographicLib_LIBRARIES}
    )
