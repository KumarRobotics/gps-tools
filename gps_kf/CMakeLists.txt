cmake_minimum_required(VERSION 2.8.3)
project(gps_kf)

find_package(catkin REQUIRED COMPONENTS
    eigen_conversions
    geometry_msgs
    kr_math
    message_generation
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    rviz_helper
    )

# generate messages
add_message_files(FILES Vector3WithCovarianceStamped.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    INCLUDE_DIRS include
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )

add_definitions(-Wall -Werror -std=c++11 -DNDEBUG -O3)

add_executable(${PROJECT_NAME}
    src/node.cpp
    src/main.cpp
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
