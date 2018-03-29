cmake_minimum_required(VERSION 2.8.3)
project(tfr_executive)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tfr_utilities
    tfr_msgs
    trajectory_msgs
    geometry_msgs
    actionlib
)

find_package(GTest REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_executable(clock_service src/clock_service.cpp)
add_dependencies(clock_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(clock_service ${catkin_LIBRARIES})

add_executable(autonomous_action_server src/autonomous_action_server.cpp)
add_dependencies(autonomous_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(autonomous_action_server ${catkin_LIBRARIES})

add_executable(teleop_action_server src/teleop_action_server.cpp)
add_dependencies(teleop_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_action_server ${catkin_LIBRARIES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
