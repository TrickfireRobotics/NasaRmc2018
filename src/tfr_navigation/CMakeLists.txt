cmake_minimum_required(VERSION 2.8.3)
project(tfr_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tfr_msgs
  tfr_utilities
  geometry_msgs
  nav_msgs
  actionlib
)

find_package(GTest REQUIRED)

include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

catkin_package(
)


add_executable(navigation_action_server 
    src/navigation_action_server.cpp
)
add_dependencies(navigation_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_action_server ${catkin_LIBRARIES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

