cmake_minimum_required(VERSION 2.8.3)
project(tfr_dumping)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    tfr_msgs
    geometry_msgs
    sensor_msgs
    image_transport
    tfr_utilities
)

find_package(GTest REQUIRED)


catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

add_executable(dumping_action_server
    src/dumping_action_server.cpp
)
add_dependencies(dumping_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(dumping_action_server ${catkin_LIBRARIES})


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_library)
endif()
