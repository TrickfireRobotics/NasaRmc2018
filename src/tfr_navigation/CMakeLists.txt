cmake_minimum_required(VERSION 2.8.3)
project(tfr_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tfr_msgs
  actionlib
)

find_package(GTest REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)




add_executable(navigation_action_server src/navigation_action_server.cpp)
target_link_libraries(navigation_action_server ${catkin_LIBRARIES})
add_dependencies(navigation_action_server ${catkin_EXPORTED_TARGETS})

add_executable(navigation_action_client src/navigation_action_client.cpp)
target_link_libraries(navigation_action_client ${catkin_LIBRARIES})
add_dependencies(navigation_action_client ${catkin_EXPORTED_TARGETS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_library)
endif()
