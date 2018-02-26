cmake_minimum_required(VERSION 2.8.3)
project(tfr_mining)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tfr_msgs
  tfr_utilities
)

find_package(GTest REQUIRED)

catkin_package(
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_executable(test_digging_queue
  src/test_digging_queue.cpp
  src/digging_queue.cpp
  src/digging_set.cpp
)
add_dependencies(test_digging_queue tfr_msgs_gencpp)
target_link_libraries(test_digging_queue
  ${catkin_LIBRARIES}
)

add_executable(digging_action_server
  src/digging_action_server.cpp
  src/digging_queue.cpp
  src/digging_set.cpp
)
add_dependencies(digging_action_server tfr_msgs_gencpp)
target_link_libraries(digging_action_server
  ${catkin_LIBRARIES}
)

add_executable(test_digging_client
  src/test_digging_client.cpp
)
add_dependencies(test_digging_client tfr_msgs_gencpp)
target_link_libraries(test_digging_client
  ${catkin_LIBRARIES}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

