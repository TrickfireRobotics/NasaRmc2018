cmake_minimum_required(VERSION 2.8.3)
project(tfr_executive)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
)

find_package(GTest REQUIRED)

catkin_package(
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
