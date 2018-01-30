# If you're having trouble getting your things to build, visit the ROS
# tutorials at <http://wiki.ros.org/ROS/Tutorials> and also read the ROS
# CMake documentation at <http://wiki.ros.org/catkin/CMakeLists.txt>.

cmake_minimum_required(VERSION 2.8.3)
project(tfr_sensor)

add_compile_options(-std=c++11)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tfr_msgs
  cv_bridge
  image_transport
  sensor_msgs
  actionlib
)

find_package(GTest REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tfr_sensor
#  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
#  DEPENDS OpenCV
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_library(light_detector
    src/light_detector.cpp
)

add_executable(light_detection_action_server ./src/light_detection_action_server.cpp)
target_link_libraries(light_detection_action_server light_detector ${catkin_LIBRARIES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_library)
endif()
