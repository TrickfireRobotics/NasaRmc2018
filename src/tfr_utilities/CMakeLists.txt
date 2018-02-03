# If you're having trouble getting your things to build, visit the ROS
# tutorials at <http://wiki.ros.org/ROS/Tutorials> and also read the ROS
# CMake documentation at <http://wiki.ros.org/catkin/CMakeLists.txt>.

cmake_minimum_required(VERSION 2.8.3)
project(tfr_utilities)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  tfr_msgs
  actionlib
  tf
  geometry_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(GTest REQUIRED)

# These are all for exporting to dependent packages/projects.
# Uncomment each if the dependent project requires it
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS tf message_runtime sensor_msgs geometry_msgs nav_msgs
#  DEPENDS system_lib
)

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/placeholder_utility.cpp
)

add_library(tf_manipulator
    ./src/tf_manipulator.cpp
)

add_executable(point_broadcaster src/point_broadcaster.cpp)
target_link_libraries(point_broadcaster ${catkin_LIBRARIES})
add_dependencies(point_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(bin_localizer src/bin_localizer.cpp)
target_link_libraries(bin_localizer tf_manipulator ${catkin_LIBRARIES})
add_dependencies(bin_localizer ${catkin_EXPORTED_TARGETS})

# This call is sometimes needed and sometimes not and I'm not really clear why
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Add gtest based cpp test target and link libraries
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

#install shared headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
