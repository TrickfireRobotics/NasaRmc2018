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
  trajectory_msgs
  tfr_msgs
  actionlib
  tf
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
)

find_package(GTest REQUIRED)

# These are all for exporting to dependent packages/projects.
# Uncomment each if the dependent project requires it
catkin_package(
    INCLUDE_DIRS include include/${PROJECT_NAME}
    LIBRARIES status_code tf_manipulator status_publisher arm_manipulator
    CATKIN_DEPENDS 
        roscpp 
        actionlib 
        tf 
        tf2 
        tf2_ros 
        tf2_geometry_msgs
        trajectory_msgs
        tfr_msgs 
        message_runtime 
        sensor_msgs 
        geometry_msgs 
        nav_msgs
)

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_library(status_code src/status_code.cpp)
add_dependencies(status_code ${catkin_EXPORTED_TARGETS})
target_link_libraries(status_code ${catkin_LIBRARIES})

add_library(tf_manipulator ./src/tf_manipulator.cpp)
add_dependencies(tf_manipulator ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_manipulator ${catkin_LIBRARIES})

add_library(arm_manipulator ./src/arm_manipulator.cpp)
add_dependencies(arm_manipulator ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_manipulator ${catkin_LIBRARIES})


add_library(status_publisher ./src/status_publisher.cpp)
add_dependencies(status_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(status_publisher status_code ${catkin_LIBRARIES})


add_executable(point_broadcaster src/point_broadcaster.cpp)
target_link_libraries(point_broadcaster ${catkin_LIBRARIES})
add_dependencies(point_broadcaster ${catkin_EXPORTED_TARGETS})

# This call is sometimes needed and sometimes not and I'm not really clear why
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_system_codes.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test status_code)
endif()

#install shared headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
