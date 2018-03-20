# If you're having trouble getting your things to build, visit the ROS
# tutorials at <http://wiki.ros.org/ROS/Tutorials> and also read the ROS
# CMake documentation at <http://wiki.ros.org/catkin/CMakeLists.txt>.

cmake_minimum_required(VERSION 2.8.3)
project(tfr_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tfr_msgs
  tfr_utilities
  actionlib
)

find_package(GTest REQUIRED)

catkin_package(
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_executable(localization_action_server src/localization_action_server.cpp)
target_link_libraries(localization_action_server tf_manipulator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(localization_action_server ${catkin_EXPORTED_TARGETS})

add_executable(localization_action_client src/localization_action_client.cpp)
target_link_libraries(localization_action_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(localization_action_client ${catkin_EXPORTED_TARGETS})


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_library)
endif()
