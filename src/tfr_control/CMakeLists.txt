# If you're having trouble getting your things to build, visit the ROS
# tutorials at <http://wiki.ros.org/ROS/Tutorials> and also read the ROS
# CMake documentation at <http://wiki.ros.org/catkin/CMakeLists.txt>.

cmake_minimum_required(VERSION 2.8.3)
project(tfr_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  tfr_msgs
  tfr_utilities
  hardware_interface
  controller_manager
  joint_state_controller
  effort_controllers
  joint_trajectory_controller
  moveit_ros_planning_interface
)

find_package(GTest REQUIRED)

# These are all for exporting to dependent packages/projects.
# Uncomment each if the dependent project requires it
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tfr_control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

# controller_launcher
add_executable(control
  src/control.cpp
  src/robot_interface.cpp
)
add_dependencies(control  tfr_msgs_gencpp)
target_link_libraries(control 
  ${catkin_LIBRARIES}
)

# drivebase
add_executable(drivebase
  src/drivebase.cpp
  src/drivebase_publisher.cpp
)
target_link_libraries(drivebase ${catkin_LIBRARIES})
add_dependencies(drivebase tfr_msgs_gencpp)

add_executable(arm_action_server src/arm_action_server.cpp)
add_dependencies(arm_action_server tfr_msgs_gencpp)
target_link_libraries(arm_action_server
  ${catkin_LIBRARIES}
)

# This call is sometimes needed and sometimes not and I'm not really clear why
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
