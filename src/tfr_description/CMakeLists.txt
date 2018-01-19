cmake_minimum_required(VERSION 2.8.3)
project(tfr_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tfr_description
#  CATKIN_DEPENDS joint_state_publisher robot_state_publisher
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
