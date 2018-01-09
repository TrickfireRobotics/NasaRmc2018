cmake_minimum_required(VERSION 2.8.3)
project(tfr_visualization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rviz
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tfr_visualization
#  CATKIN_DEPENDS rviz
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
