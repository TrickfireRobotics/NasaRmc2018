cmake_minimum_required(VERSION 2.8.3)
project(tfr_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  geometry_msgs
  actionlib_msgs
  actionlib
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Echo.msg
  SystemStatus.msg
  ArduinoAReading.msg
  ArduinoBReading.msg
  PwmCommand.msg
)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   EmptySrv.srv
   QuerySrv.srv
   ArmStateSrv.srv
   BinStateSrv.srv
   DurationSrv.srv
   PoseSrv.srv
   WrappedImage.srv
   SetOdometry.srv
 )

# Generate actions in the 'action' folder
add_action_files(
  FILES
  Teleop.action
  Example.action
  Navigation.action
  Localization.action
  Bin.action
  Aruco.action
  ArmMove.action
  Empty.action
  Digging.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
  geometry_msgs
)

# These are all for exporting to dependent packages/projects.
# Uncomment each if the dependent project requires it
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES tfr_msgs
#  CATKIN_DEPENDS roscpp std_msgs message_runtime actionlib actionlib_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
