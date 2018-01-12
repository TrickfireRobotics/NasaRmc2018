cmake_minimum_required(VERSION 2.8.3)
project(xsens_custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy roscpp std_msgs message_generation sensor_msgs geometry_msgs
)

add_message_files(
   FILES
   XsensQuaternion.msg
   ImuSensorSample.msg
   BaroSensorSample.msg   
   GnssSensorSample.msg
   Internal.msg
   sensorSample.msg
   baroSample.msg
   gnssSample.msg
   orientationEstimate.msg
   velocityEstimate.msg
   positionEstimate.msg		   
 )

generate_messages(
   DEPENDENCIES
   std_msgs  
   sensor_msgs
 )

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs sensor_msgs message_generation message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
