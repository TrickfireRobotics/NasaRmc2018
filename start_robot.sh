#!/bin/bash
export ROS_MASTER_URI=http://192.168.1.3:11311
export ROS_IP=192.168.1.3

#roslaunch for robot goes here
. devel/setup.bash
roslaunch tfr_launch robot.launch
