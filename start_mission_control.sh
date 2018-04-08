#!/bin/bash
export ROS_MASTER_URI=http://192.168.1.3:11311
export ROS_IP=192.168.1.4

#roslaunch for mission control goes here
. devel/setup.bash
roslaunch tfr_launch mission_control.launch

