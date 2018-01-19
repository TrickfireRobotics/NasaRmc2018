/****************************************************************************************
 * File:            drivebase.cpp
 * 
 * Purpose:         This node converts Twist data from the /cmd_vel topic to angular
 *                  velocity of the two motor sets (left and right). Most of the
 *                  responsibilities are delegated to the DrivebasePublisher class.
 * 
 * Launched By:     drivebase.launch
 ***************************************************************************************/
#include "ros/ros.h"
#include "drivebase_publisher.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "drivebase");

    ros::NodeHandle n;
    float axle_length, wheel_radius;
    n.getParam("axle_length", axle_length);
    n.getParam("wheel_radius", wheel_radius);
    
    tfr_control::DrivebasePublisher publisher(n, axle_length, wheel_radius);

    ros::spin();
    return 0;
}
