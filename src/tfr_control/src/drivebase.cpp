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
    float wheel_span, wheel_radius;
    if (!n.getParam("wheel_span", wheel_span))
    {
        // Parameter error
        ROS_ERROR("Parameter 'wheel_span' not found for node 'drivebase'");
        return 1;
    }
    else if (wheel_span <= 0)
    {
        ROS_ERROR("Parameter 'wheel_span' must be a positive value.");
        return 1;
    }

    if (!n.getParam("wheel_radius", wheel_radius))
    {
        // Parameter error
        ROS_ERROR("Parameter 'wheel_radius' not found for node 'drivebase'");
        return 1;
    }
    else if (wheel_radius <= 0)
    {
        ROS_ERROR("Parameter 'wheel_radius' must be a positive value.");
        return 1;
    }
    
    tfr_control::DrivebasePublisher publisher(n, wheel_span, wheel_radius);

    ros::spin();
    return 0;
}
