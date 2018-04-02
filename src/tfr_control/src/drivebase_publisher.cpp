/****************************************************************************************
 * File:            drivebase_publisher.cpp
 * 
 * Purpose:         This is the implementation file for the DrivebasePublisher class.
 *                  See tfr_control/include/tfr_control/drivebase_publisher.h for details.
 ***************************************************************************************/
#include "drivebase_publisher.h"

namespace tfr_control
{
    DrivebasePublisher::DrivebasePublisher(
        ros::NodeHandle& n, double wheel_radius, double wheel_span) : 
        n{n}, wheel_radius{wheel_radius}, wheel_span{wheel_span}, 
        left_tread_publisher{}, right_tread_publisher{}
    {
        left_tread_publisher = n.advertise<std_msgs::Float64>(
            "left_tread_velocity_controller/command", 5);
        right_tread_publisher = n.advertise<std_msgs::Float64>(
            "right_tread_velocity_controller/command", 5);


        subscriber = n.subscribe("cmd_vel", 100, &DrivebasePublisher::subscriptionCallback, this);
    }

    void DrivebasePublisher::subscriptionCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {

        double left_velocity = msg->linear.x - (wheel_span * msg->angular.z) / 2;
        double right_velocity = msg->linear.x + (wheel_span * msg->angular.z) / 2;

        std_msgs::Float64 left_cmd;
        left_cmd.data = left_velocity;
        std_msgs::Float64 right_cmd;
        right_cmd.data = right_velocity;
        left_tread_publisher.publish(left_cmd);
        right_tread_publisher.publish(right_cmd);
    }

}
