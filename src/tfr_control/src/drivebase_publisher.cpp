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
        double left_tread, right_tread;

        DrivebasePublisher::twistToDifferential(msg->linear.x, msg->angular.z,
            this->wheel_span, this->wheel_radius, left_tread, right_tread);

        std_msgs::Float64 left_cmd;
        left_cmd.data = left_tread;
        std_msgs::Float64 right_cmd;
        right_cmd.data = right_tread;
        left_tread_publisher.publish(left_cmd);
        right_tread_publisher.publish(right_cmd);
    }

    // left_tread and right_tread are output parameters; the rest are inputs.
    void DrivebasePublisher::twistToDifferential(const double linear_v, const double angular_v,
        const double wheel_radius, const double wheel_span, double& left_tread, double& right_tread)
    {
        if (wheel_radius <= 0)
        {
            throw std::invalid_argument("Wheel radius may not be zero or negative.");
        }

        if (wheel_span <= 0)
        {
            throw std::invalid_argument("Wheel span may not be zero or negative.");
        }

        // Desired velocity across the ground for each wheel
        double left_velocity = linear_v - (wheel_span * angular_v) / 2;
        double right_velocity = linear_v + (wheel_span * angular_v) / 2;

    }

}
