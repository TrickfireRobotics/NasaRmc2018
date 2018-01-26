/****************************************************************************************
 * File:            drivebase_publisher.cpp
 * 
 * Purpose:         This is the implementation file for the DrivebasePublisher class.
 *                  See tfr_control/include/tfr_control/drivebase_publisher.h for details.
 ***************************************************************************************/
#include "drivebase_publisher.h"
#include "std_msgs/Float64.h"

namespace tfr_control
{
    DrivebasePublisher::DrivebasePublisher(
        ros::NodeHandle& n, float wheel_radius, float wheel_span) : 
        n{n}, wheel_radius{wheel_radius}, wheel_span{wheel_span}, 
        left_tread_publisher{}, right_tread_publisher{}
    {
        left_tread_publisher = n.advertise<std_msgs::Float64>(
            "left_tread_velocity_controller/command", 100);

        right_tread_publisher = n.advertise<std_msgs::Float64>(
            "right_tread_velocity_controller/command", 100);

        subscriber = n.subscribe("cmd_vel", 100, &DrivebasePublisher::subscriptionCallback, this);
    }

    void DrivebasePublisher::subscriptionCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        float left_tread, right_tread;

        DrivebasePublisher::twistToDifferential(msg->linear.x, msg->angular.z,
            this->wheel_span, this->wheel_radius, left_tread, right_tread);
        
        std_msgs::Float64 left_msg;
        std_msgs::Float64 right_msg;
        left_msg.data = left_tread;
        right_msg.data = right_tread;
        left_tread_publisher.publish(left_msg);
        right_tread_publisher.publish(right_msg);
    }

    // left_tread and right_tread are output parameters; the rest are inputs.
    void DrivebasePublisher::twistToDifferential(const float linear_v, const float angular_v,
        const float wheel_radius, const float wheel_span, float& left_tread, float& right_tread)
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
        float left_velocity = linear_v - (wheel_span * angular_v) / 2;
        float right_velocity = linear_v + (wheel_span * angular_v) / 2;

        // Convert linear velocity to angular velocity of the wheel
        left_tread = left_velocity / wheel_radius;
        right_tread = right_velocity / wheel_radius;
    }

}
