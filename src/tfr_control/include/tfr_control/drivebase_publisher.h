/****************************************************************************************
 * File:            drivebase_publisher.h
 * 
 * Purpose:         This class is used by the drivebase node to convert from a Twist
 *                  message in /cmd_vel to angular velocity for the two motor sets
 *                  (left and right), handling the calculations for differential steering.
 * 
 *                  Every message read from /cmd_vel will result in two messages sent,
 *                  one to each motor controller. This class will only publish after
 *                  reading a message on /cmd_vel.
 * 
 * Subscribed To:   /cmd_vel
 * Publishes To:    /left_tread_velocity_controller/command
 *                  /right_tread_velocity_controller/command
 ***************************************************************************************/
#ifndef DRIVEBASE_PUBLISHER_H
#define DRIVEBASE_PUBLISHER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
namespace tfr_control
{
    class DrivebasePublisher
    {
    public:
        DrivebasePublisher() = delete;
        explicit DrivebasePublisher(ros::NodeHandle& n, double wheel_span, double wheel_radius);
        DrivebasePublisher(const DrivebasePublisher& other) = delete;
        DrivebasePublisher(DrivebasePublisher&&) = delete;

        ~DrivebasePublisher() = default;

        DrivebasePublisher& operator=(const DrivebasePublisher&) = delete;
        DrivebasePublisher& operator=(DrivebasePublisher&&) = delete;
        

    private:
        void subscriptionCallback(const geometry_msgs::Twist::ConstPtr& msg);

        ros::NodeHandle& n;
        const double wheel_radius;
        const double wheel_span;

        ros::Publisher left_tread_publisher;
        ros::Publisher right_tread_publisher;
        ros::Subscriber subscriber;
    };
}

#endif // DRIVEBASE_PUBLISHER_H
