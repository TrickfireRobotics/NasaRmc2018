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

namespace tfr_control
{
    class DrivebasePublisher
    {
    public:
        DrivebasePublisher() = delete;
        explicit DrivebasePublisher(ros::NodeHandle& n, float wheel_span, float wheel_radius);
        DrivebasePublisher(const DrivebasePublisher& other) = delete;
        DrivebasePublisher(DrivebasePublisher&&) = delete;

        ~DrivebasePublisher() = default;

        DrivebasePublisher& operator=(const DrivebasePublisher&) = delete;
        DrivebasePublisher& operator=(DrivebasePublisher&&) = delete;
        

        // Making this static allows for unit-testing without 
        // creating a DrivebasePublisher object
        // left_tread and right_tread are output parameters; the rest are inputs.
        static void twistToDifferential(const float linear_v, const float angular_v, const float wheel_radius, 
                                        const float wheel_span, float& left_tread, float& right_tread);

    private:
        void subscriptionCallback(const geometry_msgs::Twist::ConstPtr& msg);

        ros::NodeHandle& n;
        const float wheel_radius;
        const float wheel_span;

        ros::Publisher left_tread_publisher;
        ros::Publisher right_tread_publisher;
        ros::Subscriber subscriber;
    };
}

#endif // DRIVEBASE_PUBLISHER_H
