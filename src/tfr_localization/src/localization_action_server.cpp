/*
 * The action server in charge of localizing the robot.
 *
 * Takes in the empty action request, and provides no feedback.
 * Turns until it sees the aruco markers, exits succesfully once it does.
 *
 * Needs access to the image wrapper topic wrapper to fetch images, 
 * name is specified as a parameter.
 *
 * parameters:
 *  - ~turn_speed: how fast to turn [rad/s] (double, default: 0.0)
 *  - ~turn_duration: how long to turn [s] (double, default: 0.0)
 * */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tfr_msgs/ArucoAction.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/WrappedImage.h>

class Localizer
{
    public:
        Localizer(ros::NodeHandle &n) : 
            aruco{n, "aruco_action_server"},
            server{n, "localize", boost::bind(&Localizer::localize, this, _1) ,false}

        {
            ROS_INFO("Localization Action Server: Connecting Aruco");
            aruco.waitForServer();
            ROS_INFO("Localization Action Server: Connected Aruco");

            ROS_INFO("Localization Action Server: Connecting Image Client");
            image_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/rear_cam/image_raw");
            ROS_INFO("Localization Action Server: Connected Image Client");
            ROS_INFO("Localization Action Server: Starting");
            server.start();
            ROS_INFO("Localization Action Server: Started");
        };
        ~Localizer() = default;
        Localizer(const Localizer&) = delete;
        Localizer& operator=(const Localizer&) = delete;
        Localizer(Localizer&&) = delete;
        Localizer& operator=(Localizer&&) = delete;
    private:
        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        actionlib::SimpleActionClient<tfr_msgs::ArucoAction> aruco;
        ros::ServiceClient image_client;

        void localize( const tfr_msgs::EmptyGoalConstPtr &goal)
        {
            ROS_INFO("Localization Action Server: Localize Starting");
            ROS_INFO("Localization Action Server: Localize Finished");
        }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_action_server");
    ros::NodeHandle n{};
    double turn_speed, turn_duration;
    ros::param::param<double>("~turn_speed", turn_speed, 0.0);
    ros::param::param<double>("~turn_duration", turn_duration, 0.0);
    if (turn_speed == 0.0 || turn_duration == 0.0)
        ROS_WARN("Localization Action Server: Uninitialized Parameters");
    Localizer localizer(n);
    ros::spin();
    return 0;

}
