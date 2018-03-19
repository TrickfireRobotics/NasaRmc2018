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
 *  - ~image_service: the service to grab the image from  (string, default: "")
 * */

#include <ros/ros.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_action_server");
    ros::NodeHandle n{};
    double turn_speed, turn_duration;
    std::string image_service;
    ros::param::param<double>("~turn_speed", turn_speed, 0.0);
    ros::param::param<double>("~turn_duration", turn_duration, 0.0);
    ros::param::param<std::string>("~image_service", image_service, "");
    if (turn_speed == 0.0 || turn_duration == 0.0 || image_service.compare("") == 0)
        ROS_WARN("Localization Action Server: Uninitialized Parameters");
    ROS_INFO("%f %f %s", turn_speed, turn_duration, image_service.c_str());
    return 0;
}
