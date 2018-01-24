#include<ros/ros.h>
#include<navigator.h>
/**
 * Main entry point for the navigation action server, spins up the server and
 * awaits callbacks.
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_action_server");
    ros::NodeHandle n;
    Navigator navigator(n, ros::this_node::getName());
    ros::spin();
    return 0;
}

