/*
 *  Test code never see again
 * */

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>
#include <tfr_msgs/EmptyAction.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_action_client");
    ros::NodeHandle n{};
    actionlib::SimpleActionClient<tfr_msgs::EmptyAction> client(n, "localize");
    ROS_INFO("Localization Action Client: Connecting to server");
    client.waitForServer();
    ROS_INFO("Localization Action Client: Connected to server");
    ROS_INFO("Localization Action Client: Calling server");
    tfr_msgs::EmptyGoal goal{};
    client.sendGoal(goal);
    client.waitForResult();
    ROS_INFO("Localization Action Client: Server finished");
    return 0;
}
