#include <ros/ros.h>
#include <tfr_msgs/EmptyAction.h>
#include <actionlib/client/simple_action_client.h>
/**
 *test code never to be seen againb
 * */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_action_client");
    ros::NodeHandle n;
    actionlib::SimpleActionClient<tfr_msgs::EmptyAction> client("dump", true);
    ROS_INFO("client waiting for server");
    client.waitForServer();
    ROS_INFO("client connected to server");

    ros::Duration(1).sleep();
    tfr_msgs::EmptyGoal empty{};
    client.sendGoal(empty);
    client.waitForResult();
    ROS_INFO("Finished status: %s ", (client.getState() ==
                actionlib::SimpleClientGoalState::SUCCEEDED)?"success": "fail");

    return 0;
}
