/*
 * Test code for the autonomous action server, never to be seen again
 * */
#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/EmptySrv.h>
#include <actionlib/client/simple_action_client.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv,  "autonomous_action_client");
    ros::NodeHandle n{};
    actionlib::SimpleActionClient<tfr_msgs::EmptyAction>
        client{"autonomous_action_server",true};
    ROS_INFO("Autonomous Action Client: connecting");
    client.waitForServer();
    ROS_INFO("Autonomous Action Client: connected");
    ROS_INFO("Autonomous Action Client: starting clock %f",
            ros::Time::now().toSec());
    tfr_msgs::EmptySrv start;
    ros::service::call("start_mission", start);
    ROS_INFO("Autonomous Action Client: clock started %f",
            ros::Time::now().toSec());

    tfr_msgs::EmptyGoal goal{};
    client.sendGoal(goal);
    client.waitForResult();
    return 0;
}
