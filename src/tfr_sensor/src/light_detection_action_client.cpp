#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>
#include <tfr_msgs/LightDetectAction.h>

/**
 *  Test code to never be used again, demonstrates light detection functionality
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "light_detection_action_client");
    ros::NodeHandle n;
    actionlib::SimpleActionClient<tfr_msgs::LightDetectAction> client(
            "light_detection", true);

    ROS_INFO("waiting for detection action server to start");
    client.waitForServer();
    ROS_INFO("client connected");
    tfr_msgs::LightDetectGoal goal{};
    
    client.sendGoal(goal);
    ros::Duration(0.5).sleep();
    client.cancelAllGoals();
    ros::Duration(0.5).sleep();

    client.sendGoal(goal);
    while (!client.waitForResult(ros::Duration(0.25)));
    ROS_INFO("light detected! success");
    return 0;
}


