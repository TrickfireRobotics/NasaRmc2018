/*
 * Test code to never be seen again, used to test teleop action server
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_utilities/teleop_code.h>
#include <tfr_msgs/TeleopAction.h>
#include <actionlib/client/simple_action_client.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_action_client");
    ros::NodeHandle n{};
    actionlib::SimpleActionClient<tfr_msgs::TeleopAction>
        client{"teleop_action_server", true};
    ROS_INFO("Teleop Action Client: Connecting to server");
    client.waitForServer();
    ROS_INFO("Teleop Action Client: Connected to server");

    tfr_msgs::TeleopGoal goal{};

    ROS_INFO("Simple Action Client: sending FORWARD");
    goal.code = static_cast<uint8_t>(tfr_utilities::TeleopCode::FORWARD);
    client.sendGoal(goal);
    ROS_INFO("Simple Action Client: finished FORWARD");

    ROS_INFO("Simple Action Client: sending BACKWARD");
    goal.code = static_cast<uint8_t>(tfr_utilities::TeleopCode::BACKWARD);
    client.sendGoal(goal);
    ROS_INFO("Simple Action Client: finished BACKWARD");

    ROS_INFO("Simple Action Client: sending LEFT");
    goal.code = static_cast<uint8_t>(tfr_utilities::TeleopCode::LEFT);
    client.sendGoal(goal);
    ROS_INFO("Simple Action Client: finished LEFT");

    ROS_INFO("Simple Action Client: sending RIGHT");
    goal.code = static_cast<uint8_t>(tfr_utilities::TeleopCode::RIGHT);
    client.sendGoal(goal);
    ROS_INFO("Simple Action Client: finished RIGHT");

    ROS_INFO("Simple Action Client: sending DIG");
    goal.code = static_cast<uint8_t>(tfr_utilities::TeleopCode::DIG);
    client.sendGoal(goal);
    ROS_INFO("Simple Action Client: finished DIG");

    ROS_INFO("Simple Action Client: sending DUMP");
    goal.code = static_cast<uint8_t>(tfr_utilities::TeleopCode::DUMP);
    client.sendGoal(goal);
    ROS_INFO("Simple Action Client: finished DUMP");

    ROS_INFO("Simple Action Client: sending RESET");
    goal.code = static_cast<uint8_t>(tfr_utilities::TeleopCode::RESET);
    client.sendGoal(goal);
    ROS_INFO("Simple Action Client: finished RESET");

    ROS_INFO("Simple Action Client: sending NONE");
    goal.code = static_cast<uint8_t>(tfr_utilities::TeleopCode::NONE);
    client.sendGoal(goal);
    ROS_INFO("Simple Action Client: finished NONE");
    return 0;
}
