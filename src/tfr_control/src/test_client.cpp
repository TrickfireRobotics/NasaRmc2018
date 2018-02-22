/****************************************************************************************
 * File:    test_client.cpp
 * Node:    test_client
 * 
 * Purpose: This is a example of what an ActionServer client node might look like.
 *          It includes functions likely to be used in our production code for
 *          ActionServer clients, but does not include every function available
 *          to a client. If you are adapting this code for your system, please
 *          read the ROS API documentation on actionlib::SimpleActionClient<> at 
 *          http://docs.ros.org/api/actionlib/html/classactionlib_1_1SimpleActionClient.html
 * 
 *          This file includes <tfr_msgs/ArmMoveAction.h>, which is one of seven headers
 *          built by catkin from `tfr_msgs/action/ArmMove.action`:
 *              devel/include/tfr_msgs/ArmMoveAction.h
 *              devel/include/tfr_msgs/ArmMoveActionFeedback.h
 *              devel/include/tfr_msgs/ArmMoveActionGoal.h
 *              devel/include/tfr_msgs/ArmMoveActionResult.h
 *              devel/include/tfr_msgs/ArmMoveFeedback.h
 *              devel/include/tfr_msgs/ArmMoveGoal.h
 *              devel/include/tfr_msgs/ArmMoveResult.h
 * 
 ***************************************************************************************/

#include <tfr_msgs/ArmMoveAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <map>

typedef actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> Client;

// Called once when the goal completes
void finished(const actionlib::SimpleClientGoalState& state, const tfr_msgs::ArmMoveResultConstPtr& result)
{
    // If this goal didn't succeed (there's multiple fail states in the enum so
    // it's easier to check for this)
    if (state.state_ != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    {
        ROS_WARN("Error planning/executing motion.");
        return;
    }

    ROS_INFO("Planning and execution succeeded.");
}

int main(int argc, char** argv)
{
    // The list of positions to travel to, in order
    std::vector<std::string> position_names = { "ready", "dig", "scoop", "out", "dump", "release", "invalid" };

    // Basic ROS setup
    ros::init(argc, argv, "example_client");
    ros::NodeHandle n("~");

    // Needed to spin while we're waiting for a blocking call later (waiting for
    // server finished response)
    ros::AsyncSpinner spin(1);
    spin.start();
    
    // Set up the action server and connect
    Client client("move_arm", true);

    ROS_INFO("Connecting to action server...");
    client.waitForServer();
    ROS_INFO("Connection established with server.");

    // Loop through all of the positions to travel to and then exit
    for (auto &position : position_names)
    {
        // Get the name of the parameter in the positions namespace
        std::string param_name = "positions/" + position;
        ROS_INFO("Loading value: %s", param_name.c_str());

        // Load the parameters (an array of doubles which represent joint angles)
        std::vector<double> angles;
        if (!n.getParam(param_name, angles))
        {
            ROS_WARN("Error loading parameter %s, skipping", param_name.c_str());
            continue;
        }

        ROS_DEBUG("Loaded parameter %s successfully", param_name.c_str());
        ROS_INFO("Values: %f %f %f %f", angles[0], angles[1], angles[2], angles[3]);
        ROS_INFO("Sending values to server for execution.");
        
        // Create the goal object to send
        tfr_msgs::ArmMoveGoal goal;
        goal.pose.resize(4);
        goal.pose[0] = angles[0];
        goal.pose[1] = angles[1];
        goal.pose[2] = angles[2];
        goal.pose[3] = angles[3];

        // Execute the action
        client.sendGoal(goal, &finished, NULL, NULL);
        client.waitForResult(ros::Duration(0.0));
    }

    return 0;
}
