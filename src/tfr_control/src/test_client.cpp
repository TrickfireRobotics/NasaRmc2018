/****************************************************************************************
 * File:    example_action_client.cpp
 * Node:    example_client
 * 
 * Purpose: This is a example of what an ActionServer client node might look like.
 *          It includes functions likely to be used in our production code for
 *          ActionServer clients, but does not include every function available
 *          to a client. If you are adapting this code for your system, please
 *          read the ROS API documentation on actionlib::SimpleActionClient<> at 
 *          http://docs.ros.org/api/actionlib/html/classactionlib_1_1SimpleActionClient.html
 * 
 *          This file includes <tfr_msgs/ExampleAction.h>, which is one of seven headers
 *          built by catkin from `tfr_msgs/action/Example.action`:
 *              devel/include/tfr_msgs/ExampleAction.h
 *              devel/include/tfr_msgs/ExampleActionFeedback.h
 *              devel/include/tfr_msgs/ExampleActionGoal.h
 *              devel/include/tfr_msgs/ExampleActionResult.h
 *              devel/include/tfr_msgs/ExampleFeedback.h
 *              devel/include/tfr_msgs/ExampleGoal.h
 *              devel/include/tfr_msgs/ExampleResult.h
 * 
 ***************************************************************************************/

#include <tfr_msgs/ArmMoveAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include <vector>

typedef actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> Client;

// Called once when the goal completes
void finished(const actionlib::SimpleClientGoalState& state, const tfr_msgs::ArmMoveResultConstPtr& result)
{
    ROS_INFO("Action succeeded: %s", state.getText());
    ros::shutdown(); // This will terminate the node
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_client");
    Client client("move_arm", true);

    // Ensure connection was established with Server
    client.waitForServer();

    // Members of ArmMoveGoal are defined in tfr_msgs/action/ArmMove.action
    tfr_msgs::ArmMoveGoal goal;
    goal.pose.resize(4);
    goal.pose[0] = 0.0;
    goal.pose[1] = 1.65;
    goal.pose[2] = 0.0;
    goal.pose[3] = -0.85;

    // Callback functions: Result, Start, Feedback
    client.sendGoal(goal, &finished, NULL, NULL);

    ros::spin();

    return 0;
}
