/****************************************************************************************
 * File:    test_digging_client.cpp
 * Node:    test_digging_client
 * 
 * Purpose: This is a test node to try calling the digging action server.
 * 
 *          This file includes <tfr_msgs/DiggingAction.h>, which is one of seven headers
 *          built by catkin from `tfr_msgs/action/Digging.action`:
 *              devel/include/tfr_msgs/DiggingAction.h
 *              devel/include/tfr_msgs/DiggingActionFeedback.h
 *              devel/include/tfr_msgs/DiggingActionGoal.h
 *              devel/include/tfr_msgs/DiggingActionResult.h
 *              devel/include/tfr_msgs/DiggingFeedback.h
 *              devel/include/tfr_msgs/DiggingGoal.h
 *              devel/include/tfr_msgs/DiggingResult.h
 * 
 ***************************************************************************************/

#include <tfr_msgs/DiggingAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<tfr_msgs::DiggingAction> Client;

// Called every time feedback is received for the goal
void feedback(const tfr_msgs::DiggingFeedbackConstPtr& feedback)
{

}

// Called once when the goal completes
void finished(const actionlib::SimpleClientGoalState& state, const tfr_msgs::DiggingResultConstPtr& result)
{
    ROS_INFO("Finished!");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_digging_client");
    
    Client client("dig", true);

    // Ensure connection was established with Server
    client.waitForServer();

    // Members of ExampleGoal are defined in tfr_msgs/action/Digging.action
    tfr_msgs::DiggingGoal goal;
    goal.diggingTime = ros::Duration(40.0);

    // Callback functions: Result, Start, Feedback
    //note we must use NULL not nullptr, or boost error
    client.sendGoal(goal, &finished, NULL, &feedback);

    ros::spin();

    return 0;
}
