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

#include <tfr_msgs/ExampleAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<tfr_msgs::ExampleAction> Client;

// Called every time feedback is received for the goal
void Feedback(const tfr_msgs::ExampleFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback: %f", feedback->feedback);
}

// Called once when the goal completes
void Finished(const actionlib::SimpleClientGoalState& state, const tfr_msgs::ExampleResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %f", result->result);
    ros::shutdown(); // This will terminate the node
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_client");
    
    // The defined Client namespace (do_action) should be the same as the 
    // defined Server namespace.
    // True --> ros::spin() will be called automatically in a loop
    // If this node needs to do work while waiting for a result from
    // an ActionServer, set this to false and call ros::spinOnce() in
    // your own loop.
    Client client("do_action", true);

    // Ensure connection was established with Server
    client.waitForServer();

    // Members of ExampleGoal are defined in tfr_msgs/action/Example.action
    tfr_msgs::ExampleGoal goal;
    goal.goal = "Do the thing!";

    // Callback functions: Result, Start, Feedback
    client.sendGoal(goal, &Finished, NULL, &Feedback);

    ros::spin();

    return 0;
}
