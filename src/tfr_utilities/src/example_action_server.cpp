/****************************************************************************************
 * File:    example_action_server.cpp
 * Node:    example_server
 * 
 * Purpose: This is a example of what an ActionServer server node might look like.
 *          It includes functions likely to be used in our production code for
 *          ActionServer servers, but does not include every function available
 *          to a server. If you are adapting this code for your system, please
 *          read the ROS API documentation on actionlib::SimpleActionServer<> at 
 *          http://docs.ros.org/api/actionlib/html/classactionlib_1_1SimpleActionServer.html
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

#include <actionlib/server/simple_action_server.h>
#include <tfr_msgs/ExampleAction.h>  // Note: "Action" is appended

typedef actionlib::SimpleActionServer<tfr_msgs::ExampleAction> Server;

// This is the method that will be called when a client makes use
// of this server. The provided goal is the "input".
void execute(const tfr_msgs::ExampleGoalConstPtr& goal, Server* server)
{
    float sum = 0;
    for (int i = 0; i < 20; ++i)
    {
        // Feedback messages give periodic information on the progress
        // of a task.
        // The fields in tfr_msgs::ExampleFeedback are defined by Example.action
        tfr_msgs::ExampleFeedback message;
        message.feedback = i;
        server->publishFeedback(message);
        sum += i;
    }
    
    // The fields in tfr_msgs::ExampleResult are defined by Example.action
    tfr_msgs::ExampleResult result;
    result.result = sum;
    
    server->setSucceeded(result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_server");
    ros::NodeHandle n;

    // The defined Server namespace (do_action) should be the same as the 
    // defined Client namespace.
    Server server(n, "do_action", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}