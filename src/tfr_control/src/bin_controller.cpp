/****************************************************************************************
 * File:    bin_controller.cpp
 * Node:    bin_controller
 * 
 * Purpose: 
 * 
 ***************************************************************************************/

#include <actionlib/server/simple_action_server.h>
#include <tfr_msgs/BinAction.h>

typedef actionlib::SimpleActionServer<tfr_msgs::BinAction> Server;

// This is the method that will be called when a client makes use
// of this server. The provided goal is the "input".
void execute(const tfr_msgs::BinGoalConstPtr& goal, Server* server)
{
    // float sum = 0;
    // for (int i = 0; i < 20; ++i)
    // {
    //     // Feedback messages give periodic information on the progress
    //     // of a task.
    //     // The fields in tfr_msgs::ExampleFeedback are defined by Example.action
    //     tfr_msgs::ExampleFeedback message;
    //     message.feedback = i;
    //     server->publishFeedback(message);
    //     sum += i;
    // }
    
    // The fields in tfr_msgs::ExampleResult are defined by Example.action
    // tfr_msgs::BinResult result;
    // result.return_code = sum;
    
    // server->setSucceeded(result);
}

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "example_server");
    // ros::NodeHandle n;

    // // The defined Server namespace (do_action) should be the same as the 
    // // defined Client namespace.
    // Server server(n, "do_action", boost::bind(&execute, _1, &server), false);
    // server.start();
    // ros::spin();
    // return 0;
}
