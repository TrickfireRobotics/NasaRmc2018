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
#include <iostream>
#include <termios.h>

typedef actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> Client;

// Non-blocking input function (useful for testing preemption)
char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if (rv > 0)
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

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
    //spin.start();
    
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
        client.sendGoal(goal, NULL, NULL, NULL);
        ros::Rate rate(50.0);
        bool breaking = false;
        while ((client.getState() == actionlib::SimpleClientGoalState::ACTIVE
               || client.getState() == actionlib::SimpleClientGoalState::PENDING) && ros::ok())
        {
            if (getch())
            {
                breaking = true;
                ROS_INFO("Preempting Arm Action Server");
                // Preempt, the program is being killed
                client.cancelAllGoals();
                break;
            }

            rate.sleep();
            ros::spinOnce();
        }
        ROS_INFO("State: %s", client.getState().toString().c_str());
        if (!ros::ok() || breaking)
        {
            break;
        }
    }

    return 0;
}
