/****************************************************************************************
 * File:    test_digging_client.cpp
 * Node:    test_digging_client
 * 
 * Purpose: This is a test node to try calling the digging action server.
 *          The test sends a "dig" action to the digging action server, then waits for
 *          the user to press any key. When a key is pressed the digging action server
 *          should cancel the digging action.
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
#include <termios.h>

typedef actionlib::SimpleActionClient<tfr_msgs::DiggingAction> Client;

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
    goal.diggingTime = ros::Duration(420.0);

    // Callback functions: Result, Start, Feedback
    //note we must use NULL not nullptr, or boost error
    client.sendGoal(goal, &finished, NULL, &feedback);
    ros::Rate rate(50.0);

    while (ros::ok())
    {
        if (getch())
        {
            client.cancelAllGoals();
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
