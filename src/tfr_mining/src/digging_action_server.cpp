/****************************************************************************************
 * File:    digging_action_server.cpp
 * Node:    digging_server
 * 
 * Purpose: This is an action server that handles all of the digging subsystem.
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

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tfr_msgs/DiggingAction.h>  // Note: "Action" is appended
#include <tfr_msgs/ArmMoveAction.h>  // Note: "Action" is appended
#include <geometry_msgs/Twist.h>
#include <tfr_utilities/teleop_code.h>
#include <actionlib/client/simple_action_client.h>
#include "digging_queue.h"

typedef actionlib::SimpleActionServer<tfr_msgs::DiggingAction> Server;
typedef actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> Client;

class DiggingActionServer {
public:
    DiggingActionServer(ros::NodeHandle &nh, ros::NodeHandle &p_nh) :
        priv_nh{p_nh}, queue{priv_nh}, 
        drivebase_publisher{nh.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
        server{nh, "dig", boost::bind(&DiggingActionServer::execute, this, _1), false}
    {
        server.start();
    }

private:

    void execute(const tfr_msgs::DiggingGoalConstPtr& goal)
    {
        ROS_INFO("Executing!");
        ROS_INFO("Allowed %d seconds to dig.", goal->diggingTime.sec);
        ros::Time startTime = ros::Time::now();
        ros::Time endTime = startTime + goal->diggingTime;
        ROS_INFO("Starting at %d, ending at %d", startTime.sec, endTime.sec);

        Client client("move_arm", true);
        ROS_DEBUG("Waiting for arm action server...");
        client.waitForServer();
        ROS_DEBUG("Connected with arm action server");

        while (!queue.isEmpty())
        {
            ROS_INFO("Time remaining: %f", (endTime - ros::Time::now()).toSec());
            tfr_mining::DiggingSet set = queue.popDiggingSet();
            ros::Time now = ros::Time::now();

            ROS_INFO("starting set");
            // If we don't have enough time, bail on the action and exit
            if ((endTime - now).toSec() < set.getTimeEstimate())
            {
                ROS_INFO("Not enough time to complete the next digging set, exiting. cost: %f remaining: %f",
                        set.getTimeEstimate(), (endTime - now).toSec() );
                break;
            }

            while (!set.isEmpty())
            {
                std::vector<double> state = set.popState();
                tfr_msgs::ArmMoveGoal goal;
                goal.pose.resize(5);
                goal.pose[0] = state[0];
                goal.pose[1] = state[1];
                goal.pose[2] = state[2];
                goal.pose[3] = state[3];

                ROS_INFO("goal %f %f %f %f", goal.pose[0], goal.pose[1], goal.pose[2], goal.pose[3]);

                client.sendGoal(goal);
                ros::Rate rate(10.0);

                while (!client.getState().isDone() && ros::ok())
                {
                    if (server.isPreemptRequested())
                    {
                        ROS_INFO("Preempting digging action server");
                        client.cancelAllGoals();
                        tfr_msgs::DiggingResult result;
                        server.setPreempted(result);
                        return;
                    }

                    rate.sleep();
                }

                
                if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_WARN("Error executing arm action server to state, exiting.");
                    tfr_msgs::DiggingResult result;
                    server.setAborted(result);
                }

                if (std::abs(goal.pose[0]) < 3.14159265/2) { // If the turntable is going to around the bin (the problem area)
                    ros::Duration(1.5).sleep(); // Setting this to 2 seconds works for sure
                }
                
                
                if (std::abs(state[4]) > 1.05 )
                {
                    geometry_msgs::Twist pulse;
                    pulse.linear.x = -0.2;
                    drivebase_publisher.publish(pulse);
                    ros::Duration(0.75).sleep(); 
                    pulse.linear.x = 0;
                    drivebase_publisher.publish(pulse);
                }
                else if (std::abs(state[4]) > 0.05)
                {
                    ros::Duration(1.25).sleep(); 
                }
            }
        }

        ROS_INFO("Moving arm to safe driving/dumping position");

        tfr_msgs::ArmMoveGoal final_goal;
        final_goal.pose.resize(5);

        std::vector<double> final_angles;
        // Couldn't load parameter, go to predetermined final position
        final_goal.pose[0] = 0.00;
        final_goal.pose[1] = 0.21;
        final_goal.pose[2] = 1.07;
        final_goal.pose[3] = 0.0;
        final_goal.pose[4] = 0.0;


        client.sendGoal(final_goal);
        ros::Rate rate(10.0);

        while (client.getState().isDone() && ros::ok())
        {
            if (server.isPreemptRequested())
            {
                ROS_INFO("Preempting digging action server");
                client.cancelAllGoals();
                tfr_msgs::DiggingResult result;
                server.setPreempted(result);
                return;
            }

            rate.sleep();
        }
        if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_WARN("Error moving arm to final position, exiting.");
            tfr_msgs::DiggingResult result;
            server.setAborted(result);
            return;
        }


        tfr_msgs::DiggingResult result;
        server.setSucceeded(result);
    }

    ros::NodeHandle &priv_nh;
    ros::Publisher drivebase_publisher;
 
    tfr_mining::DiggingQueue queue;
    Server server;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "digging_server");
    ros::NodeHandle n;
    ros::NodeHandle p_n("~");

    DiggingActionServer server(n, p_n);
    ros::spin();
    return 0;
}
