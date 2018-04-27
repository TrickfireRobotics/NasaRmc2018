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
#include "digging_queue.h"

typedef actionlib::SimpleActionServer<tfr_msgs::DiggingAction> Server;
typedef actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> Client;

class DiggingActionServer {
public:
    DiggingActionServer(ros::NodeHandle &nh, ros::NodeHandle &p_nh) :
        priv_nh{p_nh}, queue{priv_nh}, 
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
                goal.pose.resize(4);
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
            }
        }

        ROS_INFO("Moving arm to safe driving/dumping position");

        tfr_msgs::ArmMoveGoal final_goal;
        final_goal.pose.resize(4);

        std::vector<double> final_angles;
        if (!priv_nh.getParam("positions/safe", final_angles))
        {
            // Couldn't load parameter, go to predetermined final position
            final_goal.pose[0] = 0.0;
            final_goal.pose[1] = 0.21;
            final_goal.pose[2] = 0.0;
            final_goal.pose[3] = 0.0;
        } else
        {
            final_goal.pose[0] = final_angles[0];
            final_goal.pose[1] = final_angles[1];
            final_goal.pose[2] = final_angles[2];
            final_goal.pose[3] = final_angles[3];
        }

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
