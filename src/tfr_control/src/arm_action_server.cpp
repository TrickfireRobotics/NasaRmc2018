/****************************************************************************************
 * File:    arm_action_server.cpp
 * Node:    arm_action_server
 * 
 * Purpose: This ActionServer is what handles the movement of the arm to given points.
 *          It's tasked with interfacing with the move_group node to execute these
 *          actions.
 * 
 *          This file includes <tfr_msgs/ArmMove.h>, which is one of seven headers
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
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <actionlib/server/simple_action_server.h>
#include <tfr_msgs/ArmMoveAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <mutex>

//typedef actionlib::SimpleActionServer<tfr_msgs::ArmMoveAction> Server;
typedef moveit::planning_interface::MoveItErrorCode MoveItErrorCode;

class ArmActionServer {
public:
    ArmActionServer(ros::NodeHandle &n) : move_group{"arm_end"}, joint_model_group(*move_group.getCurrentState()->getJointModelGroup("arm_end")),
        server{n, "move_arm", boost::bind(&ArmActionServer::execute, this, _1), false}
    {
        server.start();
        result_sub = n.subscribe("arm_controller/follow_joint_trajectory/result", 1, &ArmActionServer::resultCallback, this);
    }

private:
    void resultCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr &msg)
    {
        digging_mutex.lock();
        if (msg->result.error_code == 0)
        {
            dig_status = 0;
        }
        else
        {
            dig_status = 1;
        }
        digging_mutex.unlock();
    }

    // This is the method that will be called when a client makes use
    // of this server. The provided goal is the "input"
    void execute(const tfr_msgs::ArmMoveGoalConstPtr& goal)
    {
        ROS_INFO("Executing!");
        // Set up the joint space goal vector to travel to based on the input goal
        // from the action server
        std::vector<double> joint_group_positions(4);
        joint_group_positions[0] = goal->pose[0];
        joint_group_positions[1] = goal->pose[1];
        joint_group_positions[2] = goal->pose[2];
        joint_group_positions[3] = goal->pose[3];

        // Set the current target
        move_group.setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // Try to plan to the given target
        bool success = (move_group.plan(my_plan) == MoveItErrorCode::SUCCESS);

        // Reset the done flag, just in case
        digging_mutex.lock();
        dig_status = -1;
        digging_mutex.unlock();

        if (success)
        {
            // Planning was successful, actually execute the movement
            ROS_INFO("Executing movement");
            move_group.asyncExecute(my_plan);

            ros::Rate rate(50);

            while (true)
            {
                digging_mutex.lock();
                if (dig_status >= 0)  // We're done digging (or it errored), break the loop
                {
                    ROS_INFO("Digging done, breaking!");
                    digging_mutex.unlock();
                    break;
                }
                digging_mutex.unlock();

                if (server.isPreemptRequested())
                {
                    ROS_INFO("Preempting Arm Action Server");
                    move_group.stop();
                    // Get the current position to return in the result message
                    std::vector<double> current_pos;
                    move_group.getCurrentState()->copyJointGroupPositions(&joint_model_group, current_pos);

                    // Populate the result message
                    tfr_msgs::ArmMoveResult result;
                    result.pose.resize(4);
                    result.pose[0] = current_pos[0];
                    result.pose[1] = current_pos[1];
                    result.pose[2] = current_pos[2];
                    result.pose[3] = current_pos[3];
                    server.setPreempted(result);
                    ROS_INFO("Done preempting");
                    digging_mutex.lock();
                    dig_status = -1;
                    digging_mutex.unlock();
                    return;
                }
                rate.sleep();
            }
        } else
        {
            ROS_WARN("Planning of movement failed, not executing");
        }

        // Get the current position to return in the result message
        std::vector<double> current_pos;
        move_group.getCurrentState()->copyJointGroupPositions(&joint_model_group, current_pos);

        // Populate the result message
        tfr_msgs::ArmMoveResult result;
        result.pose.resize(4);
        result.pose[0] = current_pos[0];
        result.pose[1] = current_pos[1];
        result.pose[2] = current_pos[2];
        result.pose[3] = current_pos[3];

        // Send the result message and set the appropriate action server status
        // if both planning and execution was successful. It may be worthwile to
        // add a distinction here later (aka "did the motors fail? or is this
        // literally somewhere we aren't allowed to move?"), but that's for
        // later if we determine we need it.

        // Add a delay to ensure that the arm has time to get there within time tolerances
        // (found an issue where if you send a command too fast afterwards, it has an issue
        // getting the state and processing fast enough)
        ros::Duration(0.5).sleep();

        if (success && dig_status == 0)
        {
            ROS_INFO("Successful!");
            server.setSucceeded(result);
        } else
        {
            ROS_INFO("Unsuccessful!...");
            server.setAborted(result);
        }

        digging_mutex.lock();
        dig_status = -1;
        digging_mutex.unlock();
    }

    moveit::planning_interface::MoveGroupInterface move_group;
    const robot_state::JointModelGroup joint_model_group;
    actionlib::SimpleActionServer<tfr_msgs::ArmMoveAction> server;
    ros::Subscriber result_sub;

    std::mutex digging_mutex;
    // < 0 = in_progress, 0 = successful, > 0 = errored
    int dig_status;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_action_server");
    ros::NodeHandle n;

    // An async spinner is required here for the MoveIt setup to connect properly
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ArmActionServer aas(n);

    while (ros::ok()) ;
    return 0;
}
