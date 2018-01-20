/****************************************************************************************
 * File:    arm_action_server.cpp
 * Node:    arm_action_server
 * 
 * Purpose: This ActionServer is what handles the movement of the arm to given points.
 *          It's tasked with interfacing with the move_group node to execute these
 *          actions.
 * 
 *          This file includes <tfr_msgs/Arm.h>, which is one of seven headers
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
#include <tfr_msgs/ArmMoveAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

typedef actionlib::SimpleActionServer<tfr_msgs::ArmMoveAction> Server;

static const std::string kPlanningGroup = "arm_end";
moveit::planning_interface::MoveGroupInterface move_group(kPlanningGroup);
const robot_state::JointModelGroup *joint_model_group;

// This is the method that will be called when a client makes use
// of this server. The provided goal is the "input".
void execute(const tfr_msgs::ArmMoveGoalConstPtr& goal, Server* server)
{
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_action_server");
    ros::NodeHandle n;

    joint_model_group = move_group.getCurrentState()->getJointModelGroup(kPlanningGroup);

    // The defined Server namespace (do_action) should be the same as the 
    // defined Client namespace.
    Server server(n, "move_arm", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}