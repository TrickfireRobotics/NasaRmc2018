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
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

typedef actionlib::SimpleActionServer<tfr_msgs::ArmMoveAction> Server;

static const std::string kPlanningGroup = "arm_end";
moveit::planning_interface::MoveGroupInterface *move_group;
const robot_state::JointModelGroup *joint_model_group;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

// This is the method that will be called when a client makes use
// of this server. The provided goal is the "input".
void execute(const tfr_msgs::ArmMoveGoalConstPtr& goal, Server* server)
{
    std::vector<double> joint_group_positions(4);
    joint_group_positions[0] = goal->pose[0];
    joint_group_positions[1] = goal->pose[1];
    joint_group_positions[2] = goal->pose[2];
    joint_group_positions[3] = goal->pose[3];
    move_group->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_DEBUG("Executing movement");
        move_group->move();
        // TODO: How are errors handled?
    } else {
        ROS_WARN("Planning of movement failed, not executing");
    }

    std::vector<double> current_pos;
    move_group->getCurrentState()->copyJointGroupPositions(joint_model_group, current_pos);

    tfr_msgs::ArmMoveResult result;
    result.pose.resize(4);
    result.pose[0] = current_pos[0];
    result.pose[1] = current_pos[1];
    result.pose[2] = current_pos[2];
    result.pose[3] = current_pos[3];

    if (success) {
        server->setSucceeded(result);
    } else {
        server->setAborted(result);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_action_server");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface mg(kPlanningGroup);
    move_group = &mg;

    moveit::planning_interface::PlanningSceneInterface psi;
    planning_scene_interface = &psi;

    joint_model_group = move_group->getCurrentState()->getJointModelGroup(kPlanningGroup);

    // The defined Server namespace (do_action) should be the same as the 
    // defined Client namespace.
    Server server(n, "move_arm", boost::bind(&execute, _1, &server), false);
    server.start();
    while (ros::ok()) ;
    return 0;
}