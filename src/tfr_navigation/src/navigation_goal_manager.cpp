#include <navigation_goal_manager.h>

NavigationGoalManager::NavigationGoalManager(const GeometryConstraints &c, 
        uint8_t code): constraints{c}, location_code{code}
{
    initialize_goal();
    //NOTE ros is not really big on runtime exceptions,  I'll post an annoying
    //warning at startup
    if (    constraints.get_safe_mining_distance() < 0 || 
            constraints.get_mining_line_length() < 0 || 
            constraints.get_finish_line() < 0)
    {
        ROS_WARN("Mining constraints should be positive %f", 
                ros::Time::now().toSec());
        ROS_WARN("    safe_mining_distance: %f",
                constraints.get_safe_mining_distance());
        ROS_WARN("    mining_line_distance: %f",
                constraints.get_mining_line_length());
        ROS_WARN("    finish_line: %f",
                constraints.get_finish_line());
    }
    
}

NavigationGoalManager::NavigationGoalManager(const GeometryConstraints &c):
    NavigationGoalManager{c, tfr_msgs::NavigationGoal::UNSET} {}


/**
 *  Initializes the goal based on the location code, flashes a warning if it's
 *  not recognized.
 * */
move_base_msgs::MoveBaseGoal NavigationGoalManager::initialize_goal() {
    nav_goal.target_pose.header.stamp = ros::Time::now();
    //TODO integrate the bin reference frame here, upon completion of
    //the bin transform publisher
    switch(location_code)
    {
        case(tfr_msgs::NavigationGoal::TO_MINING):
            nav_goal.target_pose.pose.position.x =
                constraints.get_safe_mining_distance();
            //TODO 
            break;
        case(tfr_msgs::NavigationGoal::TO_DUMPING):
            nav_goal.target_pose.pose.position.x = constraints.get_finish_line();
            break;
        case(tfr_msgs::NavigationGoal::UNSET):
            //leave it alone
            break;
        default:
            ROS_WARN("location_code %d not recognized", location_code);
    }
    return nav_goal;
}

/**
 * Update the mining goal to the most efficient place on the mining.
 * fails gracefully and warns the user if there is an error.
 */
move_base_msgs::MoveBaseGoal NavigationGoalManager::get_updated_mining_goal(geometry_msgs::Pose msg)
{
    if (location_code == tfr_msgs::NavigationGoal::TO_MINING){
        nav_goal.target_pose.header.stamp =ros::Time::now();
        //TODO integrate reference frame when bin tf publisher is finished
        double v_position = msg.position.y; 
        int sign = (v_position > 0) ? 1 : -1;
        nav_goal.target_pose.pose.position.y = sign*std::min(
                constraints.get_mining_line_length()/2, std::abs(v_position));
    }
    else 
    {
        ROS_WARN("only can update mining goal, your goal: %d", location_code);
    }
    return nav_goal;
}


