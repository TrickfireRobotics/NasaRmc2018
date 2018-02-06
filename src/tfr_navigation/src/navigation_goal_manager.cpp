include <navigation_goal_manager.h>

NavigationGoalManager::NavigationGoalManager(const std::string &frame, 
        const GeometryConstraints &c 
        ): reference_frame{frame}, constraints{c}
{
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
    ROS_INFO("frame: %s", reference_frame.c_str());
}

/**
 *  Initializes the goal based on the location code, flashes a warning if it's
 *  not recognized.
 * */
void NavigationGoalManager::initialize_goal( 
        move_base_msgs::MoveBaseGoal &nav_goal,
        const tfr_utilities::LocationCode &new_goal) {
    //set reference frame
    nav_goal.target_pose.header.frame_id = reference_frame;
    nav_goal.target_pose.header.stamp = ros::Time::now();
    //set translation goal
    switch(new_goal)
    {
        case(tfr_utilities::LocationCode::MINING):
            nav_goal.target_pose.pose.position.x =
                constraints.get_safe_mining_distance();
            break;
        case(tfr_utilities::LocationCode::DUMPING):
            nav_goal.target_pose.pose.position.x = constraints.get_finish_line();
            break;
        case(tfr_utilities::LocationCode::UNSET):
            //leave it alone
            break;
        default:
            ROS_WARN("location_code %u not recognized",
                    static_cast<uint8_t>(new_goal));
    }
    //set relative rotation (none)
    nav_goal.target_pose.pose.orientation.x = 0;
    nav_goal.target_pose.pose.orientation.y = 0;
    nav_goal.target_pose.pose.orientation.z = 0;
    nav_goal.target_pose.pose.orientation.w = 1;
}

/**
 * Update the mining goal to the most efficient place on the mining.
 * fails gracefully and warns the user if there is an error.
 */
void NavigationGoalManager::update_mining_goal(
        move_base_msgs::MoveBaseGoal &nav_goal,
        const geometry_msgs::Pose &msg)
{
    nav_goal.target_pose.header.stamp = ros::Time::now();
    double y_position = msg.position.y; 
    int sign = (y_position > 0) ? 1 : -1;
    nav_goal.target_pose.pose.position.y = sign*std::min(
            constraints.get_mining_line_length()/2, std::abs(y_position));
}
