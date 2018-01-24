#include <navigation_goal_manager.h>

//c++11 is weird with constexpre and we need to define these outside of the
//class in addition to the declaration and initialization step
constexpr double NavigationGoalManager::SAFE_MINING_DISTANCE;
constexpr double NavigationGoalManager::FINISH_LINE;
constexpr double NavigationGoalManager::MINING_LINE_LENGTH;

NavigationGoalManager::NavigationGoalManager(uint8_t code): location_code{code}
{
    initialize_goal();
}
NavigationGoalManager::NavigationGoalManager():
    NavigationGoalManager{tfr_msgs::NavigationGoal::UNSET} {}


/**
 *  Initializes the goal based on the location code, flashes a warning if it's
 *  not recognized.
 * */
void NavigationGoalManager::initialize_goal() {
    nav_goal.target_pose.header.stamp = ros::Time::now();
    //TODO integrate the bin reference frame here, upon completion of
    //the bin transform publisher
    switch(location_code)
    {
        case(tfr_msgs::NavigationGoal::TO_MINING):
            nav_goal.target_pose.pose.position.x = SAFE_MINING_DISTANCE;
            //TODO 
            break;
        case(tfr_msgs::NavigationGoal::TO_DUMPING):
            nav_goal.target_pose.pose.position.x = FINISH_LINE;
            break;
        case(tfr_msgs::NavigationGoal::UNSET):
            //leave it alone
            break;
        default:
            ROS_WARN("location_code %d not recognized", location_code);
    }
}

/**
 * Update the mining goal to the most efficient place on the mining.
 * fails gracefully and warns the user if there is an error.
 */
void NavigationGoalManager::update_mining_goal(geometry_msgs::Pose msg)
{
    if (location_code == tfr_msgs::NavigationGoal::TO_MINING){
        nav_goal.target_pose.header.stamp =ros::Time::now();
        //TODO integrate reference frame when bin tf publisher is finished
        double v_position = msg.position.y; 
        int sign = (v_position > 0) ? 1 : -1;
        nav_goal.target_pose.pose.position.y = sign*std::min(
                MINING_LINE_LENGTH/2, std::abs(v_position));
    }
    else 
    {
        ROS_WARN("only can update mining goal, your goal: %d", location_code);
    }
}

