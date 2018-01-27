#include<ros/ros.h>
#include<navigator.h>
/**
 * Main entry point for the navigation action server, spins up the server and
 * awaits callbacks.
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_action_server");
    ros::NodeHandle n;
    double safe_mining_distance, mining_line_length, finish_line;

    ros::param::param<double>("~safe_mining_distance", safe_mining_distance, 5.1);
    ros::param::param<double>("~mining_line_length", mining_line_length, 2.03);
    ros::param::param<double>("~finish_line", finish_line, 0.84);
    
    NavigationGoalManager::GeometryConstraints 
        constraints(safe_mining_distance, mining_line_length, finish_line);
    Navigator navigator(n, constraints, ros::this_node::getName());
    ros::spin();
    return 0;
}

