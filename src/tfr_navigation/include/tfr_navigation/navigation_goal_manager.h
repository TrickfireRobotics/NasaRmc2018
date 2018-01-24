#ifndef NAVIGATION_GOAL_MANAGER_H
#define NAVIGATION_GOAL_MANAGER_H
#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tfr_msgs/NavigationAction.h>
#include <cstdint>
#include <cmath>
class NavigationGoalManager
{
    public:
        //dist_landing + dist_obstacle + 1/2 robot_length
        constexpr static double SAFE_MINING_DISTANCE { 1.5 + 2.98 + 1.3/2 };
        //mining_zone_width - digging_radius
        constexpr static double MINING_LINE_LENGTH { 3.78 - 1.75 };
        //length of landing zone
        constexpr static double FINISH_LINE { 1.5 };

        NavigationGoalManager();
        NavigationGoalManager(uint8_t code);
        NavigationGoalManager(const NavigationGoalManager&) = delete;
        NavigationGoalManager& operator=(const NavigationGoalManager&) = delete;
        NavigationGoalManager(NavigationGoalManager&&) = delete;
        NavigationGoalManager& operator=(NavigationGoalManager&&) = delete;

        void initialize_goal();
        void update_mining_goal(geometry_msgs::Pose msg);

        //delegate initialization to ctor
        uint8_t location_code;
        move_base_msgs::MoveBaseGoal nav_goal{};
};
#endif
