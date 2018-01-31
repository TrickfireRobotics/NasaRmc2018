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
        /* 
         * Immutable struct of geometry constraints for the goal selection
         * algorithm
         * */
        struct GeometryConstraints 
        {
            public:
                GeometryConstraints(double d, double l, double f) :
                    safe_mining_distance{d}, mining_line_length{l},
                    finish_line{f}{};

                double get_safe_mining_distance() const
                {
                    return safe_mining_distance;
                }

                double get_mining_line_length() const
                {
                    return mining_line_length;
                }

                double get_finish_line() const
                {
                    return finish_line;
                }
            private:
                //The distance to travel from the bin
                double safe_mining_distance;
                //The length of the varying mining line
                double mining_line_length;
                //the distance from the bin the finish line is
                double finish_line;

        };


        NavigationGoalManager(const GeometryConstraints &constraints);
        NavigationGoalManager(const GeometryConstraints &constraints, uint8_t code);
        NavigationGoalManager(const NavigationGoalManager&) = delete;
        NavigationGoalManager& operator=(const NavigationGoalManager&) = delete;
        NavigationGoalManager(NavigationGoalManager&&) = delete;
        NavigationGoalManager& operator=(NavigationGoalManager&&) = delete;

        move_base_msgs::MoveBaseGoal initialize_goal();
        move_base_msgs::MoveBaseGoal get_updated_mining_goal(
                geometry_msgs::Pose msg);

        //delegate initialization to ctor
        uint8_t location_code;

        //the constraints to the problem
        const GeometryConstraints &constraints;
    private:
        //the navigation goal
        move_base_msgs::MoveBaseGoal nav_goal{};
};
#endif
