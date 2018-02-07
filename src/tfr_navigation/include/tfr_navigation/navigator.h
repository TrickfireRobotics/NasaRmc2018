/**
 *  Controls the business logic of the navigation subsystem. 
 *  Mainly it accepts commands to go to specific locations, and manages the 
 *  goal of the navigation stack at a set interval to get it there.
 *
 *  Publishes and subscribes to the standard topics for a navigation action
 *  server.
 *
 *  Relevant Messages: Navigation[Goal|Feedback|Result]
 * */
#ifndef NAVIGATOR_H
#define NAVIGATOR_H
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <navigation_goal_manager.h>
#include <tfr_msgs/NavigationAction.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <cstdint>
class Navigator
{ 
    public:
        Navigator(ros::NodeHandle& n,
                const NavigationGoalManager::GeometryConstraints &constraints,
                const std::string &name, const std::string &bin_frame);
        ~Navigator(){}
        Navigator(const Navigator&) = delete;
        Navigator& operator=(const Navigator&) = delete;
        Navigator(Navigator&&) = delete;
        Navigator& operator=(Navigator&&) = delete;

    private:
        void navigate(const tfr_msgs::NavigationGoalConstPtr &goal);

        void update_position(const nav_msgs::OdometryConstPtr &msg);
        
        void update_feedback(tfr_msgs::NavigationFeedback &feedback,
                const move_base_msgs::MoveBaseGoal &nav_goal);
        void update_result(tfr_msgs::NavigationResult &result,
                const move_base_msgs::MoveBaseGoal &nav_goal);

        ros::NodeHandle& node;
        //NOTE delegate initialization of mgr to ctor
        NavigationGoalManager goal_manager;
        //NOTE delegate initialization of server to ctor
        actionlib::SimpleActionServer<tfr_msgs::NavigationAction> server;
        //NOTE delegate initialization of server to ctor
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_stack;
        //NOTE delegate initialization of subscriber to ctor
        ros::Subscriber odom_subscriber;    

        //needed msgs
        nav_msgs::OdometryConstPtr current_position{};

        //parameters
        std::string frame_id{};
        std::string action_name{};
        std::string odometry_topic{};
        float rate{};
};
#endif
