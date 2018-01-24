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
#include<ros/ros.h>
#include<ros/console.h>
#include<actionlib/server/simple_action_server.h>
#include<navigation_goal_manager.h>
#include<tfr_msgs/NavigationAction.h>
#include<nav_msgs/Odometry.h>
#include<boost/bind.hpp>
#include<cstdint>
class Navigator
{ 
    public:
        Navigator(ros::NodeHandle& n, std::string name);
        ~Navigator(){}
        Navigator(const Navigator&) = delete;
        Navigator& operator=(const Navigator&) = delete;
        Navigator(Navigator&&) = delete;
        Navigator& operator=(Navigator&&) = delete;

    private:
        void navigate(const tfr_msgs::NavigationGoalConstPtr &goal);

        void update_position(const nav_msgs::OdometryConstPtr &msg);

       using Server = actionlib::SimpleActionServer<tfr_msgs::NavigationAction>; 

       ros::NodeHandle& node;

       //NOTE delegate initialization of server to ctor
       Server server;                      //handle to as
       //NOTE delegate initialization of subscriber to ctor
       ros::Subscriber odom_subscriber;    
       //NOTE delegate initialization of mgr to ctor
       NavigationGoalManager goal_manager;

       //needed msgs
       tfr_msgs::NavigationFeedback feedback{};
       tfr_msgs::NavigationResult result{};
       nav_msgs::Odometry current_position{};

       //parameters
       std::string frame_id{};
       std::string action_name{};
       std::string odometry_topic{};
       float rate{};
};
#endif
