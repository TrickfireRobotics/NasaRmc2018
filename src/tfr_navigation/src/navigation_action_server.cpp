#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tfr_msgs/NavigationAction.h>
#include <tfr_msgs/LocalizePoint.h>
#include <tfr_utilities/location_codes.h>
#include <boost/bind.hpp>
#include <cstdint>
class Navigator
{ 
    public:
        /* 
         * Immutable struct of geometry constraints for the goal selection
         * algorithm
         * */
        struct GeometryConstraints 
        {
            public:
                GeometryConstraints(double d, double f) :
                    safe_mining_distance{d}, finish_line{f}{};

                double get_safe_mining_distance() const
                {
                    return safe_mining_distance;
                }

                double get_finish_line() const
                {
                    return finish_line;
                }
            private:
                //The distance to travel from the bin
                double safe_mining_distance;
                //the distance from the bin the finish line is
                double finish_line;
        };

        Navigator(ros::NodeHandle& n,
                const GeometryConstraints &c,
                const double& height_adj,
                const std::string &bin_f):
            node{n}, 
            rate{2},
            height_adjustment{height_adj},
            constraints{c},
            server{n, "navigate", boost::bind(&Navigator::navigate, this, _1) ,false}, 
            nav_stack{"move_base", true},
            bin_frame{bin_f}
        {
            ROS_DEBUG("Navigation server constructed %f", ros::Time::now().toSec());
            

            //NOTE ros is not really big on runtime exceptions,  I'll post an annoying
            //warning at startup
            if (    constraints.get_safe_mining_distance() < 0 || 
                    constraints.get_finish_line() < 0)
            {
                ROS_WARN("Mining constraints should be positive %f", 
                        ros::Time::now().toSec());
                ROS_WARN("    safe_mining_distance: %f",
                        constraints.get_safe_mining_distance());
                ROS_WARN("    finish_line: %f",
                        constraints.get_finish_line());
            }

            ROS_INFO("Navigation server connecting to nav_stack");
            nav_stack.waitForServer();
            ROS_INFO("Navigation server connected to nav_stack");
            server.start();
            ROS_INFO("Navigation server awaiting connection");
        }

        ~Navigator(){}
        Navigator(const Navigator&) = delete;
        Navigator& operator=(const Navigator&) = delete;
        Navigator(Navigator&&) = delete;
        Navigator& operator=(Navigator&&) = delete;



    private:
        /*
         *  Goal: 
         *      -uint8_t code corresponding to where we want to navigate. Goal list is
         *      described in Navigation.action in the tfr_msgs package
         *  Feedback:
         *      -none
         * */
        void navigate(const tfr_msgs::NavigationGoalConstPtr &goal)
        {
            auto code = static_cast<tfr_utilities::LocationCode>(goal->location_code);
            ROS_INFO("Navigation server started");
            //start with initial goal
            move_base_msgs::MoveBaseGoal nav_goal{};
            initializeGoal(nav_goal, code);
            nav_stack.sendGoal(nav_goal);


            //test for completion
            while (!nav_stack.getState().isDone())
            {
                //Deal with preemption or error
                if (server.isPreemptRequested() || !ros::ok()) 
                {
                    ROS_INFO("%s: preempted", ros::this_node::getName().c_str());
                    nav_stack.cancelAllGoals();
                    server.setPreempted();
                    return;
                }
                else
                {
                    rate.sleep();
                }
                ROS_INFO("state %s", nav_stack.getState().toString().c_str());
            }

            if (nav_stack.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                if (code == tfr_utilities::LocationCode::MINING)
                {
                    //drop the location of the hole for the return trip
                    tfr_msgs::LocalizePoint::Request request;
                    request.pose.pose.position.x=
                        nav_goal.target_pose.pose.position.x + 0.5;
                    request.pose.pose.orientation.z=1;
                    tfr_msgs::LocalizePoint::Response response;
                    while (!ros::service::call("localize_hole", request, response) 
                            && !server.isPreemptRequested() && ros::ok() )
                    {
                        ROS_INFO("placing hole");
                        rate.sleep();
                    }
                }
                server.setSucceeded();
            }
            else 
            {
                nav_stack.cancelAllGoals();
                server.setAborted();
            }
            ROS_INFO("Navigation server finished");
        }        ros::NodeHandle& node;
        //NOTE delegate initialization of server to ctor
        actionlib::SimpleActionServer<tfr_msgs::NavigationAction> server;
        //NOTE delegate initialization of server to ctor
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_stack;


        //parameters
        std::string frame_id{};
        std::string bin_frame{};
        std::string action_name{};
        ros::Rate rate;
        const double& height_adjustment;
        
        //the constraints to the problem
        const GeometryConstraints &constraints;

        void initializeGoal( move_base_msgs::MoveBaseGoal& nav_goal, 
                const tfr_utilities::LocationCode& goal)
        {
            //set reference frame
            nav_goal.target_pose.header.frame_id = bin_frame;
            nav_goal.target_pose.header.stamp = ros::Time::now();
            //set translation goal
            switch(goal)
            {
                case(tfr_utilities::LocationCode::MINING):
                    nav_goal.target_pose.pose.position.x = 0.5;
           //TODO             constraints.get_safe_mining_distance();
                    nav_goal.target_pose.pose.position.z = height_adjustment;
                    break;
                case(tfr_utilities::LocationCode::DUMPING):
                    nav_goal.target_pose.pose.position.x = 0.5;
          //TODO              constraints.get_finish_line();
                    nav_goal.target_pose.pose.position.z = height_adjustment;
                    break;
                case(tfr_utilities::LocationCode::UNSET):
                    //leave it alone
                    break;
                default:
                    ROS_WARN("location_code %u not recognized",
                            static_cast<uint8_t>(goal));
            }
            //set relative rotation (none)
            nav_goal.target_pose.pose.orientation.x = 0;
            nav_goal.target_pose.pose.orientation.y = 0;
            nav_goal.target_pose.pose.orientation.z = 0;
            nav_goal.target_pose.pose.orientation.w = 1;
        }
};

/**
 * Main entry point for the navigation action server, spins up the server and
 * awaits callbacks.
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_action_server");
    ros::NodeHandle n;
    double safe_mining_distance, finish_line, height_adjustment;
    std::string bin_frame;

    ros::param::param<double>("~safe_mining_distance", safe_mining_distance, 5.1);
    ros::param::param<double>("~finish_line", finish_line, 0.84);
    ros::param::param<double>("~height_adjustment", height_adjustment, 0);
    ros::param::param<std::string>("~bin_frame", bin_frame, "bin_footprint");

    Navigator::GeometryConstraints 
        constraints(safe_mining_distance, finish_line);
    Navigator navigator(n, constraints, height_adjustment, bin_frame);
    ros::spin();
    return 0;
}
