/**
 *  Entry point and node of the navigation action server, controls the business
 *  logic of the navigation subsystem.
 *
 *  Publishes and subscribes to the standard topics for a navigation action
 *  server.
 *
 *  Relevant Messages: Navigation[Goal|Feedback|Result]
 * */
#include <actionlib/server/simple_action_server.h>
#include <tfr_msgs/NavigationAction.h>  
#include <ros/console.h>
#include <ros/ros.h>

/**
 *  The action server, it's public interface for it's action is as follows:
 *
 *  Goal: 
 *      -uint8_t code corresponding to where we want to navigate. Goal list is
 *      described in Navigation.action in the tfr_msgs package
 *  Feedback:
 *      -uint8_t code corresponding to our current status described in 
 *      Navigation.action in the tfr_msgs package
 *      -Pose describing the current position
 *      -Pose describing the final targeted position
 *  Response
 *      -uint8_t code corresponding to the finsal status described in 
 *      Navigation.action in the tfr_msgs package
 *      -Pose describing our final position
 * */
class Navigator
{ 

    public:
        /**
         *  constructs the sever and binds it to it's execution callback
         * */
        Navigator(std::string name) :  server{n, name,
            boost::bind(&Navigator::navigate, this, _1) ,false}, action_name{name}
        {
            ROS_INFO("Navigation server constructed");
            server.start();
            ROS_INFO("Navigation server awaiting connection");
        }

        void navigate(const tfr_msgs::NavigationGoalConstPtr &goal)
        {
            //do the action here
            ROS_INFO("Navigation server started");
            if (server.isPreemptRequested() || !ros::ok()) 
            {
                ROS_INFO("%s: preempted", action_name.c_str());
                server.setPreempted();
            }
            else
            {
                feedback.status = tfr_msgs::NavigationFeedback::OK;
                server.publishFeedback(feedback);
                tfr_msgs::NavigationResult result;
                result.status = tfr_msgs::NavigationResult::OK;
                server.setSucceeded(result);
                ROS_INFO("Navigation server finished");
            }
        }

    private:
        using Server = actionlib::SimpleActionServer<tfr_msgs::NavigationAction>; 
        ros::NodeHandle n; // the node for the server
        Server server;
        tfr_msgs::NavigationFeedback feedback;
        std::string action_name;

};

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "navigation_action_server");
    Navigator navigator(ros::this_node::getName());
    ros::spin();
    return 0;
}

