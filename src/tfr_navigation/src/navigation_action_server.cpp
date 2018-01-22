#include <actionlib/server/simple_action_server.h>
#include <tfr_msgs/NavigationAction.h>  
#include <ros/console.h>
#include <ros/ros.h>

using Server = actionlib::SimpleActionServer<tfr_msgs::NavigationAction>;

class Navigator
{ 
    public:
        Navigator(std::string name): action_name{name}, n{}, s{n, action_name,
            boost::bind(&Navigator::navigate, this, _1), false}
        {
            s.start();
        }

        

        void navigate(const tfr_msgs::NavigationGoalConstPtr &goal)
        {
            //do the action here
            ROS_INFO("NavigationAction");
            s.publishFeedback(feedback);
            s.setSucceeded(result);
        }

    private:
        std::string action_name;
        ros::NodeHandle n;
        Server s;
        tfr_msgs::NavigationFeedback feedback;
        tfr_msgs::NavigationResult result;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_action_server");
    Navigator("navigate");
}
