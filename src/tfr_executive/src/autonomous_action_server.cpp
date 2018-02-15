#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/EmptySrv.h>
#include <tfr_msgs/DurationSrv.h>
#include <actionlib/server/simple_action_server.h>
class AutonomousExecutive
{
    public:
        AutonomousExecutive(ros::NodeHandle &n,double f):
            server{n, "autonomous_action_server", 
                boost::bind(&AutonomousExecutive::autonomousMission, this, _1),
                false},
            frequency{f}
        {
            server.start();
            ROS_INFO("Autonomous Action Server: online, %f",
                    ros::Time::now().toSec());
        }
        ~AutonomousExecutive() = default;
        AutonomousExecutive(const AutonomousExecutive&) = delete;
        AutonomousExecutive& operator=(const AutonomousExecutive&) = delete;
        AutonomousExecutive(AutonomousExecutive&&) = delete;
        AutonomousExecutive& operator=(AutonomousExecutive&&) = delete;
    private:
        void autonomousMission(const tfr_msgs::EmptyGoalConstPtr &goal)
        {
            ROS_INFO("Autonomous Action Server: mission started");

            ROS_INFO("Autonomous Action Server: starting clock %f",
                    ros::Time::now().toSec());
            tfr_msgs::EmptySrv start;
            ros::service::call("start_mission", start);
            ROS_INFO("Autonomous Action Server: clock started %f",
                    ros::Time::now().toSec());

            ROS_INFO("Autonomous Action Server: commencing localization");
            ROS_INFO("Autonomous Action Server: localization finished");

            ROS_INFO("Autonomous Action Server: commencing navigation");
            ROS_INFO("Autonomous Action Server: navigation finished");

            ROS_INFO("Autonomous Action Server: retrieving digging time");
            tfr_msgs::DurationSrv digging_time;
            ros::service::call("digging_time", digging_time);
            ROS_INFO("Autonomous Action Server: digging time retreived %f",
                    digging_time.response.duration.toSec());

            ROS_INFO("Autonomous Action Server: commencing digging");
            ROS_INFO("Autonomous Action Server: digging finished");

            ROS_INFO("Autonomous Action Server: commencing navigation");
            ROS_INFO("Autonomous Action Server: navigation finished");

            ROS_INFO("Autonomous Action Server: commencing dumping");
            ROS_INFO("Autonomous Action Server: dumping finished");

            tfr_msgs::EmptyResult result{};
            server.setSucceeded(result);
            ROS_INFO("Autonomous Action Server: mission finished");
        }


        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        ros::ServiceClient clock_service;
        ros::Duration frequency;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_action_server");
    ros::NodeHandle n{};
    double rate;
    ros::param::param<double>("~rate", rate, 10.0);
    AutonomousExecutive autonomousExecutive{n, 1.0/rate};
    ros::spin();
    return 0;
}
