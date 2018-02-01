#ifndef DUMPING_ACITON_SERVER_H
#define DUMPING_ACTION_SERVER_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/ArucoIntegrateAction.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
class Dumper
{
    public:        
        struct DumpingConsraints
        {
            private:
                int min_lin_vel, max_lin_vel, min_ang_vel, max_ang_vel;
            public:
            DumpingConstraints(int min_lin, int max_lin, int min_ang, int max_ang):
                min_lin_vel(min_lin), max_lin_vel(max_lin),
                min_ang_vel(min_ang), max_ang_vel(max_ang) {}
            int get_min_lin_vel() {return min_lin_vel;}
            int get_max_lin_vel() {return max_lin_vel;}
            int get_min_ang_vel() {return min_ang_vel;}
            int get_max_ang_vel() {return max_ang_vel;}
        };

        Dumper(ros::NodeHandle &node, const std::string &camera_topic,
                const DumpingConstraints &constraints);
        ~Dumper() = default;
        Dumper(const Dumper&) = delete;
        Dumper& operator=(const Dumper&) = delete;
        Dumper(Dumper&&) = delete;
        Dumper& operator=(Dumper&&) = delete;



    private:
        void dump(const tfr_msgs::EmptyGoalConstPtr &goal);
        tfr_msgs::ArucoIntegrateResult get_aruco_estimate();
        void stop_moving();
        void update_control_msg();

        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        actionlib::SimpleActionClient<tfr_msgs::EmptyAction> detector;

        //TODO add the Dumping Controller here
        //TODO add aruco here
        
        ros::ServiceClient image_client;
        ros::Publisher velocity_publisher;

        const DumpingConstraints &constraints; 

        geometry_msgs::Twist cmd{};
};

#endif
