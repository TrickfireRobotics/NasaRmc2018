#ifndef DUMPING_ACITON_SERVER_H
#define DUMPING_ACTION_SERVER_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/ArucoIntegrateAction.h>
#include <tfr_msgs/WrappedImage.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
/*
 * The dumping action server, it backs up the rover into the navigational aid
 * slowly.
 *
 * Its first step is to make sure it can see the aruco board, it will abort the
 * mission if it can't. 
 *
 * I backs up at a set speed until it get's really close and loses sight of the
 * board. When it is blind, it drives straight back, goes slower. 
 *
 * It stops when the light detector get's triggered.
 *
 * It requires a service where it can get the most recent image on demand for
 * the camera of interest for backing up. 
 * */
class Dumper
{
    public:        
        struct DumpingConstraints
        {
            private:
                double min_lin_vel, max_lin_vel, min_ang_vel, max_ang_vel;
            public:
                DumpingConstraints(double min_lin, double max_lin, 
                        double min_ang, double max_ang):
                    min_lin_vel(min_lin), max_lin_vel(max_lin),
                    min_ang_vel(min_ang), max_ang_vel(max_ang) {}
                double get_min_lin_vel() const {return min_lin_vel;}
                double get_max_lin_vel() const {return max_lin_vel;}
                double get_min_ang_vel() const {return min_ang_vel;}
                double get_max_ang_vel() const {return max_ang_vel;}
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
        void move_blind();
        void stop_moving();
        void update_control_msg(const tfr_msgs::ArucoIntegrateResult &estimate);

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
