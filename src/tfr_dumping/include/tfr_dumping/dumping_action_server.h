#ifndef DUMPING_ACITON_SERVER_H
#define DUMPING_ACTION_SERVER_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tfr_msgs/EmptyAction.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
class Dumper
{
    public:        
        Dumper(ros::NodeHandle &node, const std::string &camera_topic);
        ~Dumper() = default;
        Dumper(const Dumper&) = delete;
        Dumper& operator=(const Dumper&) = delete;
        Dumper(Dumper&&) = delete;
        Dumper& operator=(Dumper&&) = delete;

    private:
        void dump(const tfr_msgs::EmptyGoalConstPtr &goal);
        void update_image(const sensor_msgs::ImageConstPtr &msg);

        //TODO add the Dumping Controller here
        //TODO add aruco here
        

        ros::NodeHandle &n;
        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        actionlib::SimpleActionClient<tfr_msgs::EmptyAction> detector;

        //TODO add the Dumping Controller here
        //TODO add aruco here
        image_transport::ImageTransport it;
        image_transport::Subscriber image_subscriber;
        ros::Publisher velocity_publisher;

        sensor_msgs::Image current_frame;
        geometry_msgs::Twist cmd{};
};

#endif
