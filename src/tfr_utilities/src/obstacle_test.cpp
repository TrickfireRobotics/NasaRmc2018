/**
 * Test code never to be seen again to be used for testing the obstacle
 * broadcaster
 * */

#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_msgs/LocalizePoint.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bin_localizer");
    ros::NodeHandle n{};

    //send the message
    tfr_msgs::LocalizePoint::Request request;
    tfr_msgs::LocalizePoint::Response response;
    bool out = ros::service::call("localize_hole", request, response);
    return 0;
}
