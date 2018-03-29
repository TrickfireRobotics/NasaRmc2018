//test code never see again
#include <ros/ros.h>
#include <tfr_msgs/EmptySrv.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_e_stop");
    ros::NodeHandle n;

        tfr_msgs::EmptySrv request;

    while(!ros::service::call("toggle_motors", request));
    return 0;
}
