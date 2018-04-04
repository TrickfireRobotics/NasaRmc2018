//test code never see again
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_e_stop");
    ros::NodeHandle n;

    std_srvs::SetBool request;
    request.request.data = true;

    while(!ros::service::call("toggle_motors", request))
        ROS_INFO("toggle");
    while(ros::ok())
    {
        ros::service::call("toggle_motors", request);
        ros::Duration(1).sleep();
    }
    return 0;
}
