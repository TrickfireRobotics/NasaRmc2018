#include<ros/ros.h>
#include<navigation_client.h>
/**
 *  Main entry point for test class of navigation action server, eventually
 *  these helper objects will be transferred to the exective action server
 *  TODO
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_action_client");
    ros::NodeHandle n;

    NavigationClient client("navigation_action_server");
    client.navigate_to_mining();

    ros::spin();
    return 0;
}
