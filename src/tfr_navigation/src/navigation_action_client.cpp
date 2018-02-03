#include<ros/ros.h>
#include<navigation_client.h>
#include<tfr_msgs/NavigationAction.h>
#include<tfr_utilities/location_codes.h>

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
    client.navigate(tfr_utilities::LocationCode::MINING);
    
    //test happy path dumping
    ros::spin();
    return 0;
}
