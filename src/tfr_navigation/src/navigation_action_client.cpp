#include<ros/ros.h>
#include<navigation_client.h>
#include<tfr_msgs/NavigationAction.h>
#include<tfr_msgs/location_codes.h>

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
    //test preemption
    client.navigate(tfr_msgs::LocationCode::MINING);
    client.stop_all();
    
    //test happy path mining
    client.navigate(tfr_msgs::LocationCode::MINING);
    for (int i = 0; i < 16; i++)
    {
        ros::spinOnce();
        ros::Duration(0.25).sleep();
    }

    
    //test happy path dumping
    client.navigate(tfr_msgs::LocationCode::DUMPING);
    ros::spin();
    return 0;
}
