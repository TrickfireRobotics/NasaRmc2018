#include "ros/ros.h"
#include "tfr_msgs/SystemStatus.h"

/**
 * Test callback
 */
void testCallback(const tfr_msgs::SystemStatus::ConstPtr& msg)
{
  ROS_INFO("Time: %f\nStatusCode: %d\nData: %5.2f", msg->time_stamp.toSec(),
           msg->status_code, msg->data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mission_control_subscriber");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("system_status", 25, testCallback);


  ros::spin();

  return 0;
}
