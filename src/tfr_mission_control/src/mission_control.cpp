#include "ros/ros.h"
#include "tfr_msgs/SystemStatus.h"

/**
 * Test callback
 */
void testCallback(const tfr_msgs::SystemStatus::ConstPtr& msg)
{
  ROS_INFO("Time: %f\nStatusCode: %d\nData: %5.2f", msg->SendTime.toSec(),msg->StatusCode, msg->Data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mission_control");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("tfr_system_status", 1000, testCallback);


  ros::spin();

  return 0;
}