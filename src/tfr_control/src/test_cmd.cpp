//TEST CODE TO NEVER BE SEEN AGAIN
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <tfr_msgs/EmptySrv.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub =
      n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  tfr_msgs::EmptySrv request;

  while(!ros::service::call("toggle_motors", request));


  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    msg.linear.x=0.5;

//    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
