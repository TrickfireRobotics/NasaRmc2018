//TEST CODE TO NEVER BE SEEN AGAIN
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher bin_pub =
      n.advertise<std_msgs::Float64>("/bin_position_controller/command", 1000);
  ros::Publisher chatter_pub =
      n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Rate loop_rate(30);

  int count = 0;




  bool sign = false;
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    std_msgs::Float64 bin{};
    bin.data = 0.785;
    bin_pub.publish(bin);

    msg.linear.x= (sign)? -1:1;
    sign = !sign;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
