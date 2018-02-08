/**
 *  This node is a placeholder for a set of responsibilites that collin needs to
 *  test for bin localization, it is not meant to go into the final system.
 *  It's logic and responsibilities will get transfered to an action server at
 *  the appropriate point in time.
 * */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_manipulator.h>
#include <tfr_msgs/LocalizePoint.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bin_localizer");
    ros::NodeHandle n{};
    TfManipulator manipulator{};

    //this is set up for the testing bay in the lab
    geometry_msgs::PoseStamped pose{};
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "bin_link";
    pose.pose.position.x = -1.7;
    pose.pose.position.y = 0.045;
    pose.pose.position.z = 0.4425;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    

    geometry_msgs::PoseStamped transpose{};
    //process it to the correct reference frame
    while(!manipulator.transform_pose(pose,transpose, "odom"))
        ros::Duration(0.5).sleep();

    //send the message
    tfr_msgs::LocalizePoint::Request request;
    request.pose = transpose;
    tfr_msgs::LocalizePoint::Response response;
    bool out = ros::service::call("localize_bin", request, response);

    ROS_INFO("success: %s", out ? "true" : "false");

    return 0;
}
