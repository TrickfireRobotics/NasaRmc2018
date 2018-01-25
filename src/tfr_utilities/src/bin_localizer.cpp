/**
 *  This node is a placeholder for a set of responsibilites that collin needs to
 *  test for bin localization, it is not meant to go into the final system.
 *  It's logic and responsibilities will get transfered to an action server at
 *  the appropriate point in time.
 * */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_utility.h>
#include <tfr_msgs/LocalizePoint.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bin_localizer");
    ros::NodeHandle n{};

    //create some arbitrary point
    //this will be replaced by output of the aruco action server
    geometry_msgs::Pose pose{};
    pose.position.x = 0.12;
    pose.position.y = -0.20;
    pose.position.z = -0.04;
    pose.orientation.x = 0.962;
    pose.orientation.y = 0.023;
    pose.orientation.z = 0.084;
    pose.orientation.w = 0.258;

    //process it to the correct reference frame
    auto stamped = tf_utility::wrap_pose(pose, "rear_camera_link");
    auto transposed =  tf_utility::transpose_pose(stamped, "map");

    //send the message
    tfr_msgs::LocalizePoint::Request request;
    request.pose = transposed.pose;
    tfr_msgs::LocalizePoint::Response response;
    bool out = ros::service::call("localize_bin", request, response);

    ROS_INFO("success: %s", out ? "true" : "false");

    return 0;
}
