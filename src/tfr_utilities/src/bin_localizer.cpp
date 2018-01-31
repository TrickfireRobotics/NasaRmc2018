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

    //create some arbitrary point
    //this will be replaced by output of the aruco action server
    geometry_msgs::Pose pose{};
    pose.position.x = 0.1;
    pose.position.y = 0.2;
    pose.position.z = 0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = -0.819;
    pose.orientation.w = 0.574;

    //process it to the correct reference frame
    auto stamped = manipulator.wrap_pose(pose, "rear_camera_link");
    geometry_msgs::PoseStamped transpose;
    while(!manipulator.transform_pose(stamped,transpose, "map"));

    //send the message
    tfr_msgs::LocalizePoint::Request request;
    request.pose = transpose.pose;
    tfr_msgs::LocalizePoint::Response response;
    bool out = ros::service::call("localize_bin", request, response);

    ROS_INFO("success: %s", out ? "true" : "false");

    return 0;
}
