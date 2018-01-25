#ifndef TF_UTILITY_H
#define TF_UTILITY_H
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/impl/convert.h>
#include <ros/ros.h>

/**
 * Collection of utility methods for dealing with transforms in 3d space and
 * transposing them.
 * */
namespace tf_utility
{
    geometry_msgs::PoseStamped wrap_pose(const geometry_msgs::Pose &pose,
            const std::string &pose_frame);


    geometry_msgs::PoseStamped transpose_pose(const geometry_msgs::PoseStamped
            pose, std::string desired_frame);

}

#endif
