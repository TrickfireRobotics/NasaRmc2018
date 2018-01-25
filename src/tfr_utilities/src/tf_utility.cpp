#include <tf_utility.h>

geometry_msgs::PoseStamped tf_utility::wrap_pose(const geometry_msgs::Pose
        &pose, const std::string &frame)
{
    geometry_msgs::PoseStamped out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = frame;
    out.pose = pose;
    return out;
}

geometry_msgs::PoseStamped tf_utility::transpose_pose(const
        geometry_msgs::PoseStamped pose, std::string desired_frame)
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    geometry_msgs::TransformStamped transform;
    try{
        transform = buffer.lookupTransform(pose.header.frame_id,
                desired_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return pose;
    }
    geometry_msgs::PoseStamped out;
    tf2::doTransform(pose, out, transform);
    return out;
}
