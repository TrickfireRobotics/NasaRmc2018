#include <tf_manipulator.h>
/**
 * Collection of utility methods for dealing with transforms in 3d space and
 * transposing them.
 * This needs to be a class to make the transform buffers work
 * */

/*
 * Starts up the buffer and let's it fill momentarily to help avoid startup
 * errors
 * */
TfManipulator::TfManipulator():buffer{}, listener{buffer}
{
    //sleep for 1/2 a second to fill our transform buffer
    ros::Duration(0.5).sleep();
}


/**
 *  Wrap a pose in the current time stamp and give it a reference frame
 * */
geometry_msgs::PoseStamped TfManipulator::wrap_pose(const geometry_msgs::Pose
        &pose, const std::string &frame) 
{
    geometry_msgs::PoseStamped out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = frame;
    out.pose = pose;
    return out;
}

/**
 *  Transform pose from current to provided reference frame.
 *
 *  Relies on the transform buffer which needs time to fill, so if this is
 *  called before either reference frame has published transforms for a few
 *  seconds, it will fail, just repeatedly call it.
 * */
bool TfManipulator::transform_pose(const geometry_msgs::PoseStamped &from_pose, 
        geometry_msgs::PoseStamped &out, const std::string &to_frame)
{
    geometry_msgs::TransformStamped transform;
    try{
        transform = buffer.lookupTransform(
                to_frame, 
                from_pose.header.frame_id,
                ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return false;
    }
    tf2::doTransform(from_pose, out, transform);
    return true;
}
/**
 *  easy interface for looking up a transform
 *
 * */
bool TfManipulator::get_transform(geometry_msgs::Transform &output, 
        const std::string &from_frame, const std::string &to_frame)
{
    geometry_msgs::TransformStamped transform;
    try{
        transform = buffer.lookupTransform(
                from_frame,
                to_frame, 
                ros::Time(0));
         output = transform.transform;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return false;
    }
    return true;
}
