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
TfManipulator::TfManipulator():buffer(),listener(buffer)
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
bool TfManipulator::transform_pose(const geometry_msgs::PoseStamped &pose, 
        geometry_msgs::PoseStamped &out, const std::string &desired_frame)
{
    bool success = true;
    geometry_msgs::TransformStamped transform;
    try{
        transform = buffer.lookupTransform(
                desired_frame, 
                pose.header.frame_id,
                ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        success = false;
    }
    tf2::doTransform(pose, out, transform);
    return success;
}

