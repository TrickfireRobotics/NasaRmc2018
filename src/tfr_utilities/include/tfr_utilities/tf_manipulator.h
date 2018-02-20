#ifndef TF_MANIPULATOR_H
#define TF_MANIPULATOR_H
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/impl/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>

/**
 * Collection of utility methods for dealing with transforms in 3d space and
 * transposing them.
 * This needs to be a class to make the transform buffers work
 * */
class TfManipulator
{
    public:
        TfManipulator();
        ~TfManipulator(){};
        TfManipulator(const TfManipulator&) = delete;
        TfManipulator& operator=(const TfManipulator&) = delete;
        TfManipulator(TfManipulator&&)=delete;
        TfManipulator& operator=(TfManipulator&&)=delete;
        geometry_msgs::PoseStamped wrap_pose(const geometry_msgs::Pose &pose,
                const std::string &pose_frame);
        bool transform_pose(const geometry_msgs::PoseStamped &pose, 
                geometry_msgs::PoseStamped &out, const std::string &desired_frame);
        bool get_transform(geometry_msgs::Transform &transform, 
                const std::string &current_frame,const std::string &desired_frame);
    private:
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener;
};

#endif
