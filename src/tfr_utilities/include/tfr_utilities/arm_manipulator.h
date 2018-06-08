#ifndef ARM_MANIPULATOR_H
#define ARM_MANIPULATOR_H
#include <ros/ros.h>
#include <tfr_msgs/ArmMoveAction.h>
#include <tfr_msgs/ArmMoveAction.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>

/**
 * Collection of utility methods for arm
 * */
class ArmManipulator
{
    public:
        ArmManipulator(ros::NodeHandle &n);
        ~ArmManipulator(){};
        ArmManipulator(const ArmManipulator&) = delete;
        ArmManipulator& operator=(const ArmManipulator&) = delete;
        ArmManipulator(ArmManipulator&&)=delete;
        ArmManipulator& operator=(ArmManipulator&&)=delete;
        void moveArm( const double& turnatble, const double& lower_arm, const double& upper_arm, const double& scoop);
    private:
        ros::Publisher trajectory_publisher;
        ros::Publisher scoop_trajectory_publisher;
 };

#endif
