#ifndef ARM_MANIPULATOR_H
#define ARM_MANIPULATOR_H
#include <ros/ros.h>
#include <tfr_msgs/ArmMoveAction.h>
#include <tfr_msgs/ArmMoveAction.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>

/**
 * Provides a simple method for moving the arm without MoveIt.
 * This is a regular ole' class, just instantiate it and call moveArm.
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
		
		/**
         * Moves the arm to the given position.
		 * 
		 * Notes: 
		 *  - Careful what parameters are passed in, the arm could collide with the robot.
		 * 
		 *  - The method is not blocking, so the caller needs to wait for the arm to move.
		 *    See digging_action_server.cpp for example.
         * */
        void moveArm( const double& turntable, const double& lower_arm, const double& upper_arm, const double& scoop);
    private:
        ros::Publisher trajectory_publisher;
        ros::Publisher scoop_trajectory_publisher;
 };

#endif
