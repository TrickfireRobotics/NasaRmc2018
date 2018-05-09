#include <arm_manipulator.h>

ArmManipulator::ArmManipulator(ros::NodeHandle &n):
            trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 5)},
            scoop_trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/arm_end_controller/command", 5)}
{ }

void  ArmManipulator::moveArm(const double& turntable, const double& lower_arm ,const double& upper_arm,  const double& scoop )
{
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.joint_names.resize(3);
    trajectory.points.resize(1);
    trajectory.points[0].positions.resize(3);
    trajectory.joint_names[0]="turntable_joint";
    trajectory.joint_names[1]="lower_arm_joint";
    trajectory.joint_names[2]="upper_arm_joint";
    trajectory.points[0].positions[0] = turntable;
    trajectory.points[0].positions[1] = lower_arm;
    trajectory.points[0].positions[2] = upper_arm;
    trajectory.points[0].time_from_start = ros::Duration(0.06);
    trajectory_publisher.publish(trajectory);

    trajectory_msgs::JointTrajectory scoop_trajectory;
    scoop_trajectory.header.stamp = ros::Time::now();
    scoop_trajectory.joint_names.resize(1);
    scoop_trajectory.points.resize(1);
    scoop_trajectory.points[0].positions.resize(1);
    scoop_trajectory.joint_names[0]="scoop_joint";
    scoop_trajectory.points[0].positions[0] = scoop;
    scoop_trajectory.points[0].time_from_start = ros::Duration(0.06);
    scoop_trajectory_publisher.publish(scoop_trajectory);
}
