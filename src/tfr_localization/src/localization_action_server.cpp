/*
 * The action server in charge of localizing the robot.
 *
 * Takes in the empty action request, and provides no feedback.
 * Turns until it sees the aruco markers, exits succesfully once it does.
 *
 * Needs access to the image wrapper topic wrapper to fetch images, 
 * name is specified as a parameter.
 *
 * parameters:
 *  - ~turn_speed: how fast to turn [rad/s] (double, default: 0.0)
 *  - ~turn_duration: how long to turn [s] (double, default: 0.0)
 *
 * published topics:
 *  - /cmd_vel publishes to the drivebase (geometry_msgs/Twist)
 * */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tfr_msgs/ArucoAction.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/WrappedImage.h>
#include <tfr_msgs/LocalizePoint.h>
#include <tfr_utilities/tf_manipulator.h>
#include <geometry_msgs/Twist.h>

class Localizer
{
    public:
        Localizer(ros::NodeHandle &n, const double& velocity, const double& duration) : 
            aruco{n, "aruco_action_server"},
            server{n, "localize", boost::bind(&Localizer::localize, this, _1) ,false},
            cmd_publisher{n.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
            turn_velocity{velocity},
            turn_duration{duration}

        {
            ROS_INFO("Localization Action Server: Connecting Aruco");
            aruco.waitForServer();
            ROS_INFO("Localization Action Server: Connected Aruco");

            ROS_INFO("Localization Action Server: Connecting Image Client");
            rear_cam_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/rear_cam/image_raw");
            kinect_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/kinect/image_raw");
            tfr_msgs::WrappedImage request{};
            while(!rear_cam_client.call(request));
            while(!kinect_client.call(request));
            ROS_INFO("Localization Action Server: Connected Image Clients");
            ROS_INFO("Localization Action Server: Starting");
            server.start();
            ROS_INFO("Localization Action Server: Started");
        };
        ~Localizer() = default;
        Localizer(const Localizer&) = delete;
        Localizer& operator=(const Localizer&) = delete;
        Localizer(Localizer&&) = delete;
        Localizer& operator=(Localizer&&) = delete;
    private:
        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        actionlib::SimpleActionClient<tfr_msgs::ArucoAction> aruco;
        ros::Publisher cmd_publisher;
        ros::ServiceClient rear_cam_client;
        ros::ServiceClient kinect_client;
        TfManipulator tf_manipulator;
        const double& turn_velocity;
        const double& turn_duration;

        void localize( const tfr_msgs::EmptyGoalConstPtr &goal)
        {
            ROS_INFO("Localization Action Server: Localize Starting");
            //setup

            //loop
            while (true)
            {
                if (server.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("Localization Action Server: preempt requested");
                    server.setPreempted();
                    break;
                }

                tfr_msgs::WrappedImage rear_cam{}, kinect{};
                //grab an image
                rear_cam_client.call(rear_cam);
                kinect_client.call(kinect);

                tfr_msgs::ArucoResultConstPtr result = nullptr;

                tfr_msgs::ArucoGoal goal;
                goal.image = rear_cam.response.image;
                goal.camera_info = rear_cam.response.camera_info;
                //send it to the server
                aruco.sendGoal(goal);
                aruco.waitForResult();
                result = aruco.getResult();

                if (result != nullptr && result->number_found == 0)
                {
                    goal.image = kinect.response.image;
                    goal.camera_info = kinect.response.camera_info;
                    aruco.sendGoal(goal);
                    aruco.waitForResult();
                    result = aruco.getResult();
                }

                if (result != nullptr && result->number_found !=0)
                {
                    ROS_INFO("found something");
                    //We found something, transform relative to the base
                    geometry_msgs::PoseStamped bin_pose{};
                    if (!tf_manipulator.transform_pose(
                                result->relative_pose, 
                                bin_pose, 
                                "base_footprint"))
                    {
                        ROS_WARN("Localization Action Server: Transformation failed");
                        continue;
                    }
                    bin_pose.pose.position.y *=-1;
                    bin_pose.pose.position.z *=-1;
                    bin_pose.header.frame_id = "odom";
                    bin_pose.header.stamp = ros::Time::now();

                    //send the message
                    tfr_msgs::LocalizePoint::Request request;
                    request.pose = bin_pose;
                    tfr_msgs::LocalizePoint::Response response;
                    if(ros::service::call("localize_bin", request, response))
                    {
                        ROS_INFO("Localization Action Server: Success %f, %f, %f, %f, %f, %f, %f",
                                bin_pose.pose.position.x,
                                bin_pose.pose.position.y,
                                bin_pose.pose.position.z,
                                bin_pose.pose.orientation.x,
                                bin_pose.pose.orientation.y,
                                bin_pose.pose.orientation.z,
                                bin_pose.pose.orientation.w);
                        server.setSucceeded();
                        break;
                    }
                    else
                        ROS_INFO("Localization Action Server: retrying to localize movable point");
                }
                else
                {
                    ROS_INFO("Localization Action Server: No markers detected, turning");
                    geometry_msgs::Twist cmd;
                    cmd.angular.z = turn_velocity;
                    cmd_publisher.publish(cmd);
                    ros::Duration(turn_duration).sleep();
                    cmd.angular.z = 0;
                    cmd_publisher.publish(cmd);
                    continue;
                }

            }
            //teardown
            ROS_INFO("Localization Action Server: Localize Finished");
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_action_server");
    ros::NodeHandle n{};
    double turn_velocity, turn_duration;
    ros::param::param<double>("~turn_velocity", turn_velocity, 0.0);
    ros::param::param<double>("~turn_duration", turn_duration, 0.0);
    if (turn_velocity == 0.0 || turn_duration == 0.0)
        ROS_WARN("Localization Action Server: Uninitialized Parameters");
    Localizer localizer(n, turn_velocity, turn_duration);
    ros::spin();
    return 0;
}
