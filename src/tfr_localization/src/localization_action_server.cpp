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
#include <tfr_msgs/LocalizationAction.h>
#include <tfr_msgs/WrappedImage.h>
#include <tfr_msgs/PoseSrv.h>
#include <tfr_utilities/tf_manipulator.h>
#include <geometry_msgs/Twist.h>

class Localizer
{
    public:
        Localizer(ros::NodeHandle &n, const double& velocity, const double&
                duration, const double& thresh) : 
            aruco{n, "aruco_action_server"},
            server{n, "localize", boost::bind(&Localizer::localize, this, _1) ,false},
            cmd_publisher{n.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
            turn_velocity{velocity},
            turn_duration{duration},
            threshold{thresh}

        {
            ROS_INFO("Localization Action Server: Connecting Aruco");
            aruco.waitForServer();
            ROS_INFO("Localization Action Server: Connected Aruco");

            ROS_INFO("Localization Action Server: Connecting Image Client");
            rear_cam_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/rear_cam/image_raw");
            front_cam_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/front_cam/image_raw");
            tfr_msgs::WrappedImage request{};
            ros::Duration busy_wait{0.1};
            while(!rear_cam_client.call(request)) 
                busy_wait.sleep();
            while(!front_cam_client.call(request))
                busy_wait.sleep();
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
        actionlib::SimpleActionServer<tfr_msgs::LocalizationAction> server;
        actionlib::SimpleActionClient<tfr_msgs::ArucoAction> aruco;
        ros::Publisher cmd_publisher;
        ros::ServiceClient rear_cam_client;
        ros::ServiceClient front_cam_client;
        TfManipulator tf_manipulator;
        const double& turn_velocity;
        const double& turn_duration;
        const double& threshold;

        void localize( const tfr_msgs::LocalizationGoalConstPtr &goal)
        {
            ROS_INFO("Localization Action Server: Localize Starting");
            //setup
            bool odometry = goal->set_odometry, success = true, set = false;

            geometry_msgs::Twist cmd;
            cmd.angular.z = turn_velocity;
            cmd_publisher.publish(cmd);

            ROS_INFO("Localization Action Server: odometry %d, target yaw %f",
                    odometry, goal->target_yaw);

            tfr_msgs::LocalizationResult output;
            //loop
            while (true)
            {

                ROS_INFO("Localization Action Server: iterating");
                if (server.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("Localization Action Server: preempt requested");
                    server.setPreempted(output);
                    success = false;
                    break;
                }


                tfr_msgs::ArucoResultConstPtr result = nullptr;
                tfr_msgs::WrappedImage image_wrapper{};
                if (rear_cam_client.call(image_wrapper))
                    result = sendAruco(image_wrapper);
                ROS_INFO("Localization Action Server: rearcam %d", result->number_found);

                if (result != nullptr && result->number_found == 0 && front_cam_client.call(image_wrapper))
                    result = sendAruco(image_wrapper);

                ROS_INFO("Localization Action Server: frontcam %d", result->number_found);
                if (result != nullptr && result->number_found > 0)
                {
                    //we found something
                    geometry_msgs::PoseStamped unprocessed_pose = result->relative_pose;

                    //transform from camera to footprint perspective
                    geometry_msgs::PoseStamped processed_pose;
                    if (!tf_manipulator.transform_pose(unprocessed_pose, processed_pose, "base_footprint"))
                    {
                        server.setAborted(output);
                        success = false;
                        break;
                    }

                    ROS_INFO("transformed");
                    

                    processed_pose.pose.position.z = 0;
                    processed_pose.header.stamp = ros::Time::now();

                    //send the message
                    tfr_msgs::PoseSrv::Request request{};
                    request.pose = processed_pose;
                    tfr_msgs::PoseSrv::Response response;
                    output.pose = processed_pose.pose;
                    if (odometry)
                    {

                        if(!ros::service::call("localize_bin", request, response))
                        {
                            ROS_INFO("localized");
                            tfr_msgs::LocalizationResult result;
                            server.setSucceeded(output);
                            odometry = false;
                            set = true;
                        }
                        else
                        {
                            ROS_INFO("Localization Action Server: retrying to localize movable point");
                        }
                    }


                    auto siny = +2.0 * 
                        (processed_pose.pose.orientation.w * processed_pose.pose.orientation.z + 
                         processed_pose.pose.orientation.x * processed_pose.pose.orientation.y);
                    auto cosy = +1.0 - 2.0 * 
                        (processed_pose.pose.orientation.y * processed_pose.pose.orientation.y + 
                         processed_pose.pose.orientation.z * processed_pose.pose.orientation.z );  
                    auto angle = atan2(siny, cosy);

                    auto difference = goal->target_yaw - angle;
                    if (std::abs(difference) < threshold)
                    {
                        if (!set)
                        {
                            server.setSucceeded(output);
                        }
                        break;
                    }
                }
                ROS_INFO("Localization Action Server: turning");

                geometry_msgs::Twist cmd;
                cmd.angular.z = turn_velocity;
                cmd_publisher.publish(cmd);
                ros::Duration(turn_duration).sleep();
                ROS_INFO("Localization Action Server: stopping");

                cmd.angular.z = 0;
                cmd_publisher.publish(cmd);
                ros::Duration(turn_duration).sleep();
            }

            if (success)
                server.setSucceeded(output);
 
            cmd.angular.z = 0;
            cmd_publisher.publish(cmd);
            //teardown
            ROS_INFO("Localization Action Server: Localize Finished");
        }

        tfr_msgs::ArucoResultConstPtr sendAruco(const tfr_msgs::WrappedImage& msg)
        {
            tfr_msgs::ArucoGoal goal;
            goal.image = msg.response.image;
            goal.camera_info = msg.response.camera_info;
            //send it to the server
            aruco.sendGoal(goal);
            aruco.waitForResult();
            return aruco.getResult();
        }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_action_server");
    ros::NodeHandle n{};
    double turn_velocity, turn_duration, threshold;
    ros::param::param<double>("~turn_velocity", turn_velocity, 0.0);
    ros::param::param<double>("~turn_duration", turn_duration, 0.0);
    ros::param::param<double>("~yaw_threshold", threshold, 0.0);
    if (turn_velocity == 0.0 || turn_duration == 0.0)
        ROS_WARN("Localization Action Server: Uninitialized Parameters");
    Localizer localizer(n, turn_velocity, turn_duration, threshold);
    ros::spin();
    return 0;
}
