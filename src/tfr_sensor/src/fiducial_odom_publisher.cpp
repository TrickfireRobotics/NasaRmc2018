/**
 * Fiducial odometry publisher, currently it's a quick and dirty test class to
 * get sensor fusion, and navigation up and running. 
 *
 * If the proof of concept is reliable in any way, we will refactor to a more
 * maintainable form. 
 *
 * Functionally it subscribes to a camera topic, feeds that to the fiducial
 * action server, and publishes the relevant Odometry information at a supplied
 * camera_link frame.
 *
 * It only publishes odometry if the fiducial action server is successful.
 *
 * parameters:
 *   ~camera_frame: The reference frame of the camera (string, default="camera_link")
 *   ~bin_frame: The reference frame of the bin (string, default="bin_link")
 *   ~odom_frame: The reference frame of odom  (string, default="odom")
 * subscribed topics:
 *   image (sensor_msgs/Image) - the camera topic
 * published topics:
 *   odom (geometry_msgs/Odometry)- the odometry topic 
 * */

//gee whiz this file has a lot of dependencies
#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_utilities/tf_manipulator.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tfr_msgs/ArucoAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


 class FiducialOdom
{
    public:
        FiducialOdom(ros::NodeHandle& n, 
                const std::string& c_frame, 
                const std::string& b_frame,
                const std::string& o_frame) :
            client{"aruco_action_server", true},
            manipulator{},
            camera_frame{c_frame},
            bin_frame{b_frame},
            odometry_frame{o_frame}
        {
            image_transport::ImageTransport it{n};
            subscriber = it.subscribeCamera("image",5, 
                    &FiducialOdom::process_odometry, this);
            publisher = n.advertise<nav_msgs::Odometry>("odom", 5 );
            ROS_INFO("Fiducial Odom Publisher Connecting to Server");
            client.waitForServer();
            ROS_INFO("Fiducial Odom Publisher Connected to Server");
        }
        ~FiducialOdom() = default;
        FiducialOdom(const FiducialOdom&) = delete;
        FiducialOdom& operator=(const FiducialOdom&) = delete;
        FiducialOdom(FiducialOdom&&) = delete;
        FiducialOdom& operator=(FiducialOdom&&) = delete;

    private:
        void process_odometry(const sensor_msgs::ImageConstPtr& image, const
                sensor_msgs::CameraInfoConstPtr& info)
        {
            tfr_msgs::ArucoGoal goal;
            goal.image = *image;
            goal.camera_info = *info;
            client.sendGoal(goal);
            if (!client.waitForResult())
            {
                //we need to transform our pose information from the bin to odom so let'
                //reuse our old utility from the bin publisher
                geometry_msgs::PoseStamped relative_pose = client.getResult()->relative_pose;

                
                geometry_msgs::PoseStamped transformed;
                /* TODO we will need to change this coordinate frame to map 
                 * if we ever want to integrate it with slam, but 
                 * it would be a pain, and we are in proof of concept rn.*/
                manipulator.transform_pose(relative_pose, transformed, odometry_frame);

                //time to package up our odometry odom
                nav_msgs::Odometry odom;
                odom.header.frame_id = odometry_frame;
                odom.header.stamp = ros::Time::now();

                //get our pose and fudge some covariances
                odom.pose.pose = transformed.pose;
                odom.pose.covariance = {  5e-3,   0,   0,   0,   0,   0,
                                             0,5e-3,   0,   0,   0,   0,
                                             0,   0,5e-3,   0,   0,   0,
                                             0,   0,   0,5e-3,   0,   0,
                                             0,   0,   0,   0,5e-3,   0,
                                             0,   0,   0,   0,   0,5e-3};

                //velocities are harder, we need to take a diffence and do some
                //conversions
                tf2::Transform t_0;
                tf2::fromMsg(last_pose.pose, t_0);
                tf2::Transform t_1;
                tf2::fromMsg(transformed.pose, t_1);
                
                /* take fast difference to get linear and angular delta inbetween
                 * timestamps
                 * https://answers.ros.org/question/12654/relative-pose-between-two-tftransforms/
                 */
                t_1.inverseTimes(t_0);

                auto linear_deltas = t_1.getOrigin();
                auto angular_deltas = t_1.getRotation();

                //convert from quaternion to rpy for odom compatibility
                tf2::Matrix3x3 converter{};
                converter.setRotation(angular_deltas);
                tf2::Vector3 rpy_deltas{};
                converter.getRPY(rpy_deltas[0], rpy_deltas[1], rpy_deltas[2]); 

                const tf2Scalar delta_t{
                    transformed.header.stamp.toSec() - last_pose.header.stamp.toSec()};

                odom.twist.twist.linear =  tf2::toMsg(linear_deltas/delta_t);
                odom.twist.twist.angular = tf2::toMsg(rpy_deltas/delta_t);
                
                odom.twist.covariance = {  5e-3,   0,   0,   0,   0,   0,
                                              0,5e-3,   0,   0,   0,   0,
                                              0,   0,5e-3,   0,   0,   0,
                                              0,   0,   0,5e-3,   0,   0,
                                              0,   0,   0,   0,5e-3,   0,
                                              0,   0,   0,   0,   0,5e-3};

                //fire it off! and cleanup
                publisher.publish(odom);
                last_pose = transformed;
            }
            else
            {
                ROS_WARN("Fiducial action server failed.");
            }
        }

        image_transport::CameraSubscriber subscriber;
        ros::Publisher publisher;
        actionlib::SimpleActionClient<tfr_msgs::ArucoAction> client;
        TfManipulator manipulator;

        geometry_msgs::PoseStamped last_pose{};


        const std::string& camera_frame;
        const std::string& bin_frame;
        const std::string& odometry_frame;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "fiducial_odom_publisher");
    ros::NodeHandle n{};

    std::string camera_frame, bin_frame, odometry_frame;
    ros::param::param<std::string>("~camera_frame", camera_frame, "camera_link");
    ros::param::param<std::string>("~bin_frame", bin_frame, "bin_link");
    ros::param::param<std::string>("~odometry_frame", odometry_frame, "odom");

    FiducialOdom fiducial_odom{n, camera_frame, bin_frame, odometry_frame};

    ros::spin();
    return 0;
}

