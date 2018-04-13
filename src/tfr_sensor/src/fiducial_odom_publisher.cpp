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
 *   ~footprint_frame: The reference frame of the robot_footprint(string,
 *   default="footprint")
 *   ~bin_frame: The reference frame of the bin (string, default="bin_footprint")
 *   ~odom_frame: The reference frame of odom  (string, default="odom")
 *   ~debug: print debugging info (bool, default: false)
 *   ~rate: how fast to process images
 * subscribed topics:
 *   image (sensor_msgs/Image) - the camera topic
 * published topics:
 *   odom (geometry_msgs/Odometry)- the odometry topic 
 * */
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tfr_msgs/ArucoAction.h>
#include <tfr_msgs/WrappedImage.h>
#include <tfr_utilities/tf_manipulator.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class FiducialOdom
{
    public:
        FiducialOdom(ros::NodeHandle& n, 
                const std::string& f_frame, 
                const std::string& b_frame,
                const std::string& o_frame,
                bool debugging) :
            aruco{"aruco_action_server", true},
            tf_manipulator{},
            footprint_frame{f_frame},
            bin_frame{b_frame},
            odometry_frame{o_frame},
            debug{debugging}
        {
            rear_cam_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/rear_cam/image_raw");
            kinect_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/kinect/image_raw");
            publisher = n.advertise<nav_msgs::Odometry>("odom", 10 );
            ROS_INFO("Fiducial Odom Publisher Connecting to Server");
            aruco.waitForServer();
            ROS_INFO("Fiducial Odom Publisher Connected to Server");
            //fill transform buffer
            ros::Duration(2).sleep();
            //connect to the image clients
            tfr_msgs::WrappedImage request{};
            while(!rear_cam_client.call(request)) ROS_INFO("rear");
            while(!kinect_client.call(request)) ROS_INFO("kinect");
            ROS_INFO("Fiducial Odom Publisher: Connected Image Clients");
        }

        ~FiducialOdom() = default;
        FiducialOdom(const FiducialOdom&) = delete;
        FiducialOdom& operator=(const FiducialOdom&) = delete;
        FiducialOdom(FiducialOdom&&) = delete;
        FiducialOdom& operator=(FiducialOdom&&) = delete;

        void processOdometry()
        {
            tfr_msgs::ArucoResultConstPtr result = nullptr;
            tfr_msgs::WrappedImage image_wrapper{};

            //grab an image
            if (rear_cam_client.call(image_wrapper))
                result = sendAruco(image_wrapper);

            if ((result == nullptr || result->number_found == 0) && kinect_client.call(image_wrapper))
                result = sendAruco(image_wrapper);

            

            if (result != nullptr && result->number_found !=0)
			{

                geometry_msgs::PoseStamped unprocessed_pose = result->relative_pose;
                if (unprocessed_pose.header.frame_id == "kinect_rgb_optical_frame")
                {
                    //note we have to reverse signs here
                    unprocessed_pose.header.frame_id = "kinect_link";
                }



				if (debug)
					ROS_INFO("unprocessed data %s %f %f %f %f %f %f %f",
							unprocessed_pose.header.frame_id.c_str(),
							unprocessed_pose.pose.position.x,
							unprocessed_pose.pose.position.y,
							unprocessed_pose.pose.position.z,
							unprocessed_pose.pose.orientation.x,
							unprocessed_pose.pose.orientation.y,
							unprocessed_pose.pose.orientation.z,
							unprocessed_pose.pose.orientation.w);

				//transform from camera to footprint perspective
				geometry_msgs::PoseStamped processed_pose;
				if (!tf_manipulator.transform_pose(unprocessed_pose,
							processed_pose, footprint_frame))
					return;

                processed_pose.pose.position.z = 0;
                if (debug)
                    ROS_INFO("processed data %s %f %f %f %f %f %f %f",
                            processed_pose.header.frame_id.c_str(),
                            processed_pose.pose.position.x,
                            processed_pose.pose.position.y,
                            processed_pose.pose.position.z,
                            processed_pose.pose.orientation.x,
                            processed_pose.pose.orientation.y,
                            processed_pose.pose.orientation.z,
                            processed_pose.pose.orientation.w);

                //we need to express that in terms of odom

                geometry_msgs::Transform relative_bin_transform{};

                //get odom bin transform
                if (!tf_manipulator.get_transform(relative_bin_transform,
                            odometry_frame, bin_frame))
                    return;

                     //get odom bin transform
                    if (!tf_manipulator.get_transform(relative_bin_transform,
                                odometry_frame, bin_frame))
                        return;

                    if (debug)
                        ROS_INFO("relative transform %f %f %f %f %f %f %f",
                                relative_bin_transform.translation.x,
                                relative_bin_transform.translation.y,
                                relative_bin_transform.translation.z,
                                relative_bin_transform.rotation.x,
                                relative_bin_transform.rotation.y,
                                relative_bin_transform.rotation.z,
                                relative_bin_transform.rotation.w);

                    //footprint_odom transform
                    tf2::Transform p_0{};
                    tf2::convert(processed_pose.pose, p_0);
                    tf2::Transform p_1{};
                    tf2::convert(relative_bin_transform, p_1);

               geometry_msgs::Transform relative_transform{};
                //get the difference between the two transforms deal with gimble
                //lock
                if (!isUntouched(p_1))
                {

                    if (isInverted(p_0) != isInverted(p_1))
                    {
                        ROS_INFO("processing");
                        auto difference = p_1.inverse().inverseTimes(p_0.inverse());
                        relative_transform = tf2::toMsg(difference);
                    }
                    else
                    {

                        ROS_INFO("processing normal");
                        auto difference = p_1.inverse().inverseTimes(p_0.inverse());
                        relative_transform = tf2::toMsg(difference);
                    }
                }
                else
                {
                    auto difference = p_1.inverse().inverseTimes(p_0.inverse());
                    relative_transform = tf2::toMsg(difference);
                }


                if (debug)
                {
                    auto origin = p_0.getOrigin();
                    auto rotation = p_0.getRotation();
                    ROS_INFO("rotated data p_0 %f %f %f %f %f %f %f",
                            origin.x(),
                            origin.y(),
                            origin.z(),
                            rotation.x(),
                            rotation.y(),
                            rotation.z(),
                            rotation.w());
                    origin = p_1.getOrigin();
                    rotation = p_1.getRotation();
                    ROS_INFO("rotated data p_1 %f %f %f %f %f %f %f",
                            origin.x(),
                            origin.y(),
                            origin.z(),
                            rotation.x(),
                            rotation.y(),
                            rotation.z(),
                            rotation.w());
                }


                //process the odometry
                geometry_msgs::Pose relative_pose{};
                relative_pose.position.x = relative_transform.translation.x;
                relative_pose.position.y = relative_transform.translation.y;
                relative_pose.position.z = 0;
                relative_pose.orientation = relative_transform.rotation;

                if (debug)
                    ROS_INFO("relative data %f %f %f %f %f %f %f",
                            relative_pose.position.x,
                            relative_pose.position.y,
                            relative_pose.position.z,
                            relative_pose.orientation.x,
                            relative_pose.orientation.y,
                            relative_pose.orientation.z,
                            relative_pose.orientation.w);

                // handle odometry data
                nav_msgs::Odometry odom;
                odom.header.frame_id = odometry_frame;
                odom.header.stamp = ros::Time::now();
                odom.child_frame_id = footprint_frame;

                //get our pose and fudge some covariances
                odom.pose.pose = relative_pose;
                odom.pose.covariance = {  1e-1,   0,   0,   0,   0,   0,
                    0,1e-1,   0,   0,   0,   0,
                    0,   0,1e-1,   0,   0,   0,
                    0,   0,   0,1e-1,   0,   0,
                    0,   0,   0,   0,1e-1,   0,
                    0,   0,   0,   0,   0,1e-1};
                //fire it off! and cleanup
                publisher.publish(odom);
            }
        }

    private:
        ros::Publisher publisher;
        ros::ServiceClient rear_cam_client;
        ros::ServiceClient kinect_client;
        actionlib::SimpleActionClient<tfr_msgs::ArucoAction> aruco;
        tf2_ros::TransformBroadcaster broadcaster;
        TfManipulator tf_manipulator;

        const std::string& footprint_frame;
        const std::string& bin_frame;
        const std::string& odometry_frame;
        bool debug;

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

        //Gives if the quaternion is > 90 yaw in either direction
        bool isInverted(const tf2::Transform& q)
        {
            return std::abs(q.getRotation().z()) > 0.707;
        }

        bool isUntouched(const tf2::Transform& t)
        {
            auto origin = t.getOrigin();
            auto rotation = t.getRotation();
            return  origin.x() == 0.0 &&
                    origin.y() == 0.0 &&
                    origin.z() == 0.0 &&
                    rotation.x() == 0.0 &&
                    rotation.y() == 0.0 &&
                    rotation.z() == 0.0 &&
                    rotation.w() == 1.0;
        }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fiducial_odom_publisher");
    ros::NodeHandle n{};

    std::string footprint_frame, bin_frame, odometry_frame;
    bool debug;
    double rate;
    ros::param::param<std::string>("~footprint_frame", footprint_frame, "footprint");
    ros::param::param<std::string>("~bin_frame", bin_frame, "bin_footprint");
    ros::param::param<std::string>("~odometry_frame", odometry_frame, "odom");
    ros::param::param<double>("~rate",rate, 5);
    ros::param::param<bool>("~debug",debug, false);

    FiducialOdom fiducial_odom{n, footprint_frame, bin_frame,
        odometry_frame, debug};

    ros::Rate r(rate);
    while(ros::ok())
    {
        fiducial_odom.processOdometry();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
