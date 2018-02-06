#include "ros/ros.h"
#include <stdlib.h> // needed for system("clear")

// ROS-openCV bindings
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include <tfr_msgs/ArucoAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<tfr_msgs::ArucoAction> Client;

// Called once when the goal completes
void finished(const actionlib::SimpleClientGoalState& state, const tfr_msgs::ArucoResultConstPtr& result)
{
    //system("clear"); // make input easy to see
    if(result->number_found > 0) {
        ROS_INFO("[position]");
        ROS_INFO("    x: %f meters", result->relative_pose.position.x);
        ROS_INFO("    y: %f meters", result->relative_pose.position.y);
        ROS_INFO("    z: %f meters", result->relative_pose.position.z);
        ROS_INFO("[orientation]");
        ROS_INFO("    x: %f radians", result->relative_pose.orientation.x);
        ROS_INFO("    y: %f radians", result->relative_pose.orientation.y);
        ROS_INFO("    z: %f radians", result->relative_pose.orientation.z);
        ROS_INFO("    w: %f radians", result->relative_pose.orientation.w);
    } else {
        ROS_INFO("no markers found!");
    }
}

class ExampleCameraHandler 
{
public:
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport;

    // used to retrieve image
    image_transport::Subscriber imageInputSubscriber;
    sensor_msgs::Image cameraImage;

    // used to get the current camera calibration from the camera info node
    ros::Subscriber calibrationInputSubscriber;
    sensor_msgs::CameraInfo cameraInfo;

    Client client;
    bool haveCameraInfo = false;
    ExampleCameraHandler() : imageTransport(nodeHandle), client("tfr_aruco", true) 
    {
        imageInputSubscriber = imageTransport.subscribe("/image_raw", 1,
            &ExampleCameraHandler::updateCameraImage, this);

        calibrationInputSubscriber = nodeHandle.subscribe("/camera_info", 1,
            &ExampleCameraHandler::updateCameraInfo, this);

        ROS_INFO("WAITING FOR SERVER");
        // Ensure connection was established with Server
        client.waitForServer();
        ROS_INFO("Connected");
    }

    void updateCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info) 
    {
        cameraInfo = *cam_info;
	haveCameraInfo = true;
    }

    void updateCameraImage(const sensor_msgs::ImageConstPtr& msg) 
    {
        cameraImage = *msg;

        // Members of ArucoGoal are defined in tfr_msgs/action/Action.action
        tfr_msgs::ArucoGoal goal;
        goal.image = cameraImage;
        goal.camera_info = cameraInfo;

        // Callback functions: Result, Start, Feedback
        client.sendGoal(goal, &finished, NULL, NULL);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tfr_aruco_example_client");

    ExampleCameraHandler cameraHandler;

    ros::spin();

    return 0;
}

