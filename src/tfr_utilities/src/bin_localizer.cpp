/**
 * This node is meant to localize the bin on system start using the rear camera
 * and an aruco sheet. 
 *
 * It's responsibilites will eventually get transferred to the localization
 * action server
 * */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_manipulator.h>
#include <tfr_msgs/LocalizePoint.h>
#include <actionlib/client/simple_action_client.h>
#include <tfr_msgs/WrappedImage.h>
#include <tfr_msgs/ArucoAction.h>

class PartialServer
{

    public:
        PartialServer(ros::NodeHandle &n):
            aruco{"aruco_action_server", true},
            tf_manipulator{}
        {
            image_client = n.serviceClient<tfr_msgs::WrappedImage>("/on_demand/rear_cam/image_raw");
            aruco.waitForServer();
        }

        bool localize(geometry_msgs::PoseStamped &output)
        {
            tfr_msgs::WrappedImage request{};
            //grab an image
            if(!image_client.call(request))
                return false;
            tfr_msgs::ArucoGoal goal;
            goal.image = request.response.image;
            goal.camera_info = request.response.camera_info;
            //send it to the server
            aruco.sendGoal(goal);
            if (aruco.waitForResult())
            {
                //make sure we can see
                auto result = aruco.getResult();
                if (result->number_found ==0)
                    return false;

                ROS_INFO("localization %s %f %f %f %f %f %f %f",
                        result->relative_pose.header.frame_id.c_str(),
                        result->relative_pose.pose.position.x,
                        result->relative_pose.pose.position.y,
                        result->relative_pose.pose.position.z,
                        result->relative_pose.pose.orientation.x,
                        result->relative_pose.pose.orientation.y,
                        result->relative_pose.pose.orientation.z,
                        result->relative_pose.pose.orientation.w);

                //transform relative to the base
                geometry_msgs::PoseStamped cam_pose =
                    aruco.getResult()->relative_pose;
                if (!tf_manipulator.transform_pose(cam_pose, output,
                            "base_footprint"))
                    return false;
                output.pose.position.y *=-1;
                output.pose.position.z *=-1;
                ROS_INFO("localization %s %f %f %f %f %f %f %f",
                        output.header.frame_id.c_str(),
                        output.pose.position.x,
                        output.pose.position.y,
                        output.pose.position.z,
                        output.pose.orientation.x,
                        output.pose.orientation.y,
                        output.pose.orientation.z,
                        output.pose.orientation.w);


                //fire off the localized bin
                output.header.frame_id = "odom";
                output.header.stamp = ros::Time::now();
                return true;
            }
        }
    private:
        actionlib::SimpleActionClient<tfr_msgs::ArucoAction> aruco;    
        ros::ServiceClient  image_client;
        TfManipulator tf_manipulator;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bin_localizer");
    ros::NodeHandle n{};

    PartialServer example{n};
    geometry_msgs::PoseStamped location;
    //do this until we can locaize
    while (!example.localize(location));

    //send the message
    tfr_msgs::LocalizePoint::Request request;
    request.pose = location;
    tfr_msgs::LocalizePoint::Response response;
    bool out = ros::service::call("localize_bin", request, response);
    return 0;
}
