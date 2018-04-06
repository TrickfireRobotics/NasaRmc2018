#include "ros/ros.h"

// aruco and ROS-openCV bindings
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <tfr_msgs/ArucoAction.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include "generatedMarker.h"

#include <iostream>
typedef actionlib::SimpleActionServer<tfr_msgs::ArucoAction> Server;

class TFR_Aruco {
    public:
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv::Ptr<cv::aruco::Board> board;
        cv::Ptr<cv::aruco::DetectorParameters> params;
        image_geometry::PinholeCameraModel cameraModel;

        TFR_Aruco() {
            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

            // set up board. This method is temporary until an official board is created. Works for now
            // represents the board that comes in the folder of this project
            std::vector<std::vector<cv::Point3f> > boardCorners;
            std::vector<int> boardIds;
            setBoardData(boardCorners, boardIds);

            board = cv::aruco::Board::create(std::move(boardCorners), dictionary, std::move(boardIds));

            // set up params
            params = cv::Ptr<cv::aruco::DetectorParameters>(new cv::aruco::DetectorParameters);
            params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
            //params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
            params->cornerRefinementWinSize = 10;
        }

        // This is the method that will be called when a client makes use
        // of this server. The provided goal is the "input".
        void execute(const tfr_msgs::ArucoGoalConstPtr& goal, Server* server)
        {
            if (server->isPreemptRequested() || !ros::ok())
            {
                server->setPreempted();
                return;
            }
            cameraModel.fromCameraInfo(goal->camera_info);


            // convert ROS message to opencv image
            // the image is stored at imageHolder->image
            cv_bridge::CvImagePtr imageHolder;
            try 
            {
                imageHolder = cv_bridge::toCvCopy(goal->image, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }


            // detect fiducial markers
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;

            cv::aruco::detectMarkers(imageHolder->image, dictionary, markerCorners, markerIds, params);

            // get individual marker poses
            cv::Mat cameraMatrix = cv::Mat(cameraModel.fullIntrinsicMatrix()).clone();
            cv::Mat distCoeffs = cameraModel.distortionCoeffs().clone();

            std::vector< cv::Vec3d > rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

            cv::Vec3d boardRotVec, boardTransVec;
            int markersDetected = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, boardRotVec, boardTransVec);

            tfr_msgs::ArucoResult result;
            result.number_found = markersDetected;
            if (result.number_found > 0)
            {
                result.relative_pose.header.stamp = ros::Time::now();
                result.relative_pose.header.frame_id = goal->image.header.frame_id;
                /*
                 *  also the coordinate axist for the aruco are in a different
                 *  coordinate system and are rotated here.
                 * */
                result.relative_pose.pose.position.x = boardTransVec[2];
                result.relative_pose.pose.position.y = boardTransVec[0] * -1; /*y-axis is inverted*/
                result.relative_pose.pose.position.z = 0;
                //let tf do the euler angle -> quaternion math
                tf2::Quaternion rotated{};
                //change rotated perspective RPY aruco output to ros coordinate system (2d)
                rotated.setRPY(0,0, PI + boardRotVec[1]);
                result.relative_pose.pose.orientation.x = rotated.x();
                result.relative_pose.pose.orientation.y = rotated.y();
                result.relative_pose.pose.orientation.z = rotated.z();
                result.relative_pose.pose.orientation.w = rotated.w();
            }
            server->setSucceeded(result);
        }
    private:
        static constexpr double PI = 3.1415;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_action_server");
    ros::NodeHandle n;
    TFR_Aruco aruco;
    Server server(n, "aruco_action_server", boost::bind(&TFR_Aruco::execute, aruco, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}

