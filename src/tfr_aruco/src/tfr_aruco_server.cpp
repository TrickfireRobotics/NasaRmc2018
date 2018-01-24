#include "ros/ros.h"

// aruco and ROS-openCV bindings
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include <tfr_msgs/ArucoAction.h>
#include <actionlib/server/simple_action_server.h>


typedef actionlib::SimpleActionServer<tfr_msgs::ArucoAction> Server;

// This is the method that will be called when a client makes use
// of this server. The provided goal is the "input".
void execute(const tfr_msgs::ArucoGoalConstPtr& goal, Server* server)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
        
    // set up board. This method is temporary until an official board is created. Works for now
    // represents the board that comes in the folder of this project
    std::vector<std::vector<cv::Point3f> > boardCorners;
    
    // push top-left corners of markers
    // This is only temporarily hardcoded
    // TODO: write a program that generates both the code to do this from a "aruco board file"
    // and an image that, when printed, will map directly to that code
    boardCorners.push_back(std::vector<cv::Point3f>(1, cv::Point3f(0, 0, 0)));
    boardCorners.push_back(std::vector<cv::Point3f>(1, cv::Point3f(6, 0, 0)));
    
    boardCorners.push_back(std::vector<cv::Point3f>(1, cv::Point3f(0, 6, 0)));
    boardCorners.push_back(std::vector<cv::Point3f>(1, cv::Point3f(6, 6, 0)));
    
    boardCorners.push_back(std::vector<cv::Point3f>(1, cv::Point3f(0, 12, 0)));
    boardCorners.push_back(std::vector<cv::Point3f>(1, cv::Point3f(6, 12, 0)));
    
    boardCorners.push_back(std::vector<cv::Point3f>(1, cv::Point3f(0, 18, 0)));
    boardCorners.push_back(std::vector<cv::Point3f>(1, cv::Point3f(6, 18, 0)));
    
    // add rest of marker from the top left points (all the same size)
    for(auto &marker : boardCorners) 
    {
        marker.push_back(marker[0] + cv::Point3f(3.2, 0,   0));
        marker.push_back(marker[0] + cv::Point3f(3.2, 3.2, 0));
        marker.push_back(marker[0] + cv::Point3f(0,   3.2, 0));
        
        // from cm to meters
        for(auto &p : marker) {
            p *= .01;
        }
    }
    std::vector<int> boardIds = {1, 2, 3, 4, 5, 6, 7, 8};
    
    cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(std::move(boardCorners), dictionary, std::move(boardIds));
    
    
    // set up params
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::Ptr<cv::aruco::DetectorParameters>(new cv::aruco::DetectorParameters);
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    //params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
    params->cornerRefinementWinSize = 10;


    image_geometry::PinholeCameraModel cameraModel;
    cameraModel.fromCameraInfo(goal->camera_info);




    // convert ROS message to opencv image
    cv_bridge::CvImagePtr cv_ptr;
    try 
    {
        cv_ptr = cv_bridge::toCvCopy(goal->image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // using cameraImage is the same as cv_ptr->image, but more convienent and clear 
    cv::Mat cameraImage = cv_ptr->image;


    // detect fiducial markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    
    cv::aruco::detectMarkers(cameraImage, dictionary, markerCorners, markerIds, params);


    // get individual marker poses
    cv::Mat cameraMatrix = cv::Mat(cameraModel.fullIntrinsicMatrix()).clone();
    cv::Mat distCoeffs = cameraModel.distortionCoeffs().clone();
    
    std::vector< cv::Vec3d > rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);



    
    cv::Vec3d boardRotVec, boardTransVec;
    int v = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, boardRotVec, boardTransVec);
    
    tfr_msgs::ArucoResult result;
    result.number_found = v; // if v = 0, then no marker was detected
    result.position = std::vector<float>(3, 0); // 3 members initialized to 0
    result.rotation = std::vector<float>(3, 0); // 3 members initialized to 0
    if(v != 0) 
    {
        result.position[0] = boardTransVec[0];
        result.position[1] = boardTransVec[1];
        result.position[2] = boardTransVec[2];

        result.rotation[0] = boardRotVec[0];
        result.rotation[1] = boardRotVec[1];
        result.rotation[2] = boardRotVec[2];
    }
    server->setSucceeded(result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tfr_aruco");
    ros::NodeHandle n;

    Server server(n, "tfr_aruco", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}

