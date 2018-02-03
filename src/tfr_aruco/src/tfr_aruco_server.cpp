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
        result.position = std::vector<float>(3, 0); // 3 members initialized to 0
        result.rotation = std::vector<float>(3, 0); // 3 members initialized to 0
        if(markersDetected != 0) 
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tfr_aruco");
    ros::NodeHandle n;

    TFR_Aruco aruco;

    Server server(n, "tfr_aruco", boost::bind(&TFR_Aruco::execute, aruco, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}

