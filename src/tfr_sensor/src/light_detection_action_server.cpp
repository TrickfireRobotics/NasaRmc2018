#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_msgs/EmptyAction.h>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


/*
 *  An action server that looks for the switching on of a light.  
 *  Action message is empty,it merely signals the server to start processing.
 *  
 *  The server will not examine anything until commanded, and will set it's
 *  status to succeeded, when it sees the light.
 * */
class DetectionActionServer
{
    public:
        DetectionActionServer(ros::NodeHandle &node, const std::string name,
                int window_size,
                double thresh) : 
            n{node},
            server{node, name, false},
            threshold{thresh},
            it{node}
        {
            server.registerGoalCallback(
                    boost::bind(&DetectionActionServer::setGoal,this));
            server.registerPreemptCallback(
                    boost::bind(&DetectionActionServer::preempt,this));
            image_subscriber = it.subscribe("image", 1,
                    &DetectionActionServer::detect, this);
            ROS_INFO("DetectionActionServer starting connection");
            server.start();
            ROS_INFO("DetectionActionServer connected");
        }

        ~DetectionActionServer() = default;
        DetectionActionServer( const DetectionActionServer&) = delete;
        DetectionActionServer& operator=(const DetectionActionServer&) = delete;
        DetectionActionServer(DetectionActionServer&&) = delete;
        DetectionActionServer& operator=(DetectionActionServer&&)=delete;
        
    private:

        struct ColorStats
        {
            double r_ave;
            double g_ave;
            double b_ave;
            bool initialized;
        };

        void setGoal()
        {
            ROS_INFO("DetectionActionServer accepted goal");
            server.acceptNewGoal();
        }

        void preempt()
        {
            ROS_INFO("DetectionActionServer preempted goal");
            server.setPreempted();
        }

        /*
         * Detects if the light has been turned on 
         * */
        void detect(const sensor_msgs::ImageConstPtr& msg)
        {
            if (!server.isActive() || !ros::ok())
                return;

            ColorStats stats{};
            //convert out of std ros image
            cv::Mat image;
            try
            {
                image = cv_bridge::toCvCopy(msg)->image;
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s",
                        e.what());
                return;
            }

            cv::Scalar intensities = cv::sum(image);

            //note we have to reverse out of native cv bgr ordering
            stats.r_ave = intensities[2]/(image.rows*image.cols);
            stats.g_ave = intensities[1]/(image.rows*image.cols);
            stats.b_ave = intensities[0]/(image.rows*image.cols);

            if (stats.b_ave  > threshold*(stats.r_ave+stats.g_ave)/2)
                server.setSucceeded();
        }


        ros::NodeHandle &n;
        double threshold;
        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_subscriber;

};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "light_detection_action_server");
    ros::NodeHandle n;

    int window_size;
    double threshold;
    ros::param::param<double>("~threshold", threshold, 0.0);
    
    DetectionActionServer server{n, "light_detection", 
            window_size, threshold};

    ros::spin();
    return 0;
}
