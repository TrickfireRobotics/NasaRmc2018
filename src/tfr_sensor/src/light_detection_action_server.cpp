#include <ros/ros.h>
#include <ros/console.h>
#include <light_detector.h>
#include <tfr_msgs/LightDetectAction.h>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>

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
                const std::string camera_name, int window_size,
                int threshold) : 
            n{node},
            server{n, name, false},
            detector{window_size, threshold},
            it{n}
        {
            server.registerGoalCallback(
                    boost::bind(&DetectionActionServer::setGoal,this));
            server.registerPreemptCallback(
                    boost::bind(&DetectionActionServer::preempt,this));
            image_subscriber = it.subscribe(camera_name, 5,
                    &DetectionActionServer::detect, this);
        }

        ~DetectionActionServer() = default;
        DetectionActionServer( const DetectionActionServer&) = delete;
        DetectionActionServer& operator=(const DetectionActionServer&) = delete;
        DetectionActionServer(DetectionActionServer&&) = delete;
        DetectionActionServer& operator=(DetectionActionServer&&)=delete;
        
    private:
        void setGoal()
        {
            ROS_INFO("DetectionActionServer started");
            server.acceptNewGoal();
        }
        void preempt()
        {
            server.setPreempted();
        }
        void detect(const sensor_msgs::ImageConstPtr& msg)
        {
            if (!server.isActive())
                return;
            detector.add_image(msg);
            if (detector.is_on())
                server.setSucceeded();
        }

        ros::NodeHandle &n;
        actionlib::SimpleActionServer<tfr_msgs::LightDetectAction> server;
        LightDetector detector;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_subscriber;
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "light_detection_action_server");
    ros::NodeHandle n;

    std::string image_topic;
    ros::param::param<std::string>("~image_topic", image_topic, "/sensors/aruco_cam");
    int window_size;
    ros::param::param<int>("~window_size", window_size, 5);
    int threshold;
    ros::param::param<int>("~threshold", threshold, 0.2);
    
    DetectionActionServer(n, ros::this_node::getName(), image_topic,
            window_size, threshold);

    ros::spin();
    return 0;
}
