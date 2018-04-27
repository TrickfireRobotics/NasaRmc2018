#ifndef OBSTACLE_BROADCASTER_H 
#define OBSTACLE_BROADCASTER_H 

#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tfr_msgs/PoseSrv.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
/**
 *  Utility node
 *
 *  When initialized will drop a circular obstacle into the field with configurable diameter.
 *
 *  Listens for updates on a service. The service takes in a tfr_msgs/LocalizePoint.srv
 *  Once it gets called it will broadcast the obstacle at wherever the service
 *  told it to go
 *
 *  parameters:
 *      ~parent_frame: the parent frame of this point (type = string, default = "")
 *      ~point_frame: the name of the frame you want to broadcast (type = string, default = "")
 *      ~service_name: the name of the service you want to broadcast (type = string, default = "")
 *      ~hz: the frequency to pubish at. (type = double, default: 5.0)
 *
 *      ~diameter: the diameter of the obstacle in meters (type = double, default: 0.0)
 *      ~diameter: height adjustment  of the obstacle in meters (type = double, default: -.16)
 *  published topics:
 *      obstacle_cloud (PointCloud2) the cloud of points representing the obstacle
 * */
class ObstacleBroadcaster 
{
    public:
        ObstacleBroadcaster(
                ros::NodeHandle &n, 
                const std::string &point_frame, 
                const  std::string &parent_frame, 
                const std::string &service, 
                const double &diameter,
                const double &height);
        ~ObstacleBroadcaster(){};

        ObstacleBroadcaster(const ObstacleBroadcaster&) = delete;
        ObstacleBroadcaster& operator=( const ObstacleBroadcaster&) = delete;
        ObstacleBroadcaster(ObstacleBroadcaster&&) = delete;
        ObstacleBroadcaster& operator=(ObstacleBroadcaster&&) = delete;

        void broadcast();

    private:
        ros::NodeHandle &node;
        tf2_ros::TransformBroadcaster broadcaster{};
        ros::Publisher cloud_publisher{};
        geometry_msgs::TransformStamped transform{};
        ros::ServiceServer server;
        const std::string &broadcaster_frame;
        const std::string &map_frame;
        const std::string &service_name;
        const double &diameter;
        const double &height;
        bool point_set;

        bool localizePoint(tfr_msgs::PoseSrv::Request &request,
                tfr_msgs::PoseSrv::Response &response);
        void generateCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
};
#endif
