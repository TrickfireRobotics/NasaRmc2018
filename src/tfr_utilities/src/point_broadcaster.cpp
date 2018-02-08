#include <point_broadcaster.h>

/**
 *  Utility node
 *
 *  Reports information about a moveable point on command.
 *
 *  Listens for updates about the point on a specified service. The service
 *  takes in a tfr_msgs/LocalizePoint.srv
 *
 *  It will report (0 0 0) (0 0 0 0) until it is given a proper location
 *
 *  parameters:
 *      parent_frame: the parent frame of this point (type = string default = "")
 *      point_frame: the name of the frame you want to broadcast (typed = string default = "")
 *      service_name: the name of the service you want to broadcast (typed =
 *      string default = "")
 *      hz: the frequency to pubish at. (type = doulbe default: 5.0)
 *
 * */

/*
 * Initializes the broadcaster and data structures
 * */
PointBroadcaster::PointBroadcaster(ros::NodeHandle& n, const std::string
        &point_frame, const std::string &parent_frame, const std::string
        &service) : node{n}, broadcaster_frame{point_frame},
    map_frame{parent_frame}, service_name{service}
{
    server = node.advertiseService(service_name,
            &PointBroadcaster::localize_point, this);
    transform.header.frame_id = map_frame;
    transform.child_frame_id = broadcaster_frame;
    transform.transform.rotation.w = 1;
}

/*
 * Broadcasts point across the transform network
 * */
void PointBroadcaster::broadcast()
{
    transform.header.stamp = ros::Time::now();
    broadcaster.sendTransform(transform);
}

/*
 * Gives the point a new origin
 * */
bool PointBroadcaster::localize_point(tfr_msgs::LocalizePoint::Request &request,
        tfr_msgs::LocalizePoint::Response &resonse)
{
    transform.transform.translation.x = request.pose.pose.position.x;
    transform.transform.translation.y = request.pose.pose.position.y;
    transform.transform.translation.z = request.pose.pose.position.z;
    transform.transform.rotation.x = request.pose.pose.orientation.x;
    transform.transform.rotation.y = request.pose.pose.orientation.y;
    transform.transform.rotation.z = request.pose.pose.orientation.z;
    transform.transform.rotation.w = request.pose.pose.orientation.w;
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_broadcaster");
    ros::NodeHandle n;

    //get parameters
    std::string point_frame{}, parent_frame{}, service_name{};
    double hz{};

    ros::param::param<std::string>("~parent_frame", parent_frame, "");
    ros::param::param<std::string>("~point_frame", point_frame, "");
    ros::param::param<std::string>("~service_name", service_name, "");
    ros::param::param<double>("~hz", hz, 5.0 );

    PointBroadcaster broadcaster{n, point_frame, parent_frame, service_name};

    //broadcast the point across the network
    ros::Rate rate(hz);
    while(ros::ok())
    {
        broadcaster.broadcast();
        ros::spinOnce();
        rate.sleep();
    }
}
