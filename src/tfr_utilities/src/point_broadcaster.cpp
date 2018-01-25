#include <point_broadcaster.h>

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

void PointBroadcaster::broadcast()
{
    transform.header.stamp = ros::Time::now();
    broadcaster.sendTransform(transform);
}

bool PointBroadcaster::localize_point(tfr_msgs::LocalizePoint::Request &request,
        tfr_msgs::LocalizePoint::Response &resonse)
{
    transform.transform.translation.x = request.pose.position.x;
    transform.transform.translation.y = request.pose.position.y;
    transform.transform.translation.z = request.pose.position.z;
    transform.transform.rotation.x = request.pose.orientation.x;
    transform.transform.rotation.y = request.pose.orientation.y;
    transform.transform.rotation.z = request.pose.orientation.z;
    transform.transform.rotation.w = request.pose.orientation.w;
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_broadcaster");
    ros::NodeHandle n;

    std::string point_frame{}, parent_frame{}, service_name{};
    double hz{};

    ros::param::param<std::string>("~parent_frame", parent_frame, "");
    ros::param::param<std::string>("~point_frame", point_frame, "");
    ros::param::param<std::string>("~service_name", service_name, "");
    ros::param::param<double>("~hz", hz, 5.0 );

    PointBroadcaster broadcaster{n, point_frame, parent_frame, service_name};

    ros::Rate rate(hz);
    while(ros::ok())
    {
        broadcaster.broadcast();
        ros::spinOnce();
        rate.sleep();
    }
}
