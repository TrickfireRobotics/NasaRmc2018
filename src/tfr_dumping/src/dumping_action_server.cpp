#include <dumping_action_server.h>

Dumper::Dumper(ros::NodeHandle &node, const std::string &camera_topic) :
    n{node},
    server{n, "dump", boost::bind(&Dumper::dump, this, _1), false},
    detector{"light_detection"},
    //TODO add dumping controller here
    //TODO add aruco here
    it{n}
{
    ROS_INFO("dumping action server initializing");
    detector.waitForServer();

    image_subscriber = it.subscribe(camera_topic, 20, &Dumper::update_image, this);

    velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    server.start();
    ROS_INFO("dumping action server initialized");
}

void Dumper::dump(const tfr_msgs::EmptyGoalConstPtr &goal) 
{
    //TODO
}
void Dumper::update_image(const sensor_msgs::ImageConstPtr &msg) 
{
    //TODO
}


int main(int argc, char **argv)
{
    return 0;
}
