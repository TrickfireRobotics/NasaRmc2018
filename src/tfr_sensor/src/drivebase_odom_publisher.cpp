/*
 * Converts measured wheel velocities into an an odometry message for use in
 * sensor fusion.
 *
 * Subscribed topics:
 *   - /arduino :(tfr_msgs/ArduinoReading) The most current information coming
 *   in from the sensors.
 * Published topics: 
 *   - /drivebase_odometry : (nav_msgs/Odometry) the location of the
 *   base_footprint tracked by tread motion.
 * */
#include <ros/ros.h>

class DrivebaseOdometryPublisher
{
    public:
        DrivebaseOdometryPublisher();
        ~DrivebaseOdometryPublisher() = default;
        DrivebaseOdometryPublisher(const DrivebaseOdometryPublisher&) = delete;
        DrivebaseOdometryPublisher& operator=(const DrivebaseOdometryPublisher&) = delete;
        DrivebaseOdometryPublisher(DrivebaseOdometryPublisher&&) = delete;
        DrivebaseOdometryPublisher& operator=(DrivebaseOdometryPublisher&) = delete;
    private:
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drivebase_odometry_publisher");
    ros::NodeHandle n;
    ros::spin();
    return 0;
}
