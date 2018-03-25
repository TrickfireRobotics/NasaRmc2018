/*
 * Converts measured wheel velocities into an an odometry message for use in
 * sensor fusion.
 * 
 * Not currently configured to publish transforms, as that is the job of sensor
 * fusion right now.
 *
 * Parameters:
 *   - ~parent_frame: the frame our robot exists in (string, default: "odom")
 *   - ~child_frame: the frame of the robot (string, default: "base_footprint")
 *   - ~wheel_span: the separation of the treads of the robot. (double,
 *   default)
 * Subscribed topics:
 *   - /arduino :(tfr_msgs/ArduinoReading) The most current information coming
 *   in from the sensors.
 * Published topics: 
 *   - /drivebase_odometry : (nav_msgs/Odometry) the location of the
 *   base_footprint tracked by tread motion.
 * */
#include <ros/ros.h>
#include <tfr_msgs/ArduinoReading.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class DrivebaseOdometryPublisher
{
    public:
        DrivebaseOdometryPublisher(ros::NodeHandle &n, 
                const std::string& p_frame, 
                const std::string& c_frame,
                const double& wheel_sep) :
            parent_frame{p_frame},
            child_frame{c_frame},
            wheel_span{wheel_sep},
            t_0{ros::Time::now()},
            x{},
            y{},
            angle{}
        {
            arduino_subscriber = 
                n.subscribe("/arduino", 5, &DrivebaseOdometryPublisher::processOdometry, this);
            odometry_publisher = n.advertise<nav_msgs::Odometry>("/drivebase_odometry", 5);
        }

        ~DrivebaseOdometryPublisher() = default;
        DrivebaseOdometryPublisher(const DrivebaseOdometryPublisher&) = delete;
        DrivebaseOdometryPublisher& operator=(const DrivebaseOdometryPublisher&) = delete;
        DrivebaseOdometryPublisher(DrivebaseOdometryPublisher&&) = delete;
        DrivebaseOdometryPublisher& operator=(DrivebaseOdometryPublisher&) = delete;
    private:
        ros::Subscriber arduino_subscriber; //the encoder data sub
        ros::Publisher odometry_publisher; //the pub for our processed data
        const std::string& parent_frame; //the parent frame of the robot
        const std::string& child_frame; //the child frame of the robot
        const double& wheel_span;
        double x; //the x coordinate of the robot (meters)
        double y; //the y coordinate of the robot (meters)
        double angle; //angle of rotation around the z axis (radians)
        ros::Time t_0;

        /*
         * Main business logic for the node, takes in readings from the arduino,
         * and publishes them across the network.
         * */
        void processOdometry(const tfr_msgs::ArduinoReading &reading)
        {
            //first we process the data
            auto t_1 = ros::Time::now();
            double d_t = (t_1 - t_0).toSec();

            ROS_INFO("t_1, %f t_0 %f d_t %f", t_1.toSec(), t_0.toSec(), d_t);
            
            //message gives us velocity in meters/second from each individual
            //tread
            double v_l = reading.tread_left_vel;
            double v_r = reading.tread_right_vel;
            ROS_INFO("v_r %f v_l %f, span %f", v_r, v_l, wheel_span);

            //basic differential kinematics to get combined velocities
            double v_ang = (v_r-v_l)/wheel_span;
            double v_lin = (v_r+v_l)/2;
            ROS_INFO("v)ang %f v_lin %f", v_ang, v_lin);
            //break into xy components and increment
            double d_angle = v_ang * d_t;
            angle += d_angle;
            ROS_INFO("angle %f", angle);

            double v_x = v_lin*d_t*cos(angle);
            double v_y = v_lin*d_t*sin(angle);

            double d_x = v_x * d_t;
            ROS_INFO("x_0 %f d_x %f", x, d_x);
            x += d_x;
            ROS_INFO("x %f", x);

            double d_y = v_y * d_t;
            y += d_y;
            ROS_INFO("y %f", y);

            t_0 = t_1;

            //let's package up the message
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = parent_frame;
            msg.child_frame_id = child_frame;

            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = y;
            msg.pose.pose.position.z = 0;
            msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
            msg.pose.covariance = { 1e-1,    0,    0,    0,    0,    0,
                                       0, 1e-1,    0,    0,    0,    0,
                                       0,    0, 1e-1,    0,    0,    0,
                                       0,    0,    0, 1e-1,    0,    0,
                                       0,    0,    0,    0, 1e-1,    0,
                                       0,    0,    0,    0,    0, 1e-1 };

            msg.twist.twist.linear.x = v_x;
            msg.twist.twist.linear.y = v_y;
            msg.twist.twist.linear.z = 0;
            msg.twist.twist.angular.x = 0;
            msg.twist.twist.angular.y = 0;
            msg.twist.twist.angular.z = v_ang;
            msg.twist.covariance = { 1e-1,    0,    0,    0,    0,    0,
                                        0, 1e-1,    0,    0,    0,    0,
                                        0,    0, 1e-1,    0,    0,    0,
                                        0,    0,    0, 1e-1,    0,    0,
                                        0,    0,    0,    0, 1e-1,    0,
                                        0,    0,    0,    0,    0, 1e-1 };
            
            //finally send it across the network
            odometry_publisher.publish(msg);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drivebase_odometry_publisher");
    ros::NodeHandle n;
    std::string parent_frame, child_frame;
    double wheel_span;
    ros::param::param<std::string>("~parent_frame", parent_frame, "odom");
    ros::param::param<std::string>("~child_frame", child_frame, "odom");
    ros::param::param<double>("~wheel_span", wheel_span, 0.0);
    DrivebaseOdometryPublisher publisher{n, parent_frame, child_frame, wheel_span};
    ros::spin();
    return 0;
}
