/* * Converts measured wheel velocities into an an odometry message for use in
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
 *   - ~rate: how quickly to publish hz. (double, default 10)
 * Subscribed topics:
 *   - /arduino :(tfr_msgs/ArduinoReading) The most current information coming
 *   in from the sensors.
 * Published topics: 
 *   - /drivebase_odom : (nav_msgs/Odometry) the location of the
 *   base_footprint tracked by tread motion.
 * Services:
 *  - /set_drivebase_odometry : (tfr_msgs/SetOdometry) resets the basis of
 *  odometry to a new position
 * */
#include <ros/ros.h>
#include <tfr_msgs/ArduinoAReading.h>
#include <tfr_msgs/ArduinoBReading.h>
#include <tfr_msgs/SetOdometry.h>
#include <tfr_msgs/PoseSrv.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>

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
            x{},
            y{},
            angle{},
            tf_broadcaster{}
        {
            arduino_a = n.subscribe("/sensors/arduino_a", 15, &DrivebaseOdometryPublisher::readArduinoA, this);
            arduino_b = n.subscribe("/sensors/arduino_b", 15, &DrivebaseOdometryPublisher::readArduinoB, this);
            odometry_publisher = n.advertise<nav_msgs::Odometry>("/drivebase_odom", 15);
            set_odometry = n.advertiseService("set_drivebase_odometry", &DrivebaseOdometryPublisher::setOdometry, this);
            reset_odometry = n.advertiseService("reset_drivebase_odometry", &DrivebaseOdometryPublisher::resetOdometry, this);
            angle.x = 0;
            angle.y = 0;
            angle.z = 0;
            angle.w = 1;
        }

        ~DrivebaseOdometryPublisher() = default;
        DrivebaseOdometryPublisher(const DrivebaseOdometryPublisher&) = delete;
        DrivebaseOdometryPublisher& operator=(const DrivebaseOdometryPublisher&) = delete;
        DrivebaseOdometryPublisher(DrivebaseOdometryPublisher&&) = delete;
        DrivebaseOdometryPublisher& operator=(DrivebaseOdometryPublisher&) = delete;

        /*
         * Main business logic for the node, takes in readings from the arduino,
         * and publishes them across the network.
         * */
        void processOdometry()
        {
            //Grab the neccessary data
            tfr_msgs::ArduinoAReading reading_a;
            tfr_msgs::ArduinoBReading reading_b;
            if (latest_arduino_a != nullptr)
                reading_a = *latest_arduino_a;

            if (latest_arduino_b != nullptr)
                reading_b = *latest_arduino_b;

            //first we process the data
            auto t_1 = ros::Time::now();
            double d_t = (t_1 - t_0).toSec();
            //if this is the first message we skip it to initialize time
            //properly
            if (!t_0.isValid())
            {
                t_0 = ros::Time::now();
                return;
            }

            //message gives us velocity in meters/second from each individual
            //tread
            double v_l = -reading_a.tread_left_vel;
            double v_r = reading_b.tread_right_vel;




            //basic differential kinematics to get combined velocities
            double v_ang = (v_r-v_l)/wheel_span;
            double v_lin = (v_r+v_l)/2;
            
            //break into xy components and increment
            double d_angle = v_ang * d_t;

            tf2::Quaternion q_0{angle.x, angle.y, angle.z, angle.w};
            tf2::Quaternion q_1{};
            q_1.setRPY(0, 0, d_angle);
            q_0 *= q_1;
            angle.x = q_0.getX();
            angle.y = q_0.getY();
            angle.z = q_0.getZ();
            angle.w = q_0.getW();
            // yaw (z-axis rotation)
            double siny = +2.0 * (q_0.getW() * q_0.getZ() + q_0.getX() * q_0.getY());
            double cosy = +1.0 - 2.0 * (q_0.getY() * q_0.getY() + q_0.getZ() * q_0.getZ());  
            auto yaw = atan2(siny, cosy);
            //auto yaw = q_1.getAngleShortestPath();
            double v_x = v_lin*cos(yaw);
            double v_y = v_lin*sin(yaw);


            double d_x = v_x * d_t;
            x += d_x;

            double d_y = v_y * d_t;
            y += d_y;

            t_0 = t_1;

            //let's package up the message
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = parent_frame;
            msg.child_frame_id = child_frame;

            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = y;
            msg.pose.pose.position.z = 0;
            msg.pose.pose.orientation = angle;
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
            msg.twist.covariance = { 5e-2,    0,    0,    0,    0,    0,
                0, 5e-2,    0,    0,    0,    0,
                0,    0, 5e-2,    0,    0,    0,
                0,    0,    0, 5e-2,    0,    0,
                0,    0,    0,    0, 5e-2,    0,
                0,    0,    0,    0,    0, 5e-2 };
            odometry_publisher.publish(msg);
        }


    private:
        ros::Subscriber arduino_a; //the encoder data sub
        ros::Subscriber arduino_b; //the encoder data sub
        tfr_msgs::ArduinoAReadingConstPtr latest_arduino_a;
        tfr_msgs::ArduinoBReadingConstPtr latest_arduino_b;
        ros::Publisher odometry_publisher; //the pub for our processed data
        ros::ServiceServer set_odometry;
        ros::ServiceServer reset_odometry;
        tf2_ros::TransformBroadcaster tf_broadcaster;
        const std::string& parent_frame; //the parent frame of the robot
        const std::string& child_frame; //the child frame of the robot
        const double& wheel_span;
        double x; //the x coordinate of the robot (meters)
        double y; //the y coordinate of the robot (meters)
        geometry_msgs::Quaternion angle; 
        const double MAX_XY_DELTA = 0.15;
        const double MAX_THETA_DELTA = 0.13;
        ros::Time t_0;

        //callback for publisher
        void readArduinoA(const tfr_msgs::ArduinoAReadingConstPtr &msg)
        {
            latest_arduino_a = msg;
        }

        //callback for publisher
        void readArduinoB(const tfr_msgs::ArduinoBReadingConstPtr &msg)
        {
            latest_arduino_b = msg;
        }

        /*
         * Set odometry from fiducial markers, provides smoothing
         * */
        bool setOdometry(tfr_msgs::SetOdometry::Request& request,
                tfr_msgs::SetOdometry::Response& response)
        {

            auto dx = request.pose.position.x - x;
            if (std::abs(dx) >= MAX_XY_DELTA)
                dx = (dx >= 0) ? MAX_XY_DELTA : -MAX_XY_DELTA;
            x += dx;

            auto dy = request.pose.position.y - y;
            if (std::abs(dy) > MAX_XY_DELTA)
                dy = (dy >= 0) ? MAX_XY_DELTA : -MAX_XY_DELTA;
            y += dy;
            
             //footprint_odom transform
            angle = request.pose.orientation;

            std_srvs::Empty::Request req;
            std_srvs::Empty::Response res;
            ros::service::call("/move_base/clear_costmaps", req, res);
            return true;
        }

        /*
         * Set odometry from fiducial markers, provides no smoothing
         * */
        bool resetOdometry(tfr_msgs::SetOdometry::Request& request,
                tfr_msgs::SetOdometry::Response& response)
        {
            ROS_INFO("Drivebase Odometry Publisher: resetting drivebase odometry");

            x = request.pose.position.x;
            y = request.pose.position.y;
            angle = request.pose.orientation;
            return true;
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drivebase_odometry_publisher");
    ros::NodeHandle n;
    std::string parent_frame, child_frame;
    double wheel_span, r;
    ros::param::param<std::string>("~parent_frame", parent_frame, "odom");
    ros::param::param<std::string>("~child_frame", child_frame, "base_footprint");
    ros::param::param<double>("~wheel_span", wheel_span, 0.645);
    ros::param::param<double>("~rate", r, 10.0);
    DrivebaseOdometryPublisher publisher{n, parent_frame, child_frame, wheel_span};
    ros::Rate rate(r);
    while(ros::ok())
    {
        publisher.processOdometry();
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}
