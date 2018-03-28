#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/ArucoAction.h>
#include <tfr_msgs/WrappedImage.h>
#include <tfr_msgs/CodeSrv.h>
#include <tfr_utilities/control_code.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
/*
 * The dumping action server, it backs up the rover into the navigational aid
 * slowly.
 *
 * Its first step is to make sure it can see the aruco board, it will abort the
 * mission if it can't. 
 *
 * It backs up at a set speed until it get's really close and loses sight of the
 * board. When it is blind, it drives straight back, goes slower. 
 *
 * It stops when the light detector get's triggered.
 *
 * It requires a service where it can get the most recent image on demand for
 * the camera of interest for backing up. 
 *
 * This is currently filled by the camera_topic_wrapper in sensors
 *
 * published topics:
 *   -/cmd_vel geometry_msgs/Twist the drivebase velocity
 *   -/bin_position_controller/command std_msgs/Float64 the position of the bin
 *
 * */
class Dumper
{
    public:        
        struct DumpingConstraints
        {
            private:
                double min_lin_vel, max_lin_vel, min_ang_vel, max_ang_vel;
            public:
                DumpingConstraints(double min_lin, double max_lin, 
                        double min_ang, double max_ang):
                    min_lin_vel(min_lin), max_lin_vel(max_lin),
                    min_ang_vel(min_ang), max_ang_vel(max_ang) {}
                double getMinLinVel() const {return min_lin_vel;}
                double getMaxLinVel() const {return max_lin_vel;}
                double getMinAngVel() const {return min_ang_vel;}
                double getMaxAngVel() const {return max_ang_vel;}
        };

        
        Dumper(ros::NodeHandle &node, const std::string &service_name,
                const DumpingConstraints &c) :
            server{node, "dump", boost::bind(&Dumper::dump, this, _1), false},
            image_client{node.serviceClient<tfr_msgs::WrappedImage>(service_name)},
            velocity_publisher{node.advertise<geometry_msgs::Twist>("cmd_vel", 10)},
            bin_publisher{node.advertise<std_msgs::Float64>("/bin_position_controller/command", 10)},
            detector{"light_detection"},
            aruco{"aruco_action_server",true},
            constraints{c}
        {
            ROS_INFO("dumping action server initializing");
            detector.waitForServer();
            aruco.waitForServer();
            server.start();
            ROS_INFO("dumping action server initialized");
        }

        ~Dumper() = default;
        Dumper(const Dumper&) = delete;
        Dumper& operator=(const Dumper&) = delete;
        Dumper(Dumper&&) = delete;
        Dumper& operator=(Dumper&&) = delete;

    private:
        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        actionlib::SimpleActionClient<tfr_msgs::EmptyAction> detector;
        actionlib::SimpleActionClient<tfr_msgs::ArucoAction> aruco;

        ros::ServiceClient image_client;
        ros::Publisher velocity_publisher;
        ros::Publisher bin_publisher;

        const DumpingConstraints &constraints; 


        /*
         * The business logic of the action server.
         */
        void dump(const tfr_msgs::EmptyGoalConstPtr &goal) 
        {  
            ROS_INFO("dumping action server started dumping procedure");
            //check to make sure we can see the board
            tfr_msgs::ArucoResult initial_estimate{};
            getArucoEstimate(initial_estimate);
            if (initial_estimate.number_found == 0)
            {
                server.setAborted();
                return;
            }

            //initialize
            tfr_msgs::EmptyGoal empty_goal{};
            detector.sendGoal(empty_goal);

            //loop until we see the light
            while (detector.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                //handle preemption
                if (server.isPreemptRequested() || !ros::ok())
                {
                    stopMoving();
                    server.setPreempted();
                    return;
                }

                //get the most recent aruco reading
                tfr_msgs::ArucoResult estimate{};
                getArucoEstimate(estimate);

                //send motor commands
                if (estimate.number_found == 0)
                    moveBlind();
                else
                {
                    geometry_msgs::Twist cmd{};
                    updateControlMsg(estimate, cmd);
                    velocity_publisher.publish(cmd);
                }
            }

            //we detected the light, stop moving immediately
            stopMoving();
            ROS_INFO("dumping action server detected light raising bin");
            std_msgs::Float64 bin_cmd;
            bin_cmd.data = tfr_utilities::JointAngles::BIN_MAX;
            tfr_msgs::CodeSrv query;
            ros::Rate rate(10);
            while (!server.isPreemptRequested() && ros::ok())
            {
                ros::service::call("bin_state", query);
                if (static_cast<tfr_utilities::BinCode>(query.response.code) == tfr_utilities::BinCode::RAISED)
                    break;
                bin_publisher.publish(bin_cmd);
                stopMoving();
                rate.sleep();
            }
            
            server.setSucceeded();
        }

        /*
         *  Back up and turn slightly to match the orientation of the aruco board
         * */
        void updateControlMsg(const tfr_msgs::ArucoResult &estimate,
                geometry_msgs::Twist &cmd)
        {
            //back up
            cmd.linear.x = -1 * constraints.getMaxLinVel();
            /*
             * Maintenence note:
             *
             * How do we decide if we are going left or right?
             * 
             * Well the estimate will return a pose describing displacement from our
             * rear camera, a +y displacement means the center of the board is to the
             * left(ccw), -y to the right (cw). 
             *
             * This conforms to rep 103
             * */
            int sign = (estimate.relative_pose.pose.position.y < 0) ? 1 : -1;
            cmd.angular.z = sign*constraints.getMaxAngVel();
        }

        /*
         * back up slowwwwly we can't see
         * */
        void moveBlind()
        {
            ROS_INFO("backing up blind");
            geometry_msgs::Twist cmd{};
            cmd.linear.x = -1*constraints.getMinLinVel();
            cmd.angular.z = 0;
            velocity_publisher.publish(cmd);
        }

        /*
         *  Stop Moving 
         * */
        void stopMoving()
        {
            geometry_msgs::Twist cmd{};
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            velocity_publisher.publish(cmd);
        }

        /*
         * Gets the most recent position estimate from the aruco service
         */
        void getArucoEstimate(tfr_msgs::ArucoResult &result)
        {
            tfr_msgs::WrappedImage image_request{};
            tfr_msgs::ArucoGoal goal{};
            while (!image_client.call(image_request));

            goal.image = image_request.response.image;
            goal.camera_info = image_request.response.camera_info;
            
            aruco.sendGoal(goal);
            aruco.waitForResult();

            result = *aruco.getResult();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dumping_action_server");
    ros::NodeHandle n;
    double min_lin_vel, max_lin_vel, min_ang_vel, max_ang_vel;
    ros::param::param<double>("~min_lin_vel",min_lin_vel, 0);
    ros::param::param<double>("~max_lin_vel",max_lin_vel, 0);
    ros::param::param<double>("~min_ang_vel",min_ang_vel, 0);
    ros::param::param<double>("~max_ang_vel",max_ang_vel, 0);
    std::string service_name;
    ros::param::param<std::string>("~image_service_name", service_name, "");
    Dumper::DumpingConstraints constraints(min_lin_vel, max_lin_vel,
            min_ang_vel, max_ang_vel);
    Dumper dumper(n, service_name, constraints);
    ros::spin();
    return 0;
}
