/**
 * controller_launcher.cpp
 * 
 * This is the controller layer for the ros control package.
 * It has the controller manager, which has the ability to register valid
 * controllers and manage their state. 
 * This layer maintains the state of the controller manager, and performs the
 * control loop for the control package.
 *
 * PARAMETERS:
 *  ~rate: in hz how fast we want to run the control loop (double, default:10)
 * SERVICES:
 *  /toggle_motors - uses the empty service, needs to be explicitly turned on to work
 *  /is_bin_extended - uses the query service to determine if the bin has been
 *  extended
 */
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <tfr_msgs/QuerySrv.h>
#include <urdf/model.h>
#include <sstream>
#include <controller_manager/controller_manager.h>
#include "robot_interface.h"
#include "bin_control_server.h"



// ADAM'S TEST CODE
// TODO adam delete this test code when ready
// Whether we're running on hardware or using fake values
const bool use_fake_values = true;
// If we're faking the inputs, we need to know the model constraints on
// the arm: load them here.
// If not, just use zeroes, the limits don't matter. TEST code
double lower_limits[tfr_control::RobotInterface::JOINT_COUNT] = {};
double upper_limits[tfr_control::RobotInterface::JOINT_COUNT] = {};

//test code
void initializeTestCode(ros::NodeHandle& n)
{
    // Get the model description 
    std::string desc;
    n.param<std::string>("robot_description", desc, "");

    if (desc.length() == 0) 
    {
        ROS_WARN("robot_description is empty and controller_launcher is using fake values, quitting.");
    }

    urdf::Model model;
    if (!model.initString(desc)) 
    {
        ROS_WARN("Couldn't load robot_description.");
    }

    ROS_INFO("Model loaded successfully, loading joint limits.");
    lower_limits[static_cast<int>(tfr_control::Joint::BIN)] 
        = model.getJoint("bin_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_control::Joint::BIN)] 
        = model.getJoint("bin_joint")->limits->upper;
    lower_limits[static_cast<int>(tfr_control::Joint::LOWER_ARM)] 
        = model.getJoint("lower_arm_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_control::Joint::LOWER_ARM)] 
        = model.getJoint("lower_arm_joint")->limits->upper;
    lower_limits[static_cast<int>(tfr_control::Joint::UPPER_ARM)] 
        = model.getJoint("upper_arm_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_control::Joint::UPPER_ARM)] 
        = model.getJoint("upper_arm_joint")->limits->upper;
    lower_limits[static_cast<int>(tfr_control::Joint::SCOOP)] 
        = model.getJoint("scoop_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_control::Joint::SCOOP)] 
        = model.getJoint("scoop_joint")->limits->upper;
}
//END TEST CODE


class Control
{
    public:

        Control(ros::NodeHandle &n, const double& rate):
            robot_interface{n, use_fake_values, lower_limits, upper_limits},
            controller_interface{&robot_interface},
            eStop{n.advertiseService("toggle_motors", &Control::toggle,this)},
            binExtended{n.advertiseService("is_bin_extended", &Control::isExtended,this)},
            cycle{1/rate},
            enabled{false}
        {}
        
        /*
         * performs one iteration of the control loop
         * */
        void execute()
        {
            //update from hardware
            robot_interface.read();
            //update controllers
            controller_interface.update(ros::Time::now(), cycle);
            if (!enabled)
                robot_interface.clearCommands();
            //update hardware from controllers
            robot_interface.write();
            cycle.sleep();
        }
    private:
        //the hardware layer
        tfr_control::RobotInterface robot_interface;

        //the controller layer
        controller_manager::ControllerManager controller_interface;

        //emergency stop
        ros::ServiceServer eStop;

        //tells if bin is extended
        ros::ServiceServer binExtended;

        //how fast to spin
        ros::Duration cycle;

        //if our motors are enabled
        bool enabled;

        /*
         * Toggles the emergency stop on and off
         * */
        bool toggle(std_srvs::SetBool::Request& request,
                std_srvs::SetBool::Response& response)
        {
            enabled = request.data;
            return true;
        }

        /*
         * Tells if the bin has been extended or not
         * */
        bool isExtended(tfr_msgs::QuerySrv::Request& request,
                tfr_msgs::QuerySrv::Response& response)
        {
            response.data = robot_interface.isBinExtended();
            return true;
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    double rate;
    ros::param::param<double>("~rate", rate, 30.0);

    //test code
    if (use_fake_values)
        initializeTestCode(n);

    // Start a spinner for ros node in the background, seperate from this thread
    // that manages the control loop
    ros::AsyncSpinner spinner(1);
    spinner.start();

    Control control{n, rate};

    while (ros::ok())
    {
        ros::spinOnce();
        control.execute();
    }
    return 0;
}
