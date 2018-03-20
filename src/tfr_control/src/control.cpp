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
 *  /toggle_motors - uses the empty service, needs to be explicitly turned on to
 *  work
 */
#include <ros/ros.h>
#include <tfr_msgs/EmptySrv.h>
#include <urdf/model.h>
#include <sstream>
#include <controller_manager/controller_manager.h>
#include "robot_interface.h"
#include "bin_control_server.h"

// TODO adam delete this test code when ready
// Whether we're running on hardware or using fake values
const bool use_fake_values = true;
class StopManager
{
    public:
        StopManager(ros::NodeHandle &n):
            eStop{n.advertiseService("toggle_motors", &StopManager::toggle,this)},
            enabled{false} {}
        ~StopManager() = default;
        StopManager(const StopManager&) = delete;
        StopManager& operator=(const StopManager&) = delete;
        StopManager(StopManager&&) = delete;
        StopManager& operator=(StopManager&&) = delete;

        bool isEnabled() { return enabled; }

    private:
        bool toggle(tfr_msgs::EmptySrv::Request& request,
                tfr_msgs::EmptySrv::Response& response)
        {
            enabled = !enabled;
            return true;
        }
        ros::ServiceServer eStop;
        bool enabled;



};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    double rate;
    ros::param::param<double>("~rate", rate, 10.0);

    // Start a spinner for ros node in the background, seperate from this thread
    // that manages the control loop
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // If we're faking the inputs, we need to know the model constraints on
    // the arm: load them here.
    // If not, just use zeroes, the limits don't matter. TODO delete
    double lower_limits[tfr_control::RobotInterface::JOINT_COUNT] = {};
    double upper_limits[tfr_control::RobotInterface::JOINT_COUNT] = {};

    //TODO delete
    if (use_fake_values) 
    {
        // Get the model description 
        std::string desc;
        n.param<std::string>("robot_description", desc, "");
    
        if (desc.length() == 0) 
        {
            ROS_ERROR("robot_description is empty and controller_launcher is using fake values, quitting.");
            return -1;
        }

        urdf::Model model;
        if (!model.initString(desc)) 
        {
            ROS_ERROR("Couldn't load robot_description, quitting.");
            return -1;
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

    //the hardware layer
    tfr_control::RobotInterface robot_interface{n, use_fake_values, lower_limits, upper_limits};

    //the controller layer
    controller_manager::ControllerManager controller_interface(&robot_interface);

    StopManager stopManager{n};

    // Store the time of the last update to feed to ControllerManager
    ros::Time then = ros::Time::now();

    ros::Duration cycle(1/rate);
    while (ros::ok())
    {
        //update from hardware
        robot_interface.read();
        //update controllers
        controller_interface.update(ros::Time::now(), cycle);
        if (!stopManager.isEnabled())
            robot_interface.clearCommands();
        //update hardware from controllers
        robot_interface.write();
        cycle.sleep();
    }
    return 0;
}
