/****************************************************************************************
 * This is the primary node for the control package, it registers the controller instance
 * with the controller manager and handles calling updates on the hardware.
 ***************************************************************************************/
#include "control_helper.h"

#include <ros/ros.h>
#include <sstream>
#include <controller_manager/controller_manager.h>
#include "controller.h"

using namespace tfr_control;

// The controller that we'll be registering with the controller_manager
Controller controller;

int main(int argc, char **argv)
{
    // "controller_launcher is the name of this node at run-time"
    ros::init(argc, argv, "controller_launcher");
    ros::NodeHandle n;

    // Set up how often this node will loop (10 hz)
    ros::Rate loop_rate(10);

    // Register our controller with the controller_manager
    controller_manager::ControllerManager cm(&controller);

    // Start a spinner with one thread, we don't need any more than that
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Store the time of the last update to feed to ControllerManager
    ros::Time then = ros::Time::now();

    while (ros::ok())
    {
        const ros::Time now = ros::Time::now();

        // Read to and write from hardware based on values from the controller_manager
        controller.read();
        cm.update(now, now - then);
        controller.write();

        // Update the times so that we can keep an accurate measurement between
        // update cycles
        then = now;
        loop_rate.sleep();
    }
    return 0;
}
