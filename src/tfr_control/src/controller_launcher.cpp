/**
 * controller_launcher.cpp
 * 
 * This is the primary node for the control package, it registers the controller
 * instance with the controller manager and handles calling updates on the
 * hardware.
 */
#include <ros/ros.h>
#include <sstream>
#include <controller_manager/controller_manager.h>
#include "controller.h"

int main(int argc, char **argv)
{
    // "controller_launcher is the name of this node at run-time"
    ros::init(argc, argv, "controller_launcher");
    ros::NodeHandle n;

    // The controller that we'll be registering with the controller_manager
    tfr_control::Controller controller(true); // Use fake values for testing

    // Register our controller with the controller_manager
    controller_manager::ControllerManager cm(&controller);

    // Start a spinner with one thread, we don't need any more than that
    // This is actually required (vs calling ros::spinOnce() in the loop)
    // by ControllerManager for some reason, as I found in testing.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Store the time of the last update to feed to ControllerManager
    ros::Time then = ros::Time::now();

    while (ros::ok())
    {
        const ros::Time now = ros::Time::now();

        // Read to and write from hardware based on values from the
        // controller_manager
        controller.read();
        cm.update(now, now - then);
        controller.write();

        // Update the times so that we can keep an accurate measurement between
        // update cycles
        then = now;
    }
    return 0;
}
