/****************************************************************************************
 * File:            bin_control_server.cpp
 * 
 * Purpose:         This is the implementation file for the BinControlServer class.
 *                  See tfr_control/include/tfr_control/bin_control_server.h for details.
 ***************************************************************************************/
#include "bin_control_server.h"
#include <boost/bind.hpp>
#include "std_msgs/Float64.h"


// I could also pass in the name of the server, but it would end up being a literal
// string anyway unless it also gets passed into the node, which seems a bit excessive.
// If it's going to be a literal string we might as well put it as close to its destination
// as possible so it's clear what it's being used for.
BinControlServer::BinControlServer(ros::NodeHandle& n, float target_bin_angle) : node{n}, 
    server{n, "bin_control", boost::bind(&BinControlServer::ControlBin, this, _1), false},
    TARGET_BIN_ANGLE{target_bin_angle}, signal_mutex{}, bin_task_completed{false}
{
    bin_command_publisher = node.advertise<std_msgs::Float64>(
            "bin_position_controller/command", 100);
    server.start();
}

// Action Server callback function
void BinControlServer::ControlBin(const tfr_msgs::BinGoalConstPtr& goal)
{
    std_msgs::Float64 command;
    tfr_msgs::BinResult result;

    if (goal->command_code == goal->RAISE_BIN)
    {
        command.data = TARGET_BIN_ANGLE;
        bin_command_publisher.publish(command);
    }
    else if (goal->command_code == goal->LOWER_BIN)
    {
        command.data = 0;
        bin_command_publisher.publish(command);
    }
    else
    {
        ROS_WARN("Bin Controller received an invalid control code: %d", goal->command_code);
        result.return_code = result.ERROR_ENCOUNTERED;
        server.setAborted(result);
        return;
    }

    // This will block until SignalBinController() is called by the controller
    wait_for_bin();

    if (goal->command_code == goal->RAISE_BIN)
    {
        result.return_code = result.BIN_RAISED;
    }
    else if (goal->command_code == goal->LOWER_BIN)
    {
        result.return_code = result.BIN_LOWERED;
    }

    server.setSucceeded(result);
}

// This will signal the bin controller that the bin has reached its destination
// Calling this will unblock the wait_for_bin() function which should be running
// in the ActionServer part of the class (which will be running in by a different 
// process, hence the signaling semantics).
void BinControlServer::SignalBinController()
{
    // unlocks when it falls out of scope
    std::unique_lock<std::mutex> signal_lock(signal_mutex);
    bin_task_completed = true;
    signal.notify_one();
}


void BinControlServer::wait_for_bin()
{
    // unlocked by the wait() call
    std::unique_lock<std::mutex> signal_lock(signal_mutex);
    while (!bin_task_completed)
    {
        // TODO: Instead of using the signal.wait() semantics, we could check for preemptions in a 
        //       polling loop here, but I don't think there's an easy way to tell the hardware_interface
        //       to simply stop moving the bin; it takes in a target angle and simply goes until it gets 
        //       there. If we think supporting preemption is important for this component, we can 
        //       investigate further options.

        // http://en.cppreference.com/w/cpp/thread/condition_variable 
        // https://stackoverflow.com/questions/16350473/why-do-i-need-stdcondition-variable
        // wait() will release the lock. Sometimes the signal will be notified by accident, so
        // a check on bin_task_completed is required. wait() reaquires the lock after it unblocks.
        signal.wait(signal_lock);
        if (!bin_task_completed)
        {
            break;
        }
    }
}