#include "bin_control_server.h"
#include <boost/bind.hpp>
#include "std_msgs/Float64.h"

BinControlServer::BinControlServer(ros::NodeHandle& n, float target_bin_angle) : node{n}, 
    server{n, "bin_control", boost::bind(&BinControlServer::ControlBin, this, _1), false},
    TARGET_BIN_ANGLE{target_bin_angle}
{
    bin_signal_service = node.advertiseService("signal_bin_state", &BinControlServer::SignalBinController, this);
    bin_command_publisher = node.advertise<std_msgs::Float64>(
            "left_tread_velocity_controller/command", 100);
    server.start();
}

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

    while (true)
    {
        // wait for hardware interface to signal bin is ready

        // TODO: We could check for preemptions here, but I don't think there's an easy way to tell
        //       the hardware_interface to simply stop moving the bin; it takes in a target angle 
        //       and simply goes until it gets there. If we think supporting preemption is important
        //       for this component, we can investigate further options.

        break;
    }

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

bool BinControlServer::SignalBinController(tfr_msgs::BinState::Request &request, tfr_msgs::BinState::Response &resonse)
{

}