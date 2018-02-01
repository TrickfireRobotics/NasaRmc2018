#ifndef BIN_CONTROL_SERVER_H
#define BIN_CONTROL_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tfr_msgs/BinAction.h>
#include <mutex>
#include <condition_variable>

using Server = actionlib::SimpleActionServer<tfr_msgs::BinAction>;

class BinControlServer
{
public:
    BinControlServer() = delete;
    explicit BinControlServer(ros::NodeHandle& n, float target_bin_angle);
    BinControlServer(const BinControlServer& other) = delete;
    BinControlServer(BinControlServer&&) = delete;

    ~BinControlServer() = default;

    BinControlServer& operator=(const BinControlServer&) = delete;
    BinControlServer& operator=(BinControlServer&&) = delete;

    void ControlBin(const tfr_msgs::BinGoalConstPtr& goal);
    void SignalBinController();

private:
    ros::NodeHandle& node;
    Server server;
    ros::Publisher bin_command_publisher;
    const float TARGET_BIN_ANGLE;
    std::mutex signal_mutex;
    bool bin_task_completed;
    std::condition_variable signal;

};

#endif // BIN_CONTROL_SERVER_H
