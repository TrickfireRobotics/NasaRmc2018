/****************************************************************************************
 * File:            test_digging_queue.h
 * 
 * Purpose:         This class implements a simple node that tests the
 *                  functionality of 
 ***************************************************************************************/
#include <ros/ros.h>
#include "digging_queue.h"

int main(int argc, char** argv)
{
    // Basic ROS setup
    ros::init(argc, argv, "example_client");
    ros::NodeHandle n;

    tfr_mining::DiggingQueue queue;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Starting digging queue test...");

    while (!queue.isEmpty()) {
        ROS_INFO("Popping next set off of stack...");
        tfr_mining::DiggingSet set = queue.popDiggingSet();
        ROS_INFO("Set time: %f", set.getTimeEstimate());
    }

    ROS_INFO("Digging queue is now empty.");

    return 0;
}