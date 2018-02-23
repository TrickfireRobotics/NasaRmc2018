#include <status_publisher.h>

StatusPublisher::StatusPublisher(ros::NodeHandle& node) :
    n{node},
    com{n.advertise<tfr_msgs::SystemStatus>("com",5)} 
{}

// ROS_INFO visibility + mission control
void StatusPublisher::info(const StatusCode &code, const float &data) 
{
    ROS_INFO("%s" ,getStatusMessage(code,data).c_str());
    missionControl(code,data);
}

// ROS_DEBUG visibility + mission control
void StatusPublisher::debug(const StatusCode &code, const float &data) 
{
    ROS_DEBUG("%s" ,getStatusMessage(code,data).c_str());
    missionControl(code,data);
}

// ROS_WARN visibility + mission control
void StatusPublisher::warn(const StatusCode &code, const float &data) 
{
    ROS_WARN("%s" ,getStatusMessage(code,data).c_str());
    missionControl(code,data);
}

// ROS_ERROR visibility + mission control
void StatusPublisher::error(const StatusCode &code, const float &data)
{
    ROS_ERROR("%s" ,getStatusMessage(code,data).c_str());
    missionControl(code,data);
}

//Sends message to mission control
void StatusPublisher::missionControl(const StatusCode &code, const float &data) 
{
    tfr_msgs::SystemStatus status;
    status.time_stamp = ros::Time::now();
    status.status_code = static_cast<uint8_t>(code);
    status.data = data;
    com.publish(status);
}

