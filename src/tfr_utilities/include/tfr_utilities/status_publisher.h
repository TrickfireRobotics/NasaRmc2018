/* Utility class for publishing to stdout and mission control at the same time.
 *
 * Published Topics:
 *  -/com - The diagnostics topic
 * */
#ifndef STATUS_PUBLISHER_H
#define STATUS_PUBLISHER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <status_code.h>
#include <tfr_msgs/SystemStatus.h>
#include <cstdint>

class StatusPublisher
{
    public:
        StatusPublisher();
        ~StatusPublisher() = default;
        StatusPublisher(const StatusPublisher&) = delete;
        StatusPublisher& operator=(const StatusPublisher&) = delete;
        StatusPublisher(StatusPublisher&&) = delete;
        StatusPublisher& operator=(StatusPublisher&&) = delete;

        //publishes at standard ros visibility levels
        void info (const StatusCode& code,const float &data) const;
        void debug(const StatusCode& code,const float &data) const;
        void warn (const StatusCode& code,const float &data) const;
        void error(const StatusCode& code,const float &data) const;

        //only publishes at mission control scope
        void missionControl(const StatusCode& code ,const float &data) const;

        //frees up resources for qt framewwork
        void shutdown();

    private:
        ros::NodeHandle n;
        ros::Publisher com;
};
#endif

