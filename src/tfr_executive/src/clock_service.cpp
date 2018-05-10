/*
 * The clock service is a really simple node that exposes some utility services
 * to simplify the operation of the executive system.
 * 
 * PARAMETERS
 * - ~driving_time: The amount oftime we anticipate localization to take in seconds. (`float`, default 45)
 * - ~driving_time: The amount of time we anticipate driving to take seconds. (`float`, default 37)
 * - ~dumping_time: The amount of time we anticipate dumping to take in seconds.  (`float`, default 45)
 * - ~mission_time: The amount of time we allocate for the mission in total 
 *                  seconds. (`float`, default: 600)
 * SERVICES  
 * 1. start_mission: starts the mission clock.
 * 2. time_remaining: returns the amount of time remaining in the mission
 * 3. digging_time: Gives the digging time which is equal to  
 *                  (duration - start - driving_time - dumping_time) or 0 
 *                  if negative duration. 
 * */
#include <ros/ros.h>
#include <cmath>
#include <std_srvs/Empty.h>
#include <tfr_msgs/DurationSrv.h>

class ClockService
{
    public:
        ClockService(ros::NodeHandle &n, ros::Duration& mission, 
                ros::Duration& localization, ros::Duration& driving, ros::Duration& dumping):
            start_mission{n.advertiseService("start_mission", &ClockService::startMission, this)},
            time_remaining{n.advertiseService("time_remaining", &ClockService::timeRemaining, this)},
            digging_time{n.advertiseService("digging_time", &ClockService::diggingTime , this)},
            mission_start{},
            mission_duration{mission},
            localization_duration{localization},
            driving_duration{driving},
            dumping_duration{dumping}
        { }

        ~ClockService() = default;
        ClockService(const ClockService&) = delete;
        ClockService& operator=(const ClockService&) = delete;
        ClockService(ClockService&&) = delete;
        ClockService& operator=(ClockService&&) = delete;


    private: 
        /*
         * Starts the mission clock, takes in the empty service request.
         * */
        bool startMission(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res)
        {
            ROS_INFO("mission started");
            mission_start = ros::Time::now();
            return true;
        }

        /*
         * Gives the remaining time in the mission, fills a duration request.
         * */
        bool timeRemaining(tfr_msgs::DurationSrv::Request &req,
                tfr_msgs::DurationSrv::Response &res)
        {
            if (!mission_start.isValid())
                ROS_WARN("Clock Service: Uninitialized Mission Clock Detected");
            res.duration = mission_duration - (ros::Time::now() - mission_start);
            return true;
        }

        /*
         * Gives the time to allow for digging, fills a duration request.
         * */
        bool diggingTime(tfr_msgs::DurationSrv::Request &req,
                tfr_msgs::DurationSrv::Response &res)
        {
            timeRemaining(req, res);
            if (!mission_start.isValid())
                ROS_WARN("Clock Service: Uninitialized Mission Clock Detected");
            res.duration = res.duration 
                - localization_duration
                - driving_duration 
                - localization_duration
                - driving_duration 
                - localization_duration
                - dumping_duration;
            return true;
        }

        ros::ServiceServer start_mission;
        ros::ServiceServer time_remaining;
        ros::ServiceServer digging_time;

        ros::Time mission_start;
        ros::Duration mission_duration;
        ros::Duration localization_duration;
        ros::Duration driving_duration;
        ros::Duration dumping_duration;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clock_service");
    ros::NodeHandle n{};
    double mission_time, driving_time, dumping_time, localization_time;
    ros::param::param<double>("~mission_time", mission_time, 600);
    ros::param::param<double>("~driving_time", driving_time, 35);
    ros::param::param<double>("~dumping_time", dumping_time, 45);
    ros::param::param<double>("~localization_time", localization_time, 45);

    ros::Duration 
        mission{mission_time}, 
        driving{driving_time},
        dumping{dumping_time}, 
        localization{localization_time};
    ClockService clock{n, mission, localization,  driving, dumping};
    ros::spin();
    return 0;
}
