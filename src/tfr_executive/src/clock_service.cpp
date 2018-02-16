/*
 * The clock service is a really simple node that exposes some utility services
 * to simplify the operation of the executive system.
 * 
 * PARAMETERS 
 * - ~driving_time: The amount of time we anticipate driving, to take in 
 *                  seconds. (`float`, default 60)
 * - ~dumping_time: The amount of time we anticipate dumping to take in 
 *                  seconds. (`float`, default 30)
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
#include <tfr_msgs/EmptySrv.h>
#include <tfr_msgs/DurationSrv.h>

class ClockService
{
    public:
        ClockService(ros::NodeHandle &n, ros::Duration& mission, 
                ros::Duration& driving, ros::Duration& dumping):
            start_mission{n.advertiseService("start_mission", &ClockService::startMission, this)},
            time_remaining{n.advertiseService("time_remaining", &ClockService::timeRemaining, this)},
            digging_time{n.advertiseService("digging_time", &ClockService::diggingTime , this)},
            mission_start{},
            mission_duration{mission},
            driving_duration{driving},
            dumping_duration{dumping}
        {
        }

        ~ClockService() = default;
        ClockService(const ClockService&) = delete;
        ClockService& operator=(const ClockService&) = delete;
        ClockService(ClockService&&) = delete;
        ClockService& operator=(ClockService&&) = delete;


    private: 
        /*
         * Starts the mission clock, takes in the empty service request.
         * */
        bool startMission(tfr_msgs::EmptySrv::Request &req,
                tfr_msgs::EmptySrv::Response &res)
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
            res.duration = res.duration - driving_duration - dumping_duration;
            return true;
        }

        ros::ServiceServer start_mission;
        ros::ServiceServer time_remaining;
        ros::ServiceServer digging_time;

        ros::Time mission_start;
        ros::Duration mission_duration;
        ros::Duration driving_duration;
        ros::Duration dumping_duration;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clock_service");
    ros::NodeHandle n{};
    double mission_time, driving_time, dumping_time;
    ros::param::param<double>("~mission_time", mission_time, 600);
    ros::param::param<double>("~driving_time", driving_time, 60);
    ros::param::param<double>("~dumping_time", dumping_time, 30);

    ros::Duration 
        mission{mission_time}, 
        driving{driving_time},
        dumping{dumping_time};
    ClockService clock{n, mission, driving, dumping};
    ros::spin();
    return 0;
}
