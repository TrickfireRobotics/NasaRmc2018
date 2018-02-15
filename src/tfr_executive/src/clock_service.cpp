
#include <ros/ros.h>
#include <tfr_msgs/EmptySrv.h>
#include <tfr_msgs/DurationSrv.h>

class ClockService
{
    public:
        ClockService(ros::NodeHandle &n, ros::Duration& mission, 
                ros::Duration& driving, ros::Duration& dumping):
            start_mission{n.advertiseService("start_mission", &ClockService::startMission, this)},
            time_remaining{n.advertiseService("time_remaining", &ClockService::timeRemaining, this)},
            dumping_time{n.advertiseService("dumping_time", &ClockService::dumpingTime, this)},
            mission_start{ros::Time::now()},
            mission_duration{mission},
            driving_duration{driving},
            dumping_duration{dumping}{ }
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
        bool dumpingTime(tfr_msgs::DurationSrv::Request &req,
                tfr_msgs::DurationSrv::Response &res)
        {
            timeRemaining(req, res);
            res.duration = res.duration - driving_duration - dumping_duration;
            return true;
        }

        ros::ServiceServer start_mission;
        ros::ServiceServer time_remaining;
        ros::ServiceServer dumping_time;

        ros::Time mission_start;
        ros::Duration mission_duration;
        ros::Duration driving_duration;
        ros::Duration dumping_duration;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clock_service");
    ros::NodeHandle n{};
    int mission_time, driving_time, dumping_time;
    ros::param::param<int>("~mission_time", mission_time, 600);
    ros::param::param<int>("~driving_time", driving_time, 60);
    ros::param::param<int>("~dumping_time", mission_time, 30);
    ros::spin();
    return 0;
}
