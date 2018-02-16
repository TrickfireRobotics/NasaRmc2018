/*
 * DESCRIPTION
 * The executive action server, is the main business logic for autonomous 
 * operation. It is initialized at system startup, and is commanded by the 
 * distributed mission control node. 
 * 
 * The autonomous operation starts when commence autonomous operation 
 * command gets called. This will continue, until:
 * - The action gets into an inoperable state and ceases operation.
 * - The action completes the mission successfully.
 * - The action server is given a command, and is preempted.
 * 
 * Once the mission ceases operation resuming autonomy is not a system 
 * priority. Attempting to return the system to a safe state is.
 * 
 * If preemption or mission failure occurs, it is the duty of the executive 
 * node to preempt all other active action servers it has called, allow them 
 * to return to a safe state on a timeout and promptly exit. 
 * 
 * An additional responsibility of the autonomous server is to start the 
 * mission clock using the `start_mission` service.
 * 
 * On startup the action server will be expecting a command to start autonomy, 
 * and initialize autonomous operation. 
 * 
 * PARAMETERS
 * - ~rate: the rate in hz to check to preemption during long running calls, (double, default: 10)
 * 
 * PUBLISHED TOPICS
 * - /com 
 *   - the communication topic
 * */
#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/EmptySrv.h>
#include <tfr_msgs/DurationSrv.h>
#include <actionlib/server/simple_action_server.h>
class AutonomousExecutive
{
    public:
        AutonomousExecutive(ros::NodeHandle &n,double f):
            server{n, "autonomous_action_server", 
                boost::bind(&AutonomousExecutive::autonomousMission, this, _1),
                false},
            frequency{f}
        {
            server.start();
            ROS_INFO("Autonomous Action Server: online, %f",
                    ros::Time::now().toSec());
        }
        ~AutonomousExecutive() = default;
        AutonomousExecutive(const AutonomousExecutive&) = delete;
        AutonomousExecutive& operator=(const AutonomousExecutive&) = delete;
        AutonomousExecutive(AutonomousExecutive&&) = delete;
        AutonomousExecutive& operator=(AutonomousExecutive&&) = delete;
    private:
        /* ACTION DESCRIPTION
         * The main autonomous procedure is long running and needs to be responsive to 
         * preemption. This means that after making time consuming calls, for example 
         * starting navigation, it needs to do so in a non blocking manner, and query
         * for preemption from the user at a set interval. At this interval it is also 
         * appropriate to ask for feedback from the time consuming processes, or check 
         * for success. The definition of long running means any call that starts a 
         * subsystem. 
         * 
         * Also this is an action server so it is responsible for reporting important 
         * state changes and updates to the standard communication channel.
         * 
         * Judiciously following these guidelines, the skeleton of the main procedure is 
         * as follows:
         * 1. Start mission clock.
         * 2. Run the localization subsystem.
         * 3. Store the odometry information gathered.
         * 4. Run the navigation subsystem with the drive to mining zone setting.
         * 5. Run the digging action subsystem with the digging time, from the `get_digging_time` service.
         * 6. Run the navigation subsystem with the return from mining zone option.
         * 7. Run the dumping subsystem.
         * 8. Put the system in teleop mode and await instructions.
         * 
         * ACTION COMPONENTS
         * - Goal: none
         * - Feedback: none
         * - Result: none
         */ 
        void autonomousMission(const tfr_msgs::EmptyGoalConstPtr &goal)
        {
            ROS_INFO("Autonomous Action Server: mission started");

            ROS_INFO("Autonomous Action Server: starting clock %f",
                    ros::Time::now().toSec());
            tfr_msgs::EmptySrv start;
            ros::service::call("start_mission", start);
            ROS_INFO("Autonomous Action Server: clock started %f",
                    ros::Time::now().toSec());

            ROS_INFO("Autonomous Action Server: commencing localization");
            ROS_INFO("Autonomous Action Server: localization finished");

            ROS_INFO("Autonomous Action Server: commencing navigation");
            ROS_INFO("Autonomous Action Server: navigation finished");

            ROS_INFO("Autonomous Action Server: retrieving digging time");
            tfr_msgs::DurationSrv digging_time;
            ros::service::call("digging_time", digging_time);
            ROS_INFO("Autonomous Action Server: digging time retreived %f",
                    digging_time.response.duration.toSec());

            ROS_INFO("Autonomous Action Server: commencing digging");
            ROS_INFO("Autonomous Action Server: digging finished");

            ROS_INFO("Autonomous Action Server: commencing navigation");
            ROS_INFO("Autonomous Action Server: navigation finished");

            ROS_INFO("Autonomous Action Server: commencing dumping");
            ROS_INFO("Autonomous Action Server: dumping finished");

            tfr_msgs::EmptyResult result{};
            server.setSucceeded(result);
            ROS_INFO("Autonomous Action Server: mission finished");
        }

        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        ros::Duration frequency;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_action_server");
    ros::NodeHandle n{};
    double rate;
    ros::param::param<double>("~rate", rate, 10.0);
    AutonomousExecutive autonomousExecutive{n, 1.0/rate};
    ros::spin();
    return 0;
}
