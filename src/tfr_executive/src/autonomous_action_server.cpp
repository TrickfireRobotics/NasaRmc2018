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
 * On startup the action server will be expecting a command to start autonomy, 
 * and initialize autonomous operation. 
 *
 * PRECONDITION
 * The clock service must be up and started, anything else is undefined
 * behavior.
 * 
 * PARAMETERS
 * - ~rate: the rate in hz to check to preemption during long running calls, (double, default: 10)
 * - ~localization: whether to run localization or not (bool, default: true);
 * - ~navigation_to: whether to run navigation_to or not (bool, default: true);
 * - ~digging: whether to run digging or not (bool, default: true);
 * - ~hole: whether to place the hole or not (bool, default: true);
 * - ~navigation_from: whether to run from or not (bool, default: true);
 * - ~dumping: whether to run dumping or not (bool, default: true);
 * 
 * PUBLISHED TOPICS
 * - /com 
 *   - the communication topic
 * */
#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/DurationSrv.h>
#include <tfr_msgs/NavigationAction.h>
#include <tfr_msgs/DiggingAction.h>
#include <tfr_msgs/SetOdometry.h>
#include <tfr_utilities/location_codes.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

class AutonomousExecutive
{
    public:
        AutonomousExecutive(ros::NodeHandle &n,double f):
            server{n, "autonomous_action_server", 
                boost::bind(&AutonomousExecutive::autonomousMission, this, _1),
                false},
            localizationClient{n, "localize"},
            navigationClient{n, "navigate"},
            diggingClient{n, "dig"},
            dumpingClient{n, "dump"},
            frequency{f}
        {
            ros::param::param<bool>("~localization", LOCALIZATION, true);
            if (LOCALIZATION)
            {
                ROS_INFO("Autonomous Action Server: Connecting to localization server");
                localizationClient.waitForServer();
                ROS_INFO("Autonomous Action Server: Connected to localization server");
            }
            ros::param::param<bool>("~navigation_to", NAVIGATION_TO, true);
            ros::param::param<bool>("~navigation_from", NAVIGATION_FROM, true);
            if (NAVIGATION_TO || NAVIGATION_FROM)
            {
                ROS_INFO("Autonomous Action Server: Connecting to navigation server");
                navigationClient.waitForServer();
                ROS_INFO("Autonomous Action Server: Connected to navigation server");
            }
            ros::param::param<bool>("~digging", DIGGING, true);
            if (DIGGING)
            {
                ROS_INFO("Autonomous Action Server: Connecting to digging server");
                diggingClient.waitForServer();
                ROS_INFO("Autonomous Action Server: Connected to digging server");
            }
            ros::param::param<bool>("~dumping", DUMPING, true);
            if (DUMPING)
            {
                ROS_INFO("Autonomous Action Server: Connecting to digging server");
                dumpingClient.waitForServer();
                ROS_INFO("Autonomous Action Server: Connected to digging server");
            }
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
         * 1. Run the localization subsystem.
         * 2. Store the odometry information gathered.
         * 3. Run the navigation subsystem with the drive to mining zone setting.
         * 4. Run the digging action subsystem with the digging time, from the `get_digging_time` service.
         * 5. Run the navigation subsystem with the return from mining zone option.
         * 6. Run the dumping subsystem.
         * 7. Put the system in teleop mode and await instructions.
         * 
         * ACTION COMPONENTS
         * - Goal: none
         * - Feedback: none
         * - Result: none */ 
        void autonomousMission(const tfr_msgs::EmptyGoalConstPtr &goal)
        {
            
            ROS_INFO("Autonomous Action Server: mission started");
            if (server.isPreemptRequested() || ! ros::ok())
            {
                server.setPreempted();
                return;
            }

            if (LOCALIZATION)
            {
                ROS_INFO("Autonomous Action Server: commencing localization");
                tfr_msgs::EmptyGoal goal{};
                localizationClient.sendGoal(goal);
                //handle preemption
                while (!localizationClient.getState().isDone())
                {
                    if (server.isPreemptRequested() || ! ros::ok())
                    {
                        localizationClient.cancelAllGoals();
                        server.setPreempted();
                        ROS_INFO("Autonomous Action Server: localization preempted");
                        return;
                    }
                    frequency.sleep();
                }
                if (localizationClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Autonomous Action Server: localization failed");
                    server.setAborted();
                    return;
                }

                ROS_INFO("Autonomous Action Server: stabilizing odometry");
                tfr_msgs::SetOdometry::Request req;
                req.pose.orientation.w = 1;
                tfr_msgs::SetOdometry::Response res;

            
                while (!ros::service::call("set_drivebase_odometry", req, res));
                ros::Duration(5.0).sleep();
                ROS_INFO("Autonomous Action Server: odometry stabilized");
                ROS_INFO("Autonomous Action Server: localization finished");

            }

            if (NAVIGATION_TO)
            {
                ROS_INFO("Autonomous Action Server: commencing navigation");
 
                tfr_msgs::NavigationGoal goal;
                //messages can't support user defined types
                goal.location_code= static_cast<uint8_t>(tfr_utilities::LocationCode::MINING);
                navigationClient.sendGoal(goal);
                //handle preemption
                while ( !navigationClient.getState().isDone() && ros::ok())
                {
                    if (server.isPreemptRequested() || ! ros::ok())
                    {
                        navigationClient.cancelAllGoals();
                        server.setPreempted();
                        ROS_INFO("Autonomous Action Server: navigation preempted");
                        return;
                    }
                    frequency.sleep();
                }

                if (navigationClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Autonomous Action Server: navigation to failed");
                    server.setAborted();
                    return;
                }
                ROS_INFO("Autonomous Action Server: navigation finished");
            }

            if (DIGGING)
            {
                ROS_INFO("Autonomous Action Server: commencing digging");
                tfr_msgs::DiggingGoal goal{};
                ROS_INFO("Autonomous Action Server: retrieving digging time");
                tfr_msgs::DurationSrv digging_time;
                ros::service::call("digging_time", digging_time);
                ROS_INFO("Autonomous Action Server: digging time retreived %f",
                        digging_time.response.duration.toSec());
                goal.diggingTime = digging_time.response.duration;
                diggingClient.sendGoal(goal);

                //handle preemption
                while (!diggingClient.getState().isDone())
                {
                    if (server.isPreemptRequested() || ! ros::ok())
                    {
                        diggingClient.cancelAllGoals();
                        server.setPreempted();
                        ROS_INFO("Autonomous Action Server: digging preempted");
                        return;
                    }
                    frequency.sleep();
                }
                if (diggingClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Autonomous Action Server: digging failed");
                    server.setAborted();
                    return;
                }
                ROS_INFO("Autonomous Action Server: digging finished");
            }

            if (NAVIGATION_FROM)
            {
                ROS_INFO("Autonomous Action Server: Connecting to navigation server");
                navigationClient.waitForServer();
                ROS_INFO("Autonomous Action Server: Connected to navigation server");

                ROS_INFO("Autonomous Action Server: commencing navigation");
 
                tfr_msgs::NavigationGoal goal;
                //messages can't support user defined types
                goal.location_code=
                    static_cast<uint8_t>(tfr_utilities::LocationCode::DUMPING);
                navigationClient.sendGoal(goal);
                //handle preemption
                while ( !navigationClient.getState().isDone() && ros::ok())
                {
                    if (server.isPreemptRequested() || ! ros::ok())
                    {
                        navigationClient.cancelAllGoals();
                        server.setPreempted();
                        ROS_INFO("Autonomous Action Server: navigation preempted");
                        return;
                    }
                    frequency.sleep();
                }

                if (navigationClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Autonomous Action Server: navigation to failed");
                    server.setAborted();
                    return;
                }
                ROS_INFO("Autonomous Action Server: navigation finished");
            }
            if (DUMPING)
            {
                ROS_INFO("Autonomous Action Server: Connecting to dumping server");
                navigationClient.waitForServer();
                ROS_INFO("Autonomous Action Server: Connected to dumping server");

                ROS_INFO("Autonomous Action Server: commencing dumping");
 
                tfr_msgs::EmptyGoal goal;
                dumpingClient.sendGoal(goal);
                //handle preemption
                while ( !dumpingClient.getState().isDone() && ros::ok())
                {
                    if (server.isPreemptRequested() || ! ros::ok())
                    {
                        dumpingClient.cancelAllGoals();
                        server.setPreempted();
                        ROS_INFO("Autonomous Action Server: dumping preempted");
                        return;
                    }
                    frequency.sleep();
                }

                if (dumpingClient.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Autonomous Action Server: dumping failed");
                    server.setAborted();
                    return;
                }
                ROS_INFO("Autonomous Action Server: dumping finished");

            }
            ROS_INFO("Autonomous Action Server: AUTONOMOUS MISSION SUCCESS");
            server.setSucceeded();
        }

        actionlib::SimpleActionServer<tfr_msgs::EmptyAction> server;
        actionlib::SimpleActionClient<tfr_msgs::EmptyAction> localizationClient;
        actionlib::SimpleActionClient<tfr_msgs::NavigationAction> navigationClient;
        actionlib::SimpleActionClient<tfr_msgs::DiggingAction> diggingClient;
        actionlib::SimpleActionClient<tfr_msgs::EmptyAction> dumpingClient;

        bool LOCALIZATION;
        bool NAVIGATION_TO;
        bool DIGGING;
        bool HOLE;
        bool NAVIGATION_FROM;
        bool DUMPING;
        //how often to check for preemption
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
