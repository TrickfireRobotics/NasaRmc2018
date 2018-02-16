/*
 * The teleop action server is  in charge of processing user commands quickly 
 * and performing smooth remote operation for our operations team. All commands
 * for teleoperation will be processed by this server, except for emergency 
 * stop, which is handled by the control system directly for fast response time.
 * 
 * The commands it supports:
 * - None
 * - Move
 *   - Forward
 *   - Backward
 *   - Turn left
 *   - Turn right
 * - Dig
 *   - Executes digging for some duration calculated by the `get_digging_time` 
 *     service, must support preemption.
 * - Dump
 *   - Raising the dumping bin, currently does not support preemption.
 * - Reset Motor state
 *   - Resets the arm, motor and control system to a safe position, ready to 
 *     recieve commands.
 * 
 * PARAMETERS 
 * -~linear_velocity: the max linear velocity. (double, default: 0.25)
 * -~angular_velocity: the max angular velocity. (double, default: 0.1)
 */ 
#include <ros/ros.h>
#include <ros/console.h>
#include <tfr_utilities/teleop_code.h>
#include <tfr_msgs/TeleopAction.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>


class TeleopExecutive
{
    public:
        
        struct DriveVelocity
        {
            private:
                double linear;
                double angular;
            public:
                DriveVelocity(double lin, double ang):
                    linear{lin}, angular{ang} {}
                double getLinear(){return linear;}
                double getAngular(){return angular;}
        };

        TeleopExecutive(ros::NodeHandle &n , DriveVelocity &drive) :
            server{n, "teleop_action_server",
                boost::bind(&TeleopExecutive::processCommand, this, _1),
                false},
            drivebase{n.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
            drive_stats{drive}
        {
            server.start();
            ROS_INFO("Teleop Action Server: Online %f", ros::Time::now().toSec());
        }
        ~TeleopExecutive() = default;
        TeleopExecutive(const TeleopExecutive&) = delete;
        TeleopExecutive& operator=(const TeleopExecutive&) = delete;
        TeleopExecutive(TeleopExecutive&&) = delete;
        TeleopExecutive& operator=(TeleopExecutive&&) = delete;

    private:

        /*
         *The main callback for processing user commands
         * 
         * ACTION SPECIFICATION
         * The logic of the processing commands is:
         * 1. unpack the message
         * 2. if dumping dump
         * 2. else if digging process asynchronously while checking for preemption
         * 2. else if reset reset
         * 3. send the most relevant driving message (forward, backward, left, right, or stop)
         * 3. exit
         * 
         * This approach avoids additional threading beyond the action server, and should meet our response requirements.
         * 
         * ACTION MESSAGE
         * - Goal: uint8 command
         * - Feedback: none
         * - Result: none
         * */
        void processCommand(const tfr_msgs::TeleopGoalConstPtr& goal)
        {
            geometry_msgs::Twist move_cmd{};
            auto code = static_cast<tfr_utilities::TeleopCode>(goal->code);
            if (code == tfr_utilities::TeleopCode::NONE)
            {
                ROS_INFO("Teleop Action Server: Command Revieved, RESET");
                //all zeros by default
                drivebase.publish(move_cmd);
            }
            else if(code == tfr_utilities::TeleopCode::FORWARD)
            {
                ROS_INFO("Teleop Action Server: Command Revieved, FORWARD");
                move_cmd.linear.x = drive_stats.getLinear();
                drivebase.publish(move_cmd);
            }
            else if (code == tfr_utilities::TeleopCode::BACKWARD)
            {
                ROS_INFO("Teleop Action Server: Command Revieved, BACKWARD");
                move_cmd.linear.x = -drive_stats.getLinear();
                drivebase.publish(move_cmd);
            }
            else if (code == tfr_utilities::TeleopCode::LEFT)
            {
                ROS_INFO("Teleop Action Server: Command Revieved, LEFT");
                move_cmd.angular.z = drive_stats.getAngular();
                drivebase.publish(move_cmd);
           }
            else if (code == tfr_utilities::TeleopCode::RIGHT)
            {
                ROS_INFO("Teleop Action Server: Command Revieved, RIGHT");
                move_cmd.angular.z = -drive_stats.getAngular();
                drivebase.publish(move_cmd);
            }
            else if (code == tfr_utilities::TeleopCode::DIG)
            {
                ROS_INFO("Teleop Action Server: Command Revieved, DIG");
                //TODO hook up digging when ready
            }
            else if (code == tfr_utilities::TeleopCode::DUMP)
            {
                ROS_INFO("Teleop Action Server: Command Revieved, DUMP");
                //TODO hook up dumping when ready
            }
            else if (code == tfr_utilities::TeleopCode::RESET)
            {
                ROS_INFO("Teleop Action Server: Command Revieved, RESET");
                //TODO hook up reset when ready
            }
            else
            {
                ROS_WARN("Teleop Action Server: UNRECOGNIZED COMMAND");
            }

            tfr_msgs::TeleopResult result{};
            server.setSucceeded(result);
        }

        actionlib::SimpleActionServer<tfr_msgs::TeleopAction> server;
        ros::Publisher drivebase;
        DriveVelocity &drive_stats;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleopaction_server");
    ros::NodeHandle n{};
    double linear_velocity, angular_velocity;
    ros::param::param<double>("~linear_velocity", linear_velocity, 0.25);
    ros::param::param<double>("~angular_velocity", angular_velocity, 0.1);
    TeleopExecutive::DriveVelocity velocities{linear_velocity, angular_velocity};
    TeleopExecutive teleop{n, velocities};
    ros::spin();
    return 0;
}
