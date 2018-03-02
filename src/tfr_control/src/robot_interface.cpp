/** * controller.cpp * 
 * This class is in charge of handling the physical hardware interface with
 * the robot itself, and is started by the controller_launcher node.
 */
#include "robot_interface.h"

using hardware_interface::JointStateHandle;
using hardware_interface::JointHandle;

namespace tfr_control
{
    RobotInterface::RobotInterface(ros::NodeHandle &n, bool fakes, 
            const double *lower_lim, const double *upper_lim) :
        pwm{},
        encoders{n.subscribe("encoders", 5, &RobotInterface::readEncoders, this)},
        use_fake_values{fakes}, lower_limits{lower_lim}, upper_limits{upper_lim}
    {
        // Note: the string parameters in these constructors must match the
        // joint names from the URDF, and yaml controller description. 

        // Connect and register each joint with appropriate interfaces at our
        // layer
        registerJoint("left_tread_joint", Joint::LEFT_TREAD);
        registerJoint("right_tread_joint", Joint::RIGHT_TREAD);
        registerJoint("bin_joint", Joint::BIN); 
        registerJoint("turntable_joint", Joint::TURNTABLE);
        registerJoint("lower_arm_joint", Joint::LOWER_ARM);
        registerJoint("upper_arm_joint", Joint::UPPER_ARM);
        registerJoint("scoop_joint", Joint::SCOOP);
        //register the interfaces with the controller layer
        registerInterface(&joint_state_interface);
        registerInterface(&joint_effort_interface);
        pwm.enablePWM(true);
    }

    void RobotInterface::read() 
    {
        //Grab the neccessary data
        tfr_msgs::EncoderReading encoders;
        if (latest_encoders != nullptr)
            encoders = *latest_encoders;

        /*
         * Populate the shared memory, note items that are not expicity needed
         * are written to some safe sensible default (usually 0)
         * */

        //LEFT_TREAD
        velocity_values[static_cast<int>(Joint::LEFT_TREAD)] = encoders.left_vel;
        position_values[static_cast<int>(Joint::LEFT_TREAD)] = 0;
        effort_values[static_cast<int>(Joint::LEFT_TREAD)] = 0;
        //TODO put in other components

        ROS_INFO("l_enc: %f", velocity_values[static_cast<int>(Joint::LEFT_TREAD)]); 
    }

    void RobotInterface::write() 
    {
        //TODO adam delete this old test code when ready
        if (use_fake_values) 
        {
            // Update all actuators velocity with the command (effort in).
            // Then update the position as derivative of the velocity over time.
            // ADAM make sure to update this index when you need to
            for (int i = 3; i < JOINT_COUNT; i++) 
            {
                velocity_values[i] = command_values[i];
                position_values[i] += velocity_values[i] * getUpdateTime();
                // If this joint has limits, clamp the range down
                if (abs(lower_limits[i]) >= 1E-3 || abs(upper_limits[i]) >= 1E-3) 
                {
                    position_values[i] =
                        std::max(std::min(position_values[i],
                                    upper_limits[i]), lower_limits[i]);
                }
            }
        }

        /**
         * write the specified hardware taking in to account safety precautions
         * It is up to us to make sure the signals being given are scaled
         * appropriately, and routed to the right place
         * */

        //LEFT_TREAD
        double signal = velocityToPWM(command_values[static_cast<int>(Joint::LEFT_TREAD)]);
        pwm.set(PWMInterface::Address::LEFT_TREAD, signal);
        ROS_INFO("l_vel, l_pwm: %f, %f", command_values[static_cast<int>(Joint::LEFT_TREAD)], signal); 
        
        //upkeep
        prev_time = ros::Time::now();
    }


    /*
     * Register this joint with each neccessary hardware interface
     * */
    void RobotInterface::registerJoint(std::string name, Joint joint) 
    {
        auto idx = static_cast<int>(joint);
        //give the joint a state
        JointStateHandle state_handle(name, &position_values[idx],
            &velocity_values[idx], &effort_values[idx]);
        joint_state_interface.registerHandle(state_handle);

        //allow the joint to be commanded
        JointHandle handle(state_handle, &command_values[idx]);
        /* note there might need to be more complicated logic here in the 
         * future if we need to use interfaces outside of the effort_controllers
         * package
         */
        joint_effort_interface.registerHandle(handle);
    }


    /*
     * Takes in a velocity, and converts it to pwm for the drivebase.
     * Velocity is in meters per second, and output is in raw pwm frequency.
     * Scaled to match the values expected by pwm interface
     * */
    double RobotInterface::velocityToPWM(const double &vel)
    {
        //we don't anticipate this changing very much keep at method level
        double max_vel = 1;
        int sign = (vel < 0) ? -1 : 1;
        double magnitude = std::min((vel*sign)/max_vel, 1.0);
        return sign * magnitude;
    }

    /*
     * Callback for our encoder subscriber
     * */
    void RobotInterface::readEncoders(const tfr_msgs::EncoderReadingConstPtr &msg)
    {
        latest_encoders = msg;
    }

    /*
     * gets the time elapsed since last read 
     * TODO this is currently only used by test code, and is not really needed
     * by this object it should be deleted when the test code is deleted
     * */
    double RobotInterface::getUpdateTime() 
    {
        return (ros::Time::now() - prev_time).nsec / 1E9;
    }
}
