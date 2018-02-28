/**
 * controller.cpp
 * 
 * This class is in charge of handling the physical hardware interface with
 * the robot itself, and is started by the controller_launcher node.
 */
#include "controller.h"

using hardware_interface::JointStateHandle;
using hardware_interface::JointHandle;

namespace tfr_control
{
    Controller::Controller(bool fakes, const double *lower_lim, const double *upper_lim) :
        use_fake_values(fakes), lower_limits(lower_lim), upper_limits(upper_lim)
    {
        // Note: the string parameters in these constructors must match the
        // joint names from the URDF. If one changes, so must the other.

        // Connect and register the joint state and effort interfaces
        registerVelocityJoint("left_tread_joint", Joint::LEFT_TREAD);
        registerVelocityJoint("right_tread_joint", Joint::RIGHT_TREAD);
        //registerArmJoint("bin_joint", Joint::BIN); TODO FIGURE THIS OUT
        registerEffortJoint("turntable_joint", Joint::TURNTABLE);
        registerEffortJoint("lower_arm_joint", Joint::LOWER_ARM);
        registerEffortJoint("upper_arm_joint", Joint::UPPER_ARM);
        registerEffortJoint("scoop_joint", Joint::SCOOP);
        registerInterface(&joint_state_interface);
        registerInterface(&joint_effort_interface);
    }

    // Return true if bin is finished moving to its new position.
    // TODO: A more explicit return type might be better here, or a possible redesign.
    //       If other values read end up needing to be returned, then this should 
    //       probably return some data structure. It may be that read values get published
    //       out on topics instead.
    void Controller::read() 
    {
        // TODO: Waiting on further hardware development to implement

        // Return true if bin has finished moving to its new position.
        // For now, just return true so the bin controller doesn't block forever.
    }

    void Controller::write() 
    {
        // TODO: Waiting on further hardware development to implement
        if (use_fake_values) 
        {
            // Update all actuators velocity with the command (effort in).
            // Then update the position as derivative of the velocity over time.
            // ADAM make sure to update this index when you need to
            for (int i = 3; i < CONTROLLER_COUNT; i++) 
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

        prev_time = ros::Time::now();
    }

    void Controller::registerVelocityJoint(std::string name, Joint joint) 
    {
        auto idx = static_cast<int>(joint);
        JointStateHandle state_handle(name, &position_values[idx],
            &velocity_values[idx], &effort_values[idx]);
        JointHandle handle(state_handle, &command_values[idx]);
        drivebase_velocity_interface.registerHandle(handle);
    }

    void Controller::registerEffortJoint(std::string name, Joint joint) 
    {
        auto idx = static_cast<int>(joint);
        JointStateHandle state_handle(name, &position_values[idx],
            &velocity_values[idx], &effort_values[idx]);
        joint_state_interface.registerHandle(state_handle);
        JointHandle handle(state_handle, &command_values[idx]);
        joint_effort_interface.registerHandle(handle);
    }

    double Controller::getUpdateTime() 
    {
        return (ros::Time::now() - prev_time).nsec / 1E9;
    }
}
