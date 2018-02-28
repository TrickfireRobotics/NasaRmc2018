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
        registerDrivebaseJoint("left_tread_joint", Joint::LEFT_TREAD);
        registerDrivebaseJoint("right_tread_joint", Joint::RIGHT_TREAD);
        
        registerArmJoint("bin_joint", Actuator::BIN);
        registerArmJoint("turntable_joint", Actuator::TURNTABLE);
        registerArmJoint("lower_arm_joint", Actuator::LOWER_ARM);
        registerArmJoint("upper_arm_joint", Actuator::UPPER_ARM);
        registerArmJoint("scoop_joint", Actuator::SCOOP);

        registerInterface(&joint_state_interface);
        registerInterface(&joint_effort_interface);
    }

    // Return true if bin is finished moving to its new position.
    // TODO: A more explicit return type might be better here, or a possible redesign.
    //       If other values read end up needing to be returned, then this should 
    //       probably return some data structure. It may be that read values get published
    //       out on topics instead.
    bool Controller::read() 
    {
        // TODO: Waiting on further hardware development to implement

        // Return true if bin has finished moving to its new position.
        // For now, just return true so the bin controller doesn't block forever.
        return true;
    }

    void Controller::write() 
    {
        // TODO: Waiting on further hardware development to implement
        if (use_fake_values) 
        {
            // Update all actuators velocity with the command (effort in).
            // Then update the position as derivative of the velocity over time.
            for (int i = 0; i < ARM_CONTROLLER_COUNT; i++) 
            {
                arm_velocity_values[i] = arm_command_values[i];
                arm_position_values[i] += arm_velocity_values[i] * getUpdateTime();
                // If this joint has limits, clamp the range down
                if (abs(lower_limits[i]) >= 1E-3 || abs(upper_limits[i]) >= 1E-3) 
                {
                    arm_position_values[i] =
                        std::max(std::min(arm_position_values[i],
                                    upper_limits[i]), lower_limits[i]);
                }
            }
        }

        prev_time = ros::Time::now();
    }

    void Controller::registerDrivebaseJoint(std::string name, Joint joint) 
    {
        //TODO
    }

    void Controller::registerArmJoint(std::string name, Actuator actuator) 
    {
        auto idx = static_cast<int>(actuator);
        JointStateHandle state_handle(name, &arm_position_values[idx],
        &arm_velocity_values[idx], &arm_effort_values[idx]);
        joint_state_interface.registerHandle(state_handle);

        JointHandle handle(state_handle, &arm_command_values[idx]);
        joint_effort_interface.registerHandle(handle);
    }

    double Controller::getUpdateTime() 
    {
        return (ros::Time::now() - prev_time).nsec / 1E9;
    }
}
