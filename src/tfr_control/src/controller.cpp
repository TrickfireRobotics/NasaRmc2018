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
    Controller::Controller(bool fakes) : use_fake_values(fakes)
    {
        // Note: the string parameters in these constructors must match the
        // joint names from the URDF. If one changes, so must the other.

        // Connect and register the joint state and effort interfaces
        register_joint("left_tread_joint", Actuator::kLeftTread);
        register_joint("right_tread_joint", Actuator::kRightTread);
        register_joint("bin_joint", Actuator::kBin);
        register_joint("turntable_joint", Actuator::kTurntable);
        register_joint("lower_arm_joint", Actuator::kLowerArm);
        register_joint("upper_arm_joint", Actuator::kUpperArm);
        register_joint("scoop_joint", Actuator::kScoop);

        registerInterface(&joint_state_interface);
        registerInterface(&joint_effort_interface);
    }

    void Controller::read() {
        // TODO: Waiting on further hardware development to implement
    }

    void Controller::write() {
        // TODO: Waiting on further hardware development to implement
        if (use_fake_values) {
            // Update all actuators velocity with the command (effort in).
            // Then update the position as derivative of the velocity over time.
            for (int i = 0; i < 7; i++) {
                velocity_values[i] = command_values[i];
                position_values[i] += velocity_values[i] * get_update_time();
            }
        }

        prev_time = ros::Time::now();
    }
    
    void Controller::register_joint(std::string joint, Actuator actuator) {
        JointStateHandle state_handle(joint, &position_values[actuator],
        &velocity_values[actuator], &effort_values[actuator]);
        joint_state_interface.registerHandle(state_handle);

        JointHandle handle(state_handle, &command_values[actuator]);
        joint_effort_interface.registerHandle(handle);
    }

    double Controller::get_update_time() {
        return (ros::Time::now() - prev_time).nsec / 1E9;
    }
}
