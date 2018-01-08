#include "controller.h"

namespace tfr_control
{
    Controller::Controller()
    {
        // Note: the string parameters in these constructors must match the joint names from the URDF. If one changes, so must the other.

        // Connect and register the joint state interfaces
        hardware_interface::JointStateHandle left_tread_state_handle("left_tread_joint", &position_values[0], &velocity_values[0], &effort_values[0]);
        joint_state_interface.registerHandle(left_tread_state_handle);

        hardware_interface::JointStateHandle right_tread_state_handle("right_tread_joint", &position_values[1], &velocity_values[1], &effort_values[1]);
        joint_state_interface.registerHandle(right_tread_state_handle);

        hardware_interface::JointStateHandle bin_state_handle("bin_joint", &position_values[2], &velocity_values[2], &effort_values[2]);
        joint_state_interface.registerHandle(bin_state_handle);

        hardware_interface::JointStateHandle turntable_state_handle("turntable_joint", &position_values[3], &velocity_values[3], &effort_values[3]);
        joint_state_interface.registerHandle(turntable_state_handle);

        hardware_interface::JointStateHandle lower_arm_state_handle("lower_arm_joint", &position_values[4], &velocity_values[4], &effort_values[4]);
        joint_state_interface.registerHandle(lower_arm_state_handle);

        hardware_interface::JointStateHandle upper_arm_state_handle("upper_arm_joint", &position_values[5], &velocity_values[5], &effort_values[5]);
        joint_state_interface.registerHandle(upper_arm_state_handle);

        hardware_interface::JointStateHandle scoop_state_handle("scoop_joint", &position_values[6], &velocity_values[6], &effort_values[6]);
        joint_state_interface.registerHandle(scoop_state_handle);

        registerInterface(&joint_state_interface);

        // Connect and register the joint effort interfaces
        hardware_interface::JointHandle left_tread_effort_values_handle(joint_state_interface.getHandle("left_tread_joint"), &command_values[0]);
        joint_effort_interface.registerHandle(left_tread_effort_values_handle);

        hardware_interface::JointHandle right_tread_effort_values_handle(joint_state_interface.getHandle("right_tread_joint"), &command_values[1]);
        joint_effort_interface.registerHandle(right_tread_effort_values_handle);

        hardware_interface::JointHandle bin_effort_values_handle(joint_state_interface.getHandle("bin_joint"), &command_values[2]);
        joint_effort_interface.registerHandle(bin_effort_values_handle);

        hardware_interface::JointHandle turntable_effort_values_handle(joint_state_interface.getHandle("turntable_joint"), &command_values[3]);
        joint_effort_interface.registerHandle(turntable_effort_values_handle);

        hardware_interface::JointHandle lower_arm_effort_values_handle(joint_state_interface.getHandle("lower_arm_joint"), &command_values[4]);
        joint_effort_interface.registerHandle(lower_arm_effort_values_handle);
        
        hardware_interface::JointHandle upper_arm_effort_values_handle(joint_state_interface.getHandle("upper_arm_joint"), &command_values[5]);
        joint_effort_interface.registerHandle(upper_arm_effort_values_handle);
        
        hardware_interface::JointHandle scoop_effort_values_handle(joint_state_interface.getHandle("scoop_joint"), &command_values[6]);
        joint_effort_interface.registerHandle(scoop_effort_values_handle);

        registerInterface(&joint_effort_interface);
    }

    void Controller::read() {
        // TODO: Waiting on further hardware development to implement
    }

    void Controller::write() {
        // TODO: Waiting on further hardware development to implement
    }
}