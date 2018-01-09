/**
 * controller.h
 * 
 * This class is in charge of handling the physical hardware interface with
 * the robot itself, and is started by the controller_launcher node.
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace tfr_control {
    enum Actuator { kLeftTread, kRightTread, kBin, kTurntable, kLowerArm,
                    kUpperArm, kScoop };

    /**
     * The class registered with controller_manager that handles the interface
     * with all hardware components on the robot. See controller_launcher.cpp
     * for the usage of this class.
     * */
    class Controller : public hardware_interface::RobotHW
    {
    public:
        Controller();
        void read();
        void write();
    private:
        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::EffortJointInterface joint_effort_interface;
        
        /**
         * Seven actuators/motors to store data for:
         *  - Left + right tread
         *  - Bin actuator
         *  - Turntable
         *  - Lower arm actuator
         *  - Upper arm actuator
         *  - Scoop actuator
         * */
        void register_joint(std::string joint, Actuator actuator);

        // Populated by controller_manager, read in by the write() method
        double command_values[7];
        // Populated by sensors, read in by controller_manager
        double position_values[7];
        // Populated by sensors, read in by controller_manager
        double velocity_values[7];
        // Populated by sensors, read in by controller_manager
        double effort_values[7];
    };
}

#endif // CONTROLLER_H
