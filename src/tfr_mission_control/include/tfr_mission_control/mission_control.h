#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <tfr_mission_control/ui_mission_control.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_srvs/SetBool.h>

#include <tfr_msgs/EmptySrv.h>
#include <tfr_msgs/SystemStatus.h>
#include <tfr_msgs/DurationSrv.h>
#include <tfr_msgs/EmptyAction.h>
#include <tfr_msgs/TeleopAction.h>

#include <tfr_utilities/teleop_code.h>
#include <tfr_utilities/status_code.h>


#include <cstdint>

#include <QWidget>
#include <QObject>
#include <QTimer>
#include <QScrollBar>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QKeyEvent>

namespace tfr_mission_control {

    /* Main entry point for the qt application, contains all of state, business
     * logic, and only has one screen. Uses the associated .ui file for layout
     * and design.
     *
     * Much of the code having to do with settings here has been generated from
     * the "catkin_create_rqt" package. We don't have any settings to implement,
     * but I am scared to delete it.
     * */
    class MissionControl : public rqt_gui_cpp::Plugin{

        Q_OBJECT

        public:

            /* ======================================================================== */
            /* Constructor/Destructor                                                   */
            /* ======================================================================== */

            MissionControl();
            ~MissionControl();

            /* ======================================================================== */
            /* Initialize/Shutdown                                                      */
            /* ======================================================================== */

            void initPlugin(qt_gui_cpp::PluginContext& context) override;

            void shutdownPlugin() override;

            /* ======================================================================== */
            /* Settings                                                                 */
            /* ======================================================================== */

            void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                    qt_gui_cpp::Settings& instance_settings) const override;

            void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                    const qt_gui_cpp::Settings& instance_settings) override;

        private:

            /* ======================================================================== */
            /* Constants                                                                */
            /* ======================================================================== */

            const std::string TAG = "MissionControl";
            //how long to wait without keyborad input before killing the motors(hz)
            //NOTE can cause bouncy keys if user has too long of a delay for
            //repeated keys
            const double MOTOR_INTERVAL = 1000/3;

            /* ======================================================================== */
            /* Variables                                                                */
            /* ======================================================================== */

            //the user interface
            Ui::MissionControlWidget ui;
            //Out application context
            QWidget* widget;
            //the mission timer
            QTimer* countdown;
            //The watchdog for the motors
            QTimer* motorKill;

            ros::NodeHandle nh;
            //The action servers
            actionlib::SimpleActionClient<tfr_msgs::EmptyAction> autonomy;
            actionlib::SimpleActionClient<tfr_msgs::TeleopAction> teleop;

            //our message subscriber
            ros::Subscriber com;

            //the last key we pressed (for debouncing)
            int lastKey;
            //Whether teleop commands should be accepted
            bool teleopEnabled;

            /* ======================================================================== */
            /* Methods                                                                  */
            /* ======================================================================== */
            //utility for shutting down motors and turntable
            void softwareStop();
            //sets system state to teleop ready
            void setTeleop(bool value);
            //sets system state to autonomy ready
            void setAutonomy(bool value);

            //sets control system to output commands
            void setControl(bool value);

            //sets hardware system to output commands
            void setMotors(bool value);




            /* ======================================================================== */
            /* Events                                                                   */
            /* ======================================================================== */
            //debounces and processes keyboard
            bool eventFilter(QObject *obj, QEvent *event);


            /* ======================================================================== */
            /* Callbacks                                                                */
            /* ======================================================================== */
            //triggered by incoming status message, and cascades other signals into thread
            //safe gui update
            void updateStatus(const tfr_msgs::SystemStatusConstPtr &status);

            protected slots:

                /* ======================================================================== */
                /* Slots                                                                    */
                /* ======================================================================== */

                //MISSION START
                virtual void startMission();    //starts clock and autonomy
                virtual void startManual();     //starts clock and teleop
                virtual void startTimeService();
                
                //MODE STATE CHANGES
                virtual void goAutonomousMode();
                virtual void goTeleopMode();

                //GUI
                virtual void renderClock();
                virtual void renderStatus();
    
                //MISC
                virtual void performTeleop(tfr_utilities::TeleopCode code);
                virtual void toggleControl(bool state);    //e-stop and start
                virtual void toggleMotors(bool state);    //e-stop and start
    
            signals:
                /* ======================================================================== */
                /* Signals                                                                  */
                /* ======================================================================== */
    
                //used to make cascade work for status update
                void emitStatus(QString status);
    };
} // namespace
