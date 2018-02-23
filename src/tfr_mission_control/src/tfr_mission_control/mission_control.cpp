#include "tfr_mission_control/mission_control.h"

#include <pluginlib/class_list_macros.h>

namespace tfr_mission_control {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

MissionControl::MissionControl()
    : rqt_gui_cpp::Plugin(),
      widget(nullptr),
      autonomy{"autonomous_action_server",true},
      teleop{"teleop_action_server",true}
{

  setObjectName("MissionControl");
}

MissionControl::~MissionControl()
{
    delete countdown;
}

/* ========================================================================== */
/* Initialize/Shutdown                                                        */
/* ========================================================================== */

void MissionControl::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget = new QWidget();
  ui.setupUi(widget);
  if (context.serialNumber() > 1)
  {
    widget->setWindowTitle(widget->windowTitle() +
        " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget);

  countdown = new QTimer(this);




  //set up plugin here
  connect(ui.start_button, SIGNAL(pressed()),this, SLOT(startMission()));
  connect(ui.clock_button, SIGNAL(pressed()),this, SLOT(startManual()));
  connect(ui.autonomy_button, SIGNAL(pressed()), this, SLOT(startAutonomy()));
  connect(ui.teleop_button, SIGNAL(pressed()), this, SLOT(startTeleop()));
  connect(countdown,SIGNAL(timeout()), this, SLOT(renderClock()));
  ROS_INFO("Mission Control: connecting autonomy");
  autonomy.waitForServer();
  ROS_INFO("Mission Control: connected autonomy");
  ROS_INFO("Mission Control: connecting teleop");
  teleop.waitForServer();
  ROS_INFO("Mission Control: connected teleop");
}

void MissionControl::shutdownPlugin()
{
}

/* ========================================================================== */
/* Settings                                                                   */
/* ========================================================================== */

void MissionControl::saveSettings(
    qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
}

void MissionControl::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */
void MissionControl::setTeleopButtons(bool value)
{
    ui.left_button->setEnabled(value);
    ui.right_button->setEnabled(value);
    ui.forward_button->setEnabled(value);
    ui.backward_button->setEnabled(value);
    ui.cw_button->setEnabled(value);
    ui.ccw_button->setEnabled(value);
    ui.reset_start_button->setEnabled(value);
    ui.reset_dump_button->setEnabled(value);
    ui.autonomy_button->setEnabled(value);
    ui.dump_button->setEnabled(value);

}


void MissionControl::setAutonomyButtons(bool value)
{
    ui.teleop_button->setEnabled(value);
}


/* ========================================================================== */
/* Events                                                                     */
/* ========================================================================== */



/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */



/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void MissionControl::startTimeService()
{
    tfr_msgs::EmptySrv start;
    ros::service::call("start_mission", start);
    countdown->start(500);
}

void MissionControl::startMission()
{
    startTimeService();
    startAutonomy();
}
void MissionControl::startManual()
{
    startTimeService();
    startTeleop();
}

void MissionControl::startAutonomy()
{
    setAutonomyButtons(true);
    tfr_msgs::EmptyGoal goal{};
    while (!teleop.getState().isDone()) teleop.cancelAllGoals();
    autonomy.sendGoal(goal);
    setTeleopButtons(false);
}

void MissionControl::startTeleop()
{
    setAutonomyButtons(false);
    while (!autonomy.getState().isDone()) autonomy.cancelAllGoals();
    setTeleopButtons(true);
}

void MissionControl::renderClock()
{
    tfr_msgs::DurationSrv remaining_time;
    ros::service::call("time_remaining", remaining_time);
    ui.time_display->display(remaining_time.response.duration.toSec());
}


/* ========================================================================== */
/* Signals                                                                    */
/* ========================================================================== */


} // namespace

PLUGINLIB_EXPORT_CLASS(tfr_mission_control::MissionControl,
                       rqt_gui_cpp::Plugin)
