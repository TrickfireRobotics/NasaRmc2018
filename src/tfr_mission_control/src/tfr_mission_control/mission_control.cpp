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
      teleop{"teleop_action_server",true},
      com{nh.subscribe("com", 5, &MissionControl::renderStatus, this)}
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

  //set up button SIGNAL->SLOT connections
  connect(ui.start_button, &QPushButton::pressed,this, &MissionControl::startMission);
  connect(ui.clock_button, &QPushButton::pressed,this, &MissionControl::startManual);
  connect(ui.autonomy_button, &QPushButton::pressed, this,  &MissionControl::startAutonomy);
  connect(ui.teleop_button, &QPushButton::pressed, this,  &MissionControl::startTeleop);

  connect(countdown, &QTimer::timeout, this,  &MissionControl::renderClock);

  /* NOTE using lambdas here helps us avoid writing a million different parameterless
   * wrapper functions we can just pass in a constant to perform teleop for each one
   * It wraps them in a free function, so no 'this' implicit parameter  passing needed,
   * ain't qt5 great?*/
  connect(ui.reset_starting_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::RESET_STARTING);});
  connect(ui.reset_dumping_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::RESET_DUMPING);});
  connect(ui.dump_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::DUMP);});
  connect(ui.dig_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::DIG);});
  connect(ui.cw_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::CLOCKWISE);});
  connect(ui.ccw_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::COUNTERCLOCKWISE);});
  connect(ui.forward_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::FORWARD);});
  connect(ui.forward_button,&QPushButton::released,
          [this] () {performTeleop(tfr_utilities::TeleopCode::STOP);});
  connect(ui.backward_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::BACKWARD);});
  connect(ui.backward_button,&QPushButton::released,
          [this] () {performTeleop(tfr_utilities::TeleopCode::STOP);});
  connect(ui.left_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::LEFT);});
  connect(ui.left_button,&QPushButton::released,
          [this] () {performTeleop(tfr_utilities::TeleopCode::STOP);});
  connect(ui.right_button,&QPushButton::pressed,
          [this] () {performTeleop(tfr_utilities::TeleopCode::RIGHT);});
  connect(ui.right_button,&QPushButton::released,
          [this] () {performTeleop(tfr_utilities::TeleopCode::STOP);});
  ROS_INFO("Mission Control: connecting autonomy");
  autonomy.waitForServer();
  ROS_INFO("Mission Control: connected autonomy");
  ROS_INFO("Mission Control: connecting teleop");
  teleop.waitForServer();
  ROS_INFO("Mission Control: connected teleop");
}

void MissionControl::shutdownPlugin()
{
    //note because qt plugins are weird we need to manually kill ros entities
    com.shutdown();
    autonomy.cancelAllGoals();
    autonomy.stopTrackingGoal();
    teleop.cancelAllGoals();
    teleop.stopTrackingGoal();

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
    ui.reset_starting_button->setEnabled(value);
    ui.reset_dumping_button->setEnabled(value);
    ui.autonomy_button->setEnabled(value);
    ui.dump_button->setEnabled(value);
    ui.dig_button->setEnabled(value);
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
void MissionControl::renderStatus(const tfr_msgs::SystemStatusConstPtr &status)
{
    auto str = getStatusMessage(static_cast<StatusCode>(status->status_code), status->data);
    QString msg = QString::fromStdString(str);
    ui.status_log->appendPlainText(msg);
    ui.status_log->verticalScrollBar()->setValue(ui.status_log->verticalScrollBar()->maximum());
}

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

void MissionControl::performTeleop(tfr_utilities::TeleopCode code)
{
    tfr_msgs::TeleopGoal goal;
    goal.code = static_cast<uint8_t>(code);
    teleop.sendGoal(goal);
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
