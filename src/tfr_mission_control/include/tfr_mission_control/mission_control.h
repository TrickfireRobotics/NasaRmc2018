#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <tfr_mission_control/ui_mission_control.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

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
  const double MOTOR_INTERVAL = 1000/3;

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::MissionControlWidget ui;
  QWidget* widget;
  QTimer* countdown;
  QTimer* motorKill;

  ros::NodeHandle nh;
  actionlib::SimpleActionClient<tfr_msgs::EmptyAction> autonomy;
  actionlib::SimpleActionClient<tfr_msgs::TeleopAction> teleop;
  ros::Subscriber com;
  int lastKey;

  bool teleopEnabled;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */
  void softwareStop();
  void setTeleop(bool value);
  void setAutonomy(bool value);



  /* ======================================================================== */
  /* Events                                                                   */
  /* ======================================================================== */
  bool eventFilter(QObject *obj, QEvent *event);


  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */
  void updateStatus(const tfr_msgs::SystemStatusConstPtr &status);




protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */


  virtual void startMission();
  virtual void startManual();

  virtual void goAutonomousMode();
  virtual void goTeleopMode();

  virtual void performTeleop(tfr_utilities::TeleopCode code);
  virtual void toggleMotors();
  virtual void stopDrivebase();

  virtual void renderClock();
  virtual void renderStatus();
  virtual void startTimeService();


signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */
  void emitStatus(QString status);



};

} // namespace
