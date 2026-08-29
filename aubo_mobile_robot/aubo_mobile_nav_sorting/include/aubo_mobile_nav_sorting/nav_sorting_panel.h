#ifndef AUBO_MOBILE_NAV_SORTING_NAV_SORTING_PANEL_H
#define AUBO_MOBILE_NAV_SORTING_NAV_SORTING_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>

#include <QString>

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSpinBox;

namespace aubo_mobile_nav_sorting
{

class NavSortingPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit NavSortingPanel(QWidget* parent = 0);

Q_SIGNALS:
  void missionStateReceived(const QString& text);
  void sortingStateReceived(const QString& text);

private Q_SLOTS:
  void startMission();
  void stopMission();
  void applyParameters();
  void refreshParameters();
  void showMissionState(const QString& text);
  void showSortingState(const QString& text);

private:
  void callTrigger(ros::ServiceClient& client, const QString& command_name);
  void missionStateCallback(const std_msgs::String::ConstPtr& message);
  void sortingStateCallback(const std_msgs::String::ConstPtr& message);

  ros::NodeHandle node_handle_;
  ros::ServiceClient start_client_;
  ros::ServiceClient stop_client_;
  ros::ServiceClient reconfigure_client_;
  ros::Subscriber mission_state_subscriber_;
  ros::Subscriber sorting_state_subscriber_;

  QLabel* mission_state_label_;
  QLabel* sorting_state_label_;
  QLabel* command_label_;
  QPushButton* start_button_;
  QPushButton* stop_button_;
  QDoubleSpinBox* goal_x_;
  QDoubleSpinBox* goal_y_;
  QDoubleSpinBox* goal_yaw_;
  QCheckBox* near_field_enabled_;
  QDoubleSpinBox* pre_dock_x_;
  QDoubleSpinBox* pre_dock_y_;
  QDoubleSpinBox* pre_dock_yaw_;
  QDoubleSpinBox* base_clearance_;
  QDoubleSpinBox* navigation_timeout_;
  QSpinBox* navigation_retries_;
  bool mission_busy_;
};

}  // namespace aubo_mobile_nav_sorting

#endif  // AUBO_MOBILE_NAV_SORTING_NAV_SORTING_PANEL_H
