#ifndef AUBO_MOBILE_SORTING_SORTING_PANEL_H
#define AUBO_MOBILE_SORTING_SORTING_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>

#include <QString>

class QLabel;
class QPushButton;

namespace aubo_mobile_sorting
{

class SortingPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit SortingPanel(QWidget* parent = 0);

Q_SIGNALS:
  void stateReceived(const QString& text);
  void detectionsReceived(const QString& text);

private Q_SLOTS:
  void moveToObservation();
  void startSorting();
  void stopSorting();
  void openGripper();
  void moveHome();
  void showState(const QString& text);
  void showDetections(const QString& text);

private:
  void callTrigger(ros::ServiceClient& client, const QString& command_name);
  void stateCallback(const std_msgs::String::ConstPtr& message);
  void detectionsCallback(const std_msgs::String::ConstPtr& message);

  ros::NodeHandle node_handle_;
  ros::ServiceClient observation_client_;
  ros::ServiceClient start_client_;
  ros::ServiceClient stop_client_;
  ros::ServiceClient open_client_;
  ros::ServiceClient home_client_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber detections_subscriber_;

  QLabel* state_label_;
  QLabel* detections_label_;
  QLabel* command_label_;
};

}  // namespace aubo_mobile_sorting

#endif  // AUBO_MOBILE_SORTING_SORTING_PANEL_H
