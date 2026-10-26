#ifndef AUBO_HAND_EYE_VALIDATION_HAND_EYE_VERIFY_PANEL_H
#define AUBO_HAND_EYE_VALIDATION_HAND_EYE_VERIFY_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <QString>

class QLabel;
class QPushButton;

namespace aubo_handeye_validation
{

class HandeyeVerifyPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit HandeyeVerifyPanel(QWidget* parent = nullptr);

Q_SIGNALS:
  void statusReceived(const QString& status);

private Q_SLOTS:
  void plan();
  void moveToHover();
  void stop();
  void updateStatus(const QString& status);
  void serviceFinished(const QString& action, bool success, const QString& message);

private:
  void callService(const QString& action, const QString& service, bool motion_request);
  void statusCallback(const std_msgs::String::ConstPtr& message);

  ros::NodeHandle node_handle_;
  ros::Subscriber status_subscriber_;
  QLabel* status_label_;
  QPushButton* plan_button_;
  QPushButton* move_button_;
  QPushButton* stop_button_;
  bool motion_request_active_{false};
};

}  // namespace aubo_handeye_validation

#endif  // AUBO_HAND_EYE_VALIDATION_HAND_EYE_VERIFY_PANEL_H
