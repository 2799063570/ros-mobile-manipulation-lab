#ifndef AUBO_ROS_CONTROL_VISUAL_SERVO_PANEL_H
#define AUBO_ROS_CONTROL_VISUAL_SERVO_PANEL_H

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <QString>

class QComboBox;
class QLabel;

namespace aubo_ros_control
{

class VisualServoPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit VisualServoPanel(QWidget* parent = 0);

Q_SIGNALS:
  void alignedReceived(bool aligned);
  void servoStateReceived(const QString& text);
  void perceptionStateReceived(const QString& text);
  void targetPoseReceived(const QString& text);

private Q_SLOTS:
  void startServo();
  void stopServo();
  void resetServo();
  void selectTarget(const QString& label);
  void showAligned(bool aligned);
  void showServoState(const QString& text);
  void showPerceptionState(const QString& text);
  void showTargetPose(const QString& text);

private:
  bool setEnabled(ros::ServiceClient& client, bool enabled, QString* response);
  bool callReset(ros::ServiceClient& client, QString* response);
  void alignedCallback(const std_msgs::Bool::ConstPtr& message);
  void servoStateCallback(const std_msgs::String::ConstPtr& message);
  void perceptionStateCallback(const std_msgs::String::ConstPtr& message);
  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message);

  ros::NodeHandle node_handle_;
  ros::ServiceClient servo_enable_client_;
  ros::ServiceClient perception_enable_client_;
  ros::ServiceClient servo_reset_client_;
  ros::ServiceClient perception_reset_client_;
  ros::Publisher target_selection_publisher_;
  ros::Subscriber servo_state_subscriber_;
  ros::Subscriber aligned_subscriber_;
  ros::Subscriber perception_state_subscriber_;
  ros::Subscriber target_pose_subscriber_;

  QComboBox* target_combo_;
  QLabel* servo_state_label_;
  QLabel* aligned_label_;
  QLabel* perception_state_label_;
  QLabel* target_pose_label_;
  QLabel* command_label_;
};

}  // namespace aubo_ros_control

#endif  // AUBO_ROS_CONTROL_VISUAL_SERVO_PANEL_H
