#ifndef AUBO_ROS_CONTROL_VISUAL_SERVO_PANEL_H
#define AUBO_ROS_CONTROL_VISUAL_SERVO_PANEL_H

#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <QString>
#include <atomic>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QTimer;

namespace aubo_ros_control
{

class VisualServoPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit VisualServoPanel(QWidget* parent = 0);

Q_SIGNALS:
  void servoStateReceived(const QString& text);
  void perceptionStateReceived(const QString& text);
  void targetPoseReceived(const QString& text);
  void parametersChanged();

private Q_SLOTS:
  void startServo();
  void stopServo();
  void resetServo();
  void selectTarget(const QString& label);
  void showServoState(const QString& text);
  void showPerceptionState(const QString& text);
  void showTargetPose(const QString& text);
  void updateReadiness();
  void applyParameters();
  void refreshParameters();

private:
  bool setEnabled(ros::ServiceClient& client, bool enabled, QString* response);
  bool callReset(ros::ServiceClient& client, QString* response);
  void servoStateCallback(const std_msgs::String::ConstPtr& message);
  void perceptionStateCallback(const std_msgs::String::ConstPtr& message);
  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message);
  void planningSceneReadyCallback(const std_msgs::Bool::ConstPtr& message);
  void parameterUpdateCallback(const dynamic_reconfigure::Config::ConstPtr& message);

  ros::NodeHandle node_handle_;
  ros::ServiceClient servo_enable_client_;
  ros::ServiceClient perception_enable_client_;
  ros::ServiceClient servo_reset_client_;
  ros::ServiceClient perception_reset_client_;
  ros::ServiceClient planning_client_;
  ros::ServiceClient reconfigure_client_;
  ros::Publisher target_selection_publisher_;
  ros::Subscriber servo_state_subscriber_;
  ros::Subscriber perception_state_subscriber_;
  ros::Subscriber target_pose_subscriber_;
  ros::Subscriber planning_scene_ready_subscriber_;
  ros::Subscriber parameter_subscriber_;

  QComboBox* target_combo_;
  QLabel* servo_state_label_;
  QLabel* perception_state_label_;
  QLabel* target_pose_label_;
  QLabel* phase_label_;
  QLabel* readiness_label_;
  QLabel* command_label_;
  QPushButton* start_button_;
  QPushButton* stop_button_;
  QPushButton* reset_button_;
  QLabel* parameter_label_;
  QDoubleSpinBox* linear_gain_;
  QDoubleSpinBox* angular_gain_;
  QDoubleSpinBox* max_linear_velocity_;
  QDoubleSpinBox* max_angular_velocity_;
  QDoubleSpinBox* position_deadband_;
  QDoubleSpinBox* orientation_deadband_;
  QDoubleSpinBox* target_timeout_;
  QComboBox* loss_strategy_;
  QTimer* readiness_timer_;
  bool flow_active_{false};
  std::atomic<bool> planning_scene_ready_{false};
};

}  // namespace aubo_ros_control

#endif  // AUBO_ROS_CONTROL_VISUAL_SERVO_PANEL_H
