#include <aubo_ros_control/visual_servo_panel.h>

#include <pluginlib/class_list_macros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <QComboBox>
#include <QFont>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace aubo_ros_control
{

VisualServoPanel::VisualServoPanel(QWidget* parent)
  : rviz::Panel(parent)
  , target_combo_(new QComboBox())
  , servo_state_label_(new QLabel(tr("等待控制器...")))
  , perception_state_label_(new QLabel(tr("等待视觉识别...")))
  , target_pose_label_(new QLabel(tr("尚无有效三维目标")))
  , command_label_(new QLabel(tr("请选择目标，然后启动视觉伺服")))
{
  QLabel* title = new QLabel(tr("AUBO 统一视觉伺服"));
  QFont title_font = title->font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 2);
  title->setFont(title_font);

  target_combo_->addItem(tr("红色目标"), "red");
  target_combo_->addItem(tr("绿色目标"), "green");
  target_combo_->addItem(tr("蓝色目标"), "blue");
  target_combo_->addItem(tr("任意已配置目标"), "any");

  QPushButton* start_button = new QPushButton(tr("启动闭环跟踪"));
  QPushButton* stop_button = new QPushButton(tr("停止并保持"));
  QPushButton* reset_button = new QPushButton(tr("清除目标 / 重新搜索"));
  start_button->setStyleSheet(
      "font-weight: bold; color: white; background-color: #2d8a45;");
  stop_button->setStyleSheet(
      "font-weight: bold; color: white; background-color: #b33a3a;");

  servo_state_label_->setWordWrap(true);
  perception_state_label_->setWordWrap(true);
  target_pose_label_->setWordWrap(true);
  command_label_->setWordWrap(true);

  QGridLayout* buttons = new QGridLayout();
  buttons->addWidget(start_button, 0, 0);
  buttons->addWidget(stop_button, 0, 1);
  buttons->addWidget(reset_button, 1, 0, 1, 2);

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(title);
  layout->addWidget(new QLabel(tr("跟踪目标：")));
  layout->addWidget(target_combo_);
  layout->addWidget(new QLabel(tr("伺服状态：")));
  layout->addWidget(servo_state_label_);
  layout->addWidget(new QLabel(tr("识别状态：")));
  layout->addWidget(perception_state_label_);
  layout->addWidget(new QLabel(tr("相机系目标位置：")));
  layout->addWidget(target_pose_label_);
  layout->addLayout(buttons);
  layout->addWidget(command_label_);
  layout->addStretch();
  setLayout(layout);

  servo_enable_client_ = node_handle_.serviceClient<std_srvs::SetBool>(
      "/visual_servo/set_enabled");
  perception_enable_client_ = node_handle_.serviceClient<std_srvs::SetBool>(
      "/visual_servo/perception/set_enabled");
  servo_reset_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
      "/visual_servo/reset");
  perception_reset_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
      "/visual_servo/perception/reset");
  target_selection_publisher_ = node_handle_.advertise<std_msgs::String>(
      "/visual_servo/target_selection", 1, true);
  servo_state_subscriber_ = node_handle_.subscribe(
      "/visual_servo/state", 1, &VisualServoPanel::servoStateCallback, this);
  perception_state_subscriber_ = node_handle_.subscribe(
      "/visual_servo/perception_state", 1,
      &VisualServoPanel::perceptionStateCallback, this);
  target_pose_subscriber_ = node_handle_.subscribe(
      "/visual_servo/target_pose", 1, &VisualServoPanel::targetPoseCallback, this);

  connect(start_button, SIGNAL(clicked()), this, SLOT(startServo()));
  connect(stop_button, SIGNAL(clicked()), this, SLOT(stopServo()));
  connect(reset_button, SIGNAL(clicked()), this, SLOT(resetServo()));
  connect(target_combo_, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(selectTarget(QString)));
  connect(this, SIGNAL(servoStateReceived(QString)),
          this, SLOT(showServoState(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(perceptionStateReceived(QString)),
          this, SLOT(showPerceptionState(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(targetPoseReceived(QString)),
          this, SLOT(showTargetPose(QString)), Qt::QueuedConnection);

  selectTarget(target_combo_->currentText());
}

bool VisualServoPanel::setEnabled(ros::ServiceClient& client, bool enabled,
                                  QString* response)
{
  std_srvs::SetBool service;
  service.request.data = enabled;
  if (!client.call(service))
  {
    *response = tr("服务不可用");
    return false;
  }
  *response = QString::fromStdString(service.response.message);
  return service.response.success;
}

bool VisualServoPanel::callReset(ros::ServiceClient& client, QString* response)
{
  std_srvs::Trigger service;
  if (!client.call(service))
  {
    *response = tr("服务不可用");
    return false;
  }
  *response = QString::fromStdString(service.response.message);
  return service.response.success;
}

void VisualServoPanel::startServo()
{
  selectTarget(target_combo_->currentText());
  QString perception_response;
  if (!setEnabled(perception_enable_client_, true, &perception_response))
  {
    command_label_->setText(tr("视觉识别启动失败：") + perception_response);
    return;
  }
  QString servo_response;
  if (!setEnabled(servo_enable_client_, true, &servo_response))
  {
    QString unused;
    setEnabled(perception_enable_client_, false, &unused);
    command_label_->setText(tr("控制器启动失败：") + servo_response);
    return;
  }
  command_label_->setText(tr("闭环已启动，等待所选目标进入相机视野"));
}

void VisualServoPanel::stopServo()
{
  QString servo_response;
  const bool servo_ok = setEnabled(servo_enable_client_, false, &servo_response);
  QString perception_response;
  const bool perception_ok =
      setEnabled(perception_enable_client_, false, &perception_response);
  command_label_->setText(
      (servo_ok && perception_ok) ? tr("已停止：机械臂减速保持，识别输出已关闭")
                                  : tr("停止未完全执行：") + servo_response + " / " + perception_response);
}

void VisualServoPanel::resetServo()
{
  QString servo_response;
  QString perception_response;
  const bool servo_ok = callReset(servo_reset_client_, &servo_response);
  const bool perception_ok = callReset(perception_reset_client_, &perception_response);
  command_label_->setText(
      (servo_ok && perception_ok) ? tr("历史目标已清除，将等待新的图像观测")
                                  : tr("复位未完全执行：") + servo_response + " / " + perception_response);
}

void VisualServoPanel::selectTarget(const QString&)
{
  std_msgs::String message;
  message.data = target_combo_->currentData().toString().toStdString();
  target_selection_publisher_.publish(message);
  command_label_->setText(tr("目标已切换为：") + target_combo_->currentText());
}

void VisualServoPanel::servoStateCallback(const std_msgs::String::ConstPtr& message)
{
  Q_EMIT servoStateReceived(QString::fromStdString(message->data));
}

void VisualServoPanel::perceptionStateCallback(const std_msgs::String::ConstPtr& message)
{
  Q_EMIT perceptionStateReceived(QString::fromStdString(message->data));
}

void VisualServoPanel::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
{
  const QString text = QString("x=%1 m  y=%2 m  z=%3 m\nframe: %4")
      .arg(message->pose.position.x, 0, 'f', 3)
      .arg(message->pose.position.y, 0, 'f', 3)
      .arg(message->pose.position.z, 0, 'f', 3)
      .arg(QString::fromStdString(message->header.frame_id));
  Q_EMIT targetPoseReceived(text);
}

void VisualServoPanel::showServoState(const QString& text)
{
  QString translated = text;
  if (text == "DISABLED") translated = tr("已停用 / 保持当前位置");
  else if (text == "WAITING") translated = tr("等待新目标");
  else if (text == "SEARCH_INITIAL") translated = tr("初始化 / 移向可观测姿态");
  else if (text == "TRACKING") translated = tr("正在闭环跟踪");
  else if (text == "ALIGNED") translated = tr("已到达 / 稳定对齐");
  else if (text == "COAST") translated = tr("目标短暂丢失 / 减速滑行");
  else if (text == "SEARCH_RECOVERY") translated = tr("目标丢失 / 重新搜索");
  else if (text == "HOLD") translated = tr("搜索超时 / 保持");
  servo_state_label_->setText(translated);
}

void VisualServoPanel::showPerceptionState(const QString& text)
{
  const QString state = text.section('|', 0, 0);
  const QString label = text.section('|', 1, 1);
  const QString detail = text.section('|', 2);
  QString translated = state;
  if (state == "DISABLED") translated = tr("已停用");
  else if (state == "SEARCHING") translated = tr("正在搜索");
  else if (state == "DETECTED") translated = tr("已识别");
  perception_state_label_->setText(
      translated + tr(" | 目标：") + label +
      (detail.isEmpty() ? QString() : QString(" | ") + detail));
}

void VisualServoPanel::showTargetPose(const QString& text)
{
  target_pose_label_->setText(text);
}

}  // namespace aubo_ros_control

PLUGINLIB_EXPORT_CLASS(aubo_ros_control::VisualServoPanel, rviz::Panel)
