#include <aubo_ros_control/visual_servo_panel.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <QComboBox>
#include <QFont>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace aubo_ros_control
{

VisualServoPanel::VisualServoPanel(QWidget* parent)
  : rviz::Panel(parent)
  , target_combo_(new QComboBox())
  , servo_state_label_(new QLabel(tr("等待控制器...")))
  , perception_state_label_(new QLabel(tr("等待视觉识别...")))
  , target_pose_label_(new QLabel(tr("尚无有效三维目标")))
  , phase_label_(new QLabel(tr("未启动")))
  , readiness_label_(new QLabel(tr("正在检查节点与服务...")))
  , command_label_(new QLabel(tr("请选择目标，然后启动两阶段流程")))
  , start_button_(new QPushButton(tr("启动两阶段流程")))
  , stop_button_(new QPushButton(tr("停止并保持")))
  , reset_button_(new QPushButton(tr("复位流程")))
  , readiness_timer_(new QTimer(this))
{
  QLabel* title = new QLabel(tr("AUBO 路径规划 + 位置伺服"));
  QFont title_font = title->font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 2);
  title->setFont(title_font);

  target_combo_->addItem(tr("红色目标"), "red");
  target_combo_->addItem(tr("绿色目标"), "green");
  target_combo_->addItem(tr("蓝色目标"), "blue");
  target_combo_->addItem(tr("任意已配置目标"), "any");

  QLabel* flow = new QLabel(tr("准备：眼在手上先移动到观察位\n"
                               "阶段 1：MoveIt 路径规划与接近\n"
                               "阶段 2：视觉位置伺服与对齐"));
  flow->setStyleSheet("color: #666666;");
  phase_label_->setAlignment(Qt::AlignCenter);
  phase_label_->setStyleSheet(
      "font-weight: bold; padding: 8px; border-radius: 3px; "
      "background-color: #777777; color: white;");
  readiness_label_->setWordWrap(true);

  start_button_->setStyleSheet(
      "font-weight: bold; color: white; background-color: #2d8a45;");
  stop_button_->setStyleSheet(
      "font-weight: bold; color: white; background-color: #b33a3a;");

  servo_state_label_->setWordWrap(true);
  perception_state_label_->setWordWrap(true);
  target_pose_label_->setWordWrap(true);
  command_label_->setWordWrap(true);

  QGridLayout* buttons = new QGridLayout();
  buttons->addWidget(start_button_, 0, 0);
  buttons->addWidget(stop_button_, 0, 1);
  buttons->addWidget(reset_button_, 1, 0, 1, 2);

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(title);
  layout->addWidget(flow);
  layout->addWidget(phase_label_);
  layout->addWidget(new QLabel(tr("系统就绪状态：")));
  layout->addWidget(readiness_label_);
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
  planning_client_ = node_handle_.serviceClient<moveit_msgs::GetMotionPlan>(
      "/plan_kinematic_path");
  target_selection_publisher_ = node_handle_.advertise<std_msgs::String>(
      "/visual_servo/target_selection", 1, true);
  servo_state_subscriber_ = node_handle_.subscribe(
      "/visual_servo/state", 1, &VisualServoPanel::servoStateCallback, this);
  perception_state_subscriber_ = node_handle_.subscribe(
      "/visual_servo/perception_state", 1,
      &VisualServoPanel::perceptionStateCallback, this);
  target_pose_subscriber_ = node_handle_.subscribe(
      "/visual_servo/target_pose", 1, &VisualServoPanel::targetPoseCallback, this);
  planning_scene_ready_subscriber_ = node_handle_.subscribe(
      "/hybrid/planning_scene_ready", 1,
      &VisualServoPanel::planningSceneReadyCallback, this);

  connect(start_button_, SIGNAL(clicked()), this, SLOT(startServo()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopServo()));
  connect(reset_button_, SIGNAL(clicked()), this, SLOT(resetServo()));
  connect(target_combo_, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(selectTarget(QString)));
  connect(this, SIGNAL(servoStateReceived(QString)),
          this, SLOT(showServoState(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(perceptionStateReceived(QString)),
          this, SLOT(showPerceptionState(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(targetPoseReceived(QString)),
          this, SLOT(showTargetPose(QString)), Qt::QueuedConnection);
  connect(readiness_timer_, SIGNAL(timeout()), this, SLOT(updateReadiness()));

  selectTarget(target_combo_->currentText());
  readiness_timer_->start(500);
  updateReadiness();
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
  command_label_->setText(
      tr("流程已启动：发现目标后先规划接近，再自动切换到位置伺服"));
}

void VisualServoPanel::stopServo()
{
  QString servo_response;
  const bool servo_ok = setEnabled(servo_enable_client_, false, &servo_response);
  QString perception_response;
  const bool perception_ok =
      setEnabled(perception_enable_client_, false, &perception_response);
  command_label_->setText(
      (servo_ok && perception_ok) ? tr("已停止：机械臂保持，识别输出已关闭")
                                  : tr("停止未完全执行：") + servo_response + " / " + perception_response);
}

void VisualServoPanel::resetServo()
{
  QString servo_response;
  QString perception_response;
  const bool servo_ok = callReset(servo_reset_client_, &servo_response);
  const bool perception_ok = callReset(perception_reset_client_, &perception_response);
  command_label_->setText(
      (servo_ok && perception_ok) ? tr("流程已复位，将等待新的图像观测")
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

void VisualServoPanel::planningSceneReadyCallback(
    const std_msgs::Bool::ConstPtr& message)
{
  planning_scene_ready_.store(message->data);
}

void VisualServoPanel::showServoState(const QString& text)
{
  QString translated = text;
  QString phase = tr("状态未知");
  QString color = "#777777";
  if (text == "DISABLED") {
    translated = tr("已停用 / 保持当前位置");
    phase = tr("未启动");
  } else if (text == "WAITING") {
    translated = tr("等待有效目标和稳定关节反馈");
    phase = tr("准备阶段");
    color = "#b07d18";
  } else if (text == "PLANNING") {
    translated = tr("正在请求 MoveIt 规划接近轨迹");
    phase = tr("阶段 1 / 路径规划");
    color = "#3569a8";
  } else if (text == "APPROACH") {
    translated = tr("正在执行规划轨迹，接近视觉伺服范围");
    phase = tr("阶段 1 / 轨迹接近");
    color = "#3569a8";
  } else if (text == "SEARCH_INITIAL") {
    translated = tr("正在移动到腕部相机观察位");
    phase = tr("准备 / 移动到观察位");
    color = "#00838f";
  } else if (text == "TRACKING") {
    translated = tr("正在进行近距离视觉位置伺服");
    phase = tr("阶段 2 / 位置伺服");
    color = "#7a4ca3";
  } else if (text == "ALIGNED") {
    translated = tr("目标已稳定对齐，可以执行后续抓取");
    phase = tr("流程完成 / 已对齐");
    color = "#2d8a45";
  } else if (text == "COAST") {
    translated = tr("目标短暂丢失 / 减速滑行");
    phase = tr("安全过渡");
    color = "#b07d18";
  } else if (text == "SEARCH_RECOVERY") {
    translated = tr("目标丢失 / 重新搜索");
    phase = tr("恢复阶段");
    color = "#b07d18";
  } else if (text == "HOLD") {
    translated = tr("故障或超时，机械臂保持；排查后点击复位");
    phase = tr("安全保持 / 需要复位");
    color = "#b33a3a";
  }
  servo_state_label_->setText(translated);
  phase_label_->setText(phase);
  phase_label_->setStyleSheet(
      QString("font-weight: bold; padding: 8px; border-radius: 3px; "
              "background-color: %1; color: white;").arg(color));
  flow_active_ = text != "DISABLED" && text != "HOLD";
  start_button_->setEnabled(!flow_active_);
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

void VisualServoPanel::updateReadiness()
{
  const bool planning_ready = planning_client_.exists();
  const bool scene_ready = planning_scene_ready_.load();
  const bool servo_ready = servo_enable_client_.exists();
  const bool perception_ready = perception_enable_client_.exists();
  readiness_label_->setText(
      QString(tr("规划 %1  |  碰撞场景 %2  |  控制 %3  |  感知 %4"))
          .arg(planning_ready ? tr("就绪") : tr("未就绪"))
          .arg(scene_ready ? tr("就绪") : tr("未就绪"))
          .arg(servo_ready ? tr("就绪") : tr("未就绪"))
          .arg(perception_ready ? tr("就绪") : tr("未就绪")));
  readiness_label_->setStyleSheet(
      planning_ready && scene_ready && servo_ready && perception_ready
          ? "color: #2d8a45; font-weight: bold;"
          : "color: #b33a3a; font-weight: bold;");
  start_button_->setEnabled(
      planning_ready && scene_ready && servo_ready && perception_ready &&
      !flow_active_);
}

}  // namespace aubo_ros_control

PLUGINLIB_EXPORT_CLASS(aubo_ros_control::VisualServoPanel, rviz::Panel)
