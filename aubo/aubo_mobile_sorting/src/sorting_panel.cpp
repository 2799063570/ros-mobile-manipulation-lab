#include <aubo_mobile_sorting/sorting_panel.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <QFont>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

namespace aubo_mobile_sorting
{

SortingPanel::SortingPanel(QWidget* parent)
  : rviz::Panel(parent)
  , state_label_(new QLabel(tr("等待分拣节点...")))
  , detections_label_(new QLabel(tr("红:0  绿:0  蓝:0")))
  , command_label_(new QLabel(tr("请先移动到观察位并确认图像")))
{
  QLabel* title = new QLabel(tr("AUBO 视觉分拣")); // 提高字号加粗
  QFont title_font = title->font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 2);
  title->setFont(title_font);

  QPushButton* observation_button = new QPushButton(tr("1. 移动到相机观察位"));
  QPushButton* start_button = new QPushButton(tr("2. 开始分拣"));
  QPushButton* stop_button = new QPushButton(tr("停止当前任务"));
  QPushButton* open_button = new QPushButton(tr("张开夹爪"));
  QPushButton* home_button = new QPushButton(tr("机械臂归位"));

  start_button->setStyleSheet("font-weight: bold; color: #ffffff; background-color: #2d8a45;");
  stop_button->setStyleSheet("font-weight: bold; color: #ffffff; background-color: #b33a3a;");
  state_label_->setWordWrap(true);
  detections_label_->setWordWrap(true);
  command_label_->setWordWrap(true);

  QGridLayout* button_layout = new QGridLayout();
  button_layout->addWidget(observation_button, 0, 0, 1, 2);
  button_layout->addWidget(start_button, 1, 0, 1, 2);
  button_layout->addWidget(stop_button, 2, 0, 1, 2);
  button_layout->addWidget(open_button, 3, 0);
  button_layout->addWidget(home_button, 3, 1);

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(title);
  layout->addWidget(new QLabel(tr("运行状态：")));
  layout->addWidget(state_label_);
  layout->addWidget(new QLabel(tr("相机识别：")));
  layout->addWidget(detections_label_);
  layout->addLayout(button_layout);
  layout->addWidget(command_label_);
  layout->addStretch();
  setLayout(layout);

  // 五个按钮对应的服务请求
  observation_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
      "/sorting/move_to_observation");
  start_client_ = node_handle_.serviceClient<std_srvs::Trigger>("/sorting/start");
  stop_client_ = node_handle_.serviceClient<std_srvs::Trigger>("/sorting/stop");
  open_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
      "/sorting/open_gripper");
  home_client_ = node_handle_.serviceClient<std_srvs::Trigger>("/sorting/home");

  state_subscriber_ = node_handle_.subscribe(
      "/sorting/state", 1, &SortingPanel::stateCallback, this);   // 订阅分拣的状态
  detections_subscriber_ = node_handle_.subscribe(
      "/sorting/detection_summary", 1, &SortingPanel::detectionsCallback, this);

  connect(observation_button, SIGNAL(clicked()), this, SLOT(moveToObservation()));
  connect(start_button, SIGNAL(clicked()), this, SLOT(startSorting()));
  connect(stop_button, SIGNAL(clicked()), this, SLOT(stopSorting()));
  connect(open_button, SIGNAL(clicked()), this, SLOT(openGripper()));
  connect(home_button, SIGNAL(clicked()), this, SLOT(moveHome())); // 五个按钮 和对应事件连接
  connect(this, SIGNAL(stateReceived(QString)), this, SLOT(showState(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(detectionsReceived(QString)), this,
          SLOT(showDetections(QString)), Qt::QueuedConnection);
}

// 想对应服务客户端 发起请求 
void SortingPanel::callTrigger(ros::ServiceClient& client,
                               const QString& command_name)
{
  std_srvs::Trigger service;
  if (!client.call(service))
  {
    command_label_->setText(command_name + tr("：服务不可用"));
    return;
  }
  const QString response = QString::fromStdString(service.response.message);
  command_label_->setText(command_name + (service.response.success ? tr("：已接受，") : tr("：被拒绝，")) + response);
}

void SortingPanel::moveToObservation()
{
  callTrigger(observation_client_, tr("移动到观察位"));
}

void SortingPanel::startSorting()
{
  callTrigger(start_client_, tr("开始分拣"));
}

void SortingPanel::stopSorting()
{
  callTrigger(stop_client_, tr("停止"));
}

void SortingPanel::openGripper()
{
  callTrigger(open_client_, tr("张开夹爪"));
}

void SortingPanel::moveHome()
{
  callTrigger(home_client_, tr("机械臂归位"));
}

void SortingPanel::stateCallback(const std_msgs::String::ConstPtr& message)
{
  Q_EMIT stateReceived(QString::fromStdString(message->data));
}

void SortingPanel::detectionsCallback(const std_msgs::String::ConstPtr& message)
{
  Q_EMIT detectionsReceived(QString::fromStdString(message->data));
}

void SortingPanel::showState(const QString& text)
{
  const QString code = text.section('|', 0, 0).trimmed();
  const QString detail = text.section('|', 1).trimmed();
  QString translated = code;
  if (code == "INITIALIZING") translated = tr("初始化中");
  else if (code == "IDLE") translated = tr("待命");
  else if (code == "OBSERVING") translated = tr("正在移动到观察位");
  else if (code == "READY") translated = tr("已就绪");
  else if (code == "SORTING") translated = tr("正在分拣");
  else if (code == "DETECTING") translated = tr("正在识别目标");
  else if (code == "PICKING") translated = tr("正在抓取和放置");
  else if (code == "OPENING") translated = tr("正在张开夹爪");
  else if (code == "HOMING") translated = tr("正在归位");
  else if (code == "STOPPED") translated = tr("已停止");
  else if (code == "ERROR") translated = tr("发生错误");
  state_label_->setText(detail.isEmpty() ? translated : translated + " | " + detail);
}

void SortingPanel::showDetections(const QString& text)
{
  detections_label_->setText(text);
}

}  // namespace aubo_mobile_sorting

PLUGINLIB_EXPORT_CLASS(aubo_mobile_sorting::SortingPanel, rviz::Panel)
