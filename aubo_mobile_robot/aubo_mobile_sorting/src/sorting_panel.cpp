#include <aubo_mobile_sorting/sorting_panel.h>

#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <QFont>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>
#include <cmath>

namespace {
QDoubleSpinBox* parameterBox(double minimum, double maximum, double step, int decimals)
{
  QDoubleSpinBox* box = new QDoubleSpinBox();
  box->setRange(minimum, maximum);
  box->setSingleStep(step);
  box->setDecimals(decimals);
  return box;
}

void addDouble(dynamic_reconfigure::Config& config, const std::string& name, double value)
{
  dynamic_reconfigure::DoubleParameter parameter;
  parameter.name = name;
  parameter.value = value;
  config.doubles.push_back(parameter);
}
}  // namespace

namespace aubo_mobile_sorting
{

SortingPanel::SortingPanel(QWidget* parent)
  : rviz::Panel(parent)
  , state_label_(new QLabel(tr("等待分拣节点...")))
  , detections_label_(new QLabel(tr("红:0  绿:0  蓝:0")))
  , command_label_(new QLabel(tr("请先移动到观察位并确认图像")))
  , parameter_label_(new QLabel(tr("等待动态参数服务...")))
  , detection_timeout_(parameterBox(1.0, 120.0, 1.0, 1))
  , detection_samples_(new QSpinBox())
  , grasp_offset_x_(parameterBox(-0.05, 0.05, 0.001, 3))
  , grasp_offset_y_(parameterBox(-0.05, 0.05, 0.001, 3))
  , velocity_scaling_(parameterBox(0.01, 1.0, 0.01, 2))
  , acceleration_scaling_(parameterBox(0.01, 1.0, 0.01, 2))
  , apply_button_(new QPushButton(tr("应用参数")))
{
  detection_samples_->setRange(1, 30);
  apply_button_->setEnabled(false);
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

  QGroupBox* tuning_group = new QGroupBox(tr("在线参数（机械臂空闲时应用）"));
  QFormLayout* tuning = new QFormLayout();
  tuning->addRow(tr("识别等待 s"), detection_timeout_);
  tuning->addRow(tr("识别采样帧数"), detection_samples_);
  tuning->addRow(tr("抓取 X 偏移 m"), grasp_offset_x_);
  tuning->addRow(tr("抓取 Y 偏移 m"), grasp_offset_y_);
  tuning->addRow(tr("速度比例"), velocity_scaling_);
  tuning->addRow(tr("加速度比例"), acceleration_scaling_);
  QPushButton* refresh_button = new QPushButton(tr("读取当前值"));
  QGridLayout* tuning_buttons = new QGridLayout();
  tuning_buttons->addWidget(apply_button_, 0, 0);
  tuning_buttons->addWidget(refresh_button, 0, 1);
  tuning->addRow(tuning_buttons);
  tuning->addRow(parameter_label_);
  tuning_group->setLayout(tuning);
  QScrollArea* tuning_scroll = new QScrollArea();
  tuning_scroll->setWidgetResizable(true);
  tuning_scroll->setMaximumHeight(265);
  tuning_scroll->setWidget(tuning_group);

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(title);
  layout->addWidget(new QLabel(tr("运行状态：")));
  layout->addWidget(state_label_);
  layout->addWidget(new QLabel(tr("相机识别：")));
  layout->addWidget(detections_label_);
  layout->addLayout(button_layout);
  layout->addWidget(tuning_scroll);
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
  reconfigure_client_ = node_handle_.serviceClient<dynamic_reconfigure::Reconfigure>(
      "/color_sorting_task/set_parameters");

  state_subscriber_ = node_handle_.subscribe(
      "/sorting/state", 1, &SortingPanel::stateCallback, this);   // 订阅分拣的状态
  detections_subscriber_ = node_handle_.subscribe(
      "/sorting/detection_summary", 1, &SortingPanel::detectionsCallback, this);
  parameter_subscriber_ = node_handle_.subscribe(
      "/color_sorting_task/parameter_updates", 1,
      &SortingPanel::parameterUpdateCallback, this);

  connect(observation_button, SIGNAL(clicked()), this, SLOT(moveToObservation()));
  connect(start_button, SIGNAL(clicked()), this, SLOT(startSorting()));
  connect(stop_button, SIGNAL(clicked()), this, SLOT(stopSorting()));
  connect(open_button, SIGNAL(clicked()), this, SLOT(openGripper()));
  connect(home_button, SIGNAL(clicked()), this, SLOT(moveHome())); // 五个按钮 和对应事件连接
  connect(apply_button_, SIGNAL(clicked()), this, SLOT(applyParameters()));
  connect(refresh_button, SIGNAL(clicked()), this, SLOT(refreshParameters()));
  connect(this, SIGNAL(parametersChanged()), this, SLOT(refreshParameters()), Qt::QueuedConnection);
  connect(this, SIGNAL(stateReceived(QString)), this, SLOT(showState(QString)),
          Qt::QueuedConnection);
  connect(this, SIGNAL(detectionsReceived(QString)), this,
          SLOT(showDetections(QString)), Qt::QueuedConnection);
  refreshParameters();
}

void SortingPanel::parameterUpdateCallback(const dynamic_reconfigure::Config::ConstPtr&)
{
  Q_EMIT parametersChanged();
}

void SortingPanel::refreshParameters()
{
  const std::string prefix = "/color_sorting_task/";
  double value;
  bool found = false;
#define READ_DOUBLE(name, widget) \
  if (node_handle_.getParam(prefix + name, value)) { widget->setValue(value); found = true; }
  READ_DOUBLE("detection_timeout", detection_timeout_)
  READ_DOUBLE("grasp_offset_x", grasp_offset_x_)
  READ_DOUBLE("grasp_offset_y", grasp_offset_y_)
  READ_DOUBLE("velocity_scaling", velocity_scaling_)
  READ_DOUBLE("acceleration_scaling", acceleration_scaling_)
#undef READ_DOUBLE
  int samples;
  if (node_handle_.getParam(prefix + "detection_samples", samples))
  {
    detection_samples_->setValue(samples);
    found = true;
  }
  parameter_label_->setText(found && reconfigure_client_.exists() ?
      tr("已读取当前生效值") : tr("分拣节点尚未启动或动态服务不可用"));
}

void SortingPanel::applyParameters()
{
  if (!reconfigure_client_.exists())
  {
    parameter_label_->setText(tr("动态参数服务不可用（默认 C++ 任务节点）"));
    return;
  }
  dynamic_reconfigure::Reconfigure service;
  addDouble(service.request.config, "detection_timeout", detection_timeout_->value());
  dynamic_reconfigure::IntParameter samples;
  samples.name = "detection_samples";
  samples.value = detection_samples_->value();
  service.request.config.ints.push_back(samples);
  addDouble(service.request.config, "grasp_offset_x", grasp_offset_x_->value());
  addDouble(service.request.config, "grasp_offset_y", grasp_offset_y_->value());
  addDouble(service.request.config, "velocity_scaling", velocity_scaling_->value());
  addDouble(service.request.config, "acceleration_scaling", acceleration_scaling_->value());
  if (!reconfigure_client_.call(service))
  {
    parameter_label_->setText(tr("参数应用失败：服务调用未完成"));
    return;
  }
  bool accepted = true;
  for (const auto& parameter : service.request.config.doubles)
  {
    bool matched = false;
    for (const auto& actual : service.response.config.doubles)
      if (actual.name == parameter.name)
        matched = std::abs(actual.value - parameter.value) < 1e-6;
    accepted = accepted && matched;
  }
  bool samples_matched = false;
  for (const auto& actual : service.response.config.ints)
    if (actual.name == samples.name)
      samples_matched = actual.value == samples.value;
  accepted = accepted && samples_matched;
  refreshParameters();
  command_label_->setText(accepted ? tr("参数已生效") : tr("节点拒绝或修正了参数，请检查当前值"));
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
  apply_button_->setEnabled(code == "IDLE" || code == "READY" || code == "STOPPED");
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
