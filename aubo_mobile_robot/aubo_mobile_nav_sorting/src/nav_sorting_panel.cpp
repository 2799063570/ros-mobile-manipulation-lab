#include <aubo_mobile_nav_sorting/nav_sorting_panel.h>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Trigger.h>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace
{

QDoubleSpinBox* poseSpinBox(double minimum, double maximum)
{
  QDoubleSpinBox* box = new QDoubleSpinBox();
  box->setDecimals(3);
  box->setRange(minimum, maximum);
  box->setSingleStep(0.05);
  return box;
}

void addDouble(dynamic_reconfigure::Config& config, const std::string& name,
               double value)
{
  dynamic_reconfigure::DoubleParameter parameter;
  parameter.name = name;
  parameter.value = value;
  config.doubles.push_back(parameter);
}

void addInt(dynamic_reconfigure::Config& config, const std::string& name,
            int value)
{
  dynamic_reconfigure::IntParameter parameter;
  parameter.name = name;
  parameter.value = value;
  config.ints.push_back(parameter);
}

void addBool(dynamic_reconfigure::Config& config, const std::string& name,
             bool value)
{
  dynamic_reconfigure::BoolParameter parameter;
  parameter.name = name;
  parameter.value = value;
  config.bools.push_back(parameter);
}

}  // namespace

namespace aubo_mobile_nav_sorting
{

NavSortingPanel::NavSortingPanel(QWidget* parent)
  : rviz::Panel(parent)
  , mission_state_label_(new QLabel(tr("等待 nav_sorting 节点...")))
  , sorting_state_label_(new QLabel(tr("等待 sorting 节点...")))
  , command_label_(new QLabel(tr("任务空闲时可以应用新参数")))
  , start_button_(new QPushButton(tr("开始导航分拣")))
  , stop_button_(new QPushButton(tr("停止当前任务")))
  , goal_x_(poseSpinBox(-20.0, 20.0))
  , goal_y_(poseSpinBox(-20.0, 20.0))
  , goal_yaw_(poseSpinBox(-3.14159, 3.14159))
  , near_field_enabled_(new QCheckBox(tr("启用近场协同精停")))
  , pre_dock_x_(poseSpinBox(-20.0, 20.0))
  , pre_dock_y_(poseSpinBox(-20.0, 20.0))
  , pre_dock_yaw_(poseSpinBox(-3.14159, 3.14159))
  , base_clearance_(poseSpinBox(0.10, 1.50))
  , navigation_timeout_(poseSpinBox(5.0, 900.0))
  , navigation_retries_(new QSpinBox())
  , mission_busy_(false)
{
  QLabel* title = new QLabel(tr("AUBO 导航抓取分拣"));
  QFont title_font = title->font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 2);
  title->setFont(title_font);

  start_button_->setStyleSheet(
      "font-weight: bold; color: #ffffff; background-color: #2d8a45;");
  stop_button_->setStyleSheet(
      "font-weight: bold; color: #ffffff; background-color: #b33a3a;");
  navigation_retries_->setRange(0, 10);
  stop_button_->setEnabled(false);
  navigation_timeout_->setSingleStep(10.0);
  navigation_timeout_->setSuffix(tr(" 秒"));
  base_clearance_->setSuffix(tr(" 米"));
  mission_state_label_->setWordWrap(true);
  sorting_state_label_->setWordWrap(true);
  command_label_->setWordWrap(true);

  QHBoxLayout* command_buttons = new QHBoxLayout();
  command_buttons->addWidget(start_button_);
  command_buttons->addWidget(stop_button_);

  QGroupBox* parameters_group = new QGroupBox(tr("在线任务参数（仅空闲时生效）"));
  QFormLayout* parameters = new QFormLayout();
  parameters->addRow(tr("工位 X"), goal_x_);
  parameters->addRow(tr("工位 Y"), goal_y_);
  parameters->addRow(tr("工位航向 yaw"), goal_yaw_);
  parameters->addRow(near_field_enabled_);
  parameters->addRow(tr("预停靠 X"), pre_dock_x_);
  parameters->addRow(tr("预停靠 Y"), pre_dock_y_);
  parameters->addRow(tr("预停靠 yaw"), pre_dock_yaw_);
  parameters->addRow(tr("桌边最小间距"), base_clearance_);
  parameters->addRow(tr("单次导航超时"), navigation_timeout_);
  parameters->addRow(tr("导航重试次数"), navigation_retries_);

  QPushButton* apply_button = new QPushButton(tr("应用参数"));
  QPushButton* refresh_button = new QPushButton(tr("读取当前值"));
  QHBoxLayout* parameter_buttons = new QHBoxLayout();
  parameter_buttons->addWidget(apply_button);
  parameter_buttons->addWidget(refresh_button);
  parameters->addRow(parameter_buttons);
  parameters_group->setLayout(parameters);

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(title);
  layout->addWidget(new QLabel(tr("总任务状态：")));
  layout->addWidget(mission_state_label_);
  layout->addWidget(new QLabel(tr("抓取子任务状态：")));
  layout->addWidget(sorting_state_label_);
  layout->addLayout(command_buttons);
  layout->addWidget(parameters_group);
  layout->addWidget(command_label_);
  layout->addStretch();
  setLayout(layout);

  start_client_ = node_handle_.serviceClient<std_srvs::Trigger>("/nav_sorting/start");
  stop_client_ = node_handle_.serviceClient<std_srvs::Trigger>("/nav_sorting/stop");
  reconfigure_client_ = node_handle_.serviceClient<dynamic_reconfigure::Reconfigure>(
      "/nav_sorting_mission/set_parameters");
  mission_state_subscriber_ = node_handle_.subscribe(
      "/nav_sorting/state", 1, &NavSortingPanel::missionStateCallback, this);
  sorting_state_subscriber_ = node_handle_.subscribe(
      "/sorting/state", 1, &NavSortingPanel::sortingStateCallback, this);

  connect(start_button_, SIGNAL(clicked()), this, SLOT(startMission()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopMission()));
  connect(apply_button, SIGNAL(clicked()), this, SLOT(applyParameters()));
  connect(refresh_button, SIGNAL(clicked()), this, SLOT(refreshParameters()));
  connect(this, SIGNAL(missionStateReceived(QString)), this,
          SLOT(showMissionState(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(sortingStateReceived(QString)), this,
          SLOT(showSortingState(QString)), Qt::QueuedConnection);

  goal_x_->setValue(2.15);
  pre_dock_x_->setValue(1.85);
  near_field_enabled_->setChecked(true);
  base_clearance_->setValue(0.40);
  navigation_timeout_->setValue(180.0);
  navigation_retries_->setValue(1);
  refreshParameters();
}

void NavSortingPanel::callTrigger(ros::ServiceClient& client,
                                  const QString& command_name)
{
  std_srvs::Trigger service;
  if (!client.call(service))
  {
    command_label_->setText(command_name + tr("：服务不可用"));
    return;
  }
  command_label_->setText(
      command_name + (service.response.success ? tr("：已接受，") : tr("：被拒绝，")) +
      QString::fromStdString(service.response.message));
}

void NavSortingPanel::startMission()
{
  callTrigger(start_client_, tr("开始任务"));
}

void NavSortingPanel::stopMission()
{
  callTrigger(stop_client_, tr("停止任务"));
}

void NavSortingPanel::applyParameters()
{
  if (mission_busy_)
  {
    command_label_->setText(tr("任务运行中，参数未修改；请先停止或等待任务结束"));
    return;
  }
  dynamic_reconfigure::Reconfigure service;
  addDouble(service.request.config, "goal_x", goal_x_->value());
  addDouble(service.request.config, "goal_y", goal_y_->value());
  addDouble(service.request.config, "goal_yaw", goal_yaw_->value());
  addBool(service.request.config, "near_field_enabled", near_field_enabled_->isChecked());
  addDouble(service.request.config, "pre_dock_x", pre_dock_x_->value());
  addDouble(service.request.config, "pre_dock_y", pre_dock_y_->value());
  addDouble(service.request.config, "pre_dock_yaw", pre_dock_yaw_->value());
  addDouble(service.request.config, "near_field_base_clearance", base_clearance_->value());
  addDouble(service.request.config, "navigation_timeout", navigation_timeout_->value());
  addInt(service.request.config, "navigation_retries", navigation_retries_->value());
  if (!reconfigure_client_.call(service))
  {
    command_label_->setText(tr("应用参数：动态调参服务不可用"));
    return;
  }
  command_label_->setText(tr("参数已应用；下一次任务将使用新值"));
  refreshParameters();
}

void NavSortingPanel::refreshParameters()
{
  const std::string prefix = "/nav_sorting_mission/";
  double double_value;
  int int_value;
  bool bool_value;
  bool found = false;
#define READ_DOUBLE(name, widget) \
  if (node_handle_.getParam(prefix + name, double_value)) { widget->setValue(double_value); found = true; }
  READ_DOUBLE("goal_x", goal_x_)
  READ_DOUBLE("goal_y", goal_y_)
  READ_DOUBLE("goal_yaw", goal_yaw_)
  READ_DOUBLE("pre_dock_x", pre_dock_x_)
  READ_DOUBLE("pre_dock_y", pre_dock_y_)
  READ_DOUBLE("pre_dock_yaw", pre_dock_yaw_)
  READ_DOUBLE("near_field_base_clearance", base_clearance_)
  READ_DOUBLE("navigation_timeout", navigation_timeout_)
#undef READ_DOUBLE
  if (node_handle_.getParam(prefix + "navigation_retries", int_value))
  {
    navigation_retries_->setValue(int_value);
    found = true;
  }
  if (node_handle_.getParam(prefix + "near_field_enabled", bool_value))
  {
    near_field_enabled_->setChecked(bool_value);
    found = true;
  }
  command_label_->setText(found ? tr("已读取当前参数") : tr("参数节点尚未启动，显示默认值"));
}

void NavSortingPanel::missionStateCallback(const std_msgs::String::ConstPtr& message)
{
  Q_EMIT missionStateReceived(QString::fromStdString(message->data));
}

void NavSortingPanel::sortingStateCallback(const std_msgs::String::ConstPtr& message)
{
  Q_EMIT sortingStateReceived(QString::fromStdString(message->data));
}

void NavSortingPanel::showMissionState(const QString& text)
{
  const QString code = text.section('|', 0, 0).trimmed();
  const QString detail = text.section('|', 1).trimmed();
  mission_busy_ = !(code == "IDLE" || code == "STOPPED" ||
                    code == "SUCCEEDED" || code == "FAILED");
  start_button_->setEnabled(!mission_busy_);
  stop_button_->setEnabled(mission_busy_ && code != "INITIALIZING" &&
                          code != "STOP_UNCONFIRMED");
  QString translated = code;
  if (code == "INITIALIZING") translated = tr("初始化中");
  else if (code == "IDLE") translated = tr("待命");
  else if (code == "STOWING_ARM") translated = tr("正在收拢机械臂");
  else if (code == "NAVIGATING") translated = tr("正在导航");
  else if (code == "PREPARING_ARM") translated = tr("正在准备机械臂");
  else if (code == "COORDINATING") translated = tr("正在协同精停");
  else if (code == "VALIDATING_DOCK") translated = tr("正在验证停靠位");
  else if (code == "AT_WORKSTATION") translated = tr("已到达工位");
  else if (code == "SORTING") translated = tr("正在抓取分拣");
  else if (code == "STOPPING") translated = tr("正在停止");
  else if (code == "STOP_UNCONFIRMED") translated = tr("停止未确认，已禁止新任务；请检查分拣服务");
  else if (code == "STOPPED") translated = tr("已停止");
  else if (code == "SUCCEEDED") translated = tr("任务成功");
  else if (code == "FAILED") translated = tr("任务失败");
  mission_state_label_->setText(detail.isEmpty() ? translated : translated + " | " + detail);
}

void NavSortingPanel::showSortingState(const QString& text)
{
  sorting_state_label_->setText(text);
}

}  // namespace aubo_mobile_nav_sorting

PLUGINLIB_EXPORT_CLASS(aubo_mobile_nav_sorting::NavSortingPanel, rviz::Panel)
