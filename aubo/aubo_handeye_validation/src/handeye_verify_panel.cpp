#include <aubo_handeye_validation/handeye_verify_panel.h>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <pluginlib/class_list_macros.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QThread>
#include <QVBoxLayout>

namespace aubo_handeye_validation
{
namespace
{

class TriggerCallThread : public QThread
{
  Q_OBJECT

public:
  TriggerCallThread(QString action, QString service, QObject* parent = nullptr)
    : QThread(parent), action_(std::move(action)), service_(std::move(service))
  {
  }

Q_SIGNALS:
  void completed(const QString& action, bool success, const QString& message);

protected:
  void run() override
  {
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<std_srvs::Trigger>(service_.toStdString());
    std_srvs::Trigger request;
    if (!client.call(request))
    {
      emit completed(action_, false, tr("服务调用失败：%1").arg(service_));
      return;
    }
    emit completed(action_, request.response.success,
                   QString::fromStdString(request.response.message));
  }

private:
  QString action_;
  QString service_;
};

}  // namespace

HandeyeVerifyPanel::HandeyeVerifyPanel(QWidget* parent)
  : rviz::Panel(parent)
  , status_label_(new QLabel(tr("等待验证节点状态…")))
  , plan_button_(new QPushButton(tr("规划预览")))
  , move_button_(new QPushButton(tr("执行悬停")))
  , stop_button_(new QPushButton(tr("停止轨迹")))
{
  status_label_->setWordWrap(true);
  stop_button_->setToolTip(tr("请求 MoveIt 取消当前验证轨迹；这不是控制柜急停。"));

  auto* buttons = new QHBoxLayout;
  buttons->addWidget(plan_button_);
  buttons->addWidget(move_button_);
  buttons->addWidget(stop_button_);

  auto* layout = new QVBoxLayout;
  layout->addWidget(new QLabel(tr("眼在手外目标悬停验证")));
  layout->addLayout(buttons);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(plan_button_, &QPushButton::clicked, this, &HandeyeVerifyPanel::plan);
  connect(move_button_, &QPushButton::clicked, this, &HandeyeVerifyPanel::moveToHover);
  connect(stop_button_, &QPushButton::clicked, this, &HandeyeVerifyPanel::stop);
  connect(this, &HandeyeVerifyPanel::statusReceived,
          this, &HandeyeVerifyPanel::updateStatus, Qt::QueuedConnection);
  status_subscriber_ = node_handle_.subscribe(
      "/handeye_yolo_verify/status", 10,
      &HandeyeVerifyPanel::statusCallback, this);
}

void HandeyeVerifyPanel::plan()
{
  callService(tr("规划预览"), QStringLiteral("/handeye_yolo_verify/plan"), true);
}

void HandeyeVerifyPanel::moveToHover()
{
  const auto choice = QMessageBox::warning(
      this, tr("确认真机运动"),
      tr("将调用验证服务，机械臂会低速移动并悬停在目标上方，不会闭合夹爪。\n"
         "请确认 RViz 中目标、桌面和轨迹均与现场一致。"),
      QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
  if (choice == QMessageBox::Yes)
    callService(tr("执行悬停"), QStringLiteral("/handeye_yolo_verify/move"), true);
}

void HandeyeVerifyPanel::stop()
{
  callService(tr("停止请求"), QStringLiteral("/handeye_yolo_verify/stop"), false);
}

void HandeyeVerifyPanel::callService(const QString& action, const QString& service,
                                     bool motion_request)
{
  if (motion_request && motion_request_active_)
    return;
  if (motion_request)
  {
    motion_request_active_ = true;
    plan_button_->setEnabled(false);
    move_button_->setEnabled(false);
  }
  status_label_->setText(tr("正在请求：%1…").arg(action));

  auto* worker = new TriggerCallThread(action, service);
  connect(worker, &TriggerCallThread::completed,
          this, &HandeyeVerifyPanel::serviceFinished, Qt::QueuedConnection);
  connect(worker, &QThread::finished, worker, &QObject::deleteLater);
  worker->start();
}

void HandeyeVerifyPanel::statusCallback(const std_msgs::String::ConstPtr& message)
{
  emit statusReceived(QString::fromStdString(message->data));
}

void HandeyeVerifyPanel::updateStatus(const QString& status)
{
  status_label_->setText(status);
}

void HandeyeVerifyPanel::serviceFinished(const QString& action, bool success,
                                         const QString& message)
{
  if (action == tr("规划预览") || action == tr("执行悬停"))
  {
    motion_request_active_ = false;
    plan_button_->setEnabled(true);
    move_button_->setEnabled(true);
  }
  const QString detail = message.isEmpty() ? tr("无服务返回消息") : message;
  status_label_->setText(tr("%1：%2（%3）")
                             .arg(action, success ? tr("成功") : tr("失败"), detail));
}

}  // namespace aubo_handeye_validation

PLUGINLIB_EXPORT_CLASS(aubo_handeye_validation::HandeyeVerifyPanel, rviz::Panel)

#include "handeye_verify_panel.moc"
