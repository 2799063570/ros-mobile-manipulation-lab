#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEvent>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <array>
#include <algorithm>

// 独立 Qt 窗口，不依赖 RViz 面板；ROS 回调通过 GUI 定时器处理。
// 窗口失焦、按键释放、关闭或状态心跳消失，都先发布零速度。
class JogWindow : public QWidget
{
public:
  JogWindow()
  {
    ros::NodeHandle private_nh("~");
    nh_.param<std::string>("/aubo_cartesian_jog/base_link", base_, "base_link");
    nh_.param<std::string>("/aubo_cartesian_jog/tool_link", tool_, "tcp_link");
    private_nh.param<std::string>("base_link", base_, base_);
    private_nh.param<std::string>("tool_link", tool_, tool_);
    double max_speed;
    nh_.param("/aubo_cartesian_jog/max_linear_velocity", max_speed, 0.02);
    setWindowTitle(QString::fromUtf8("AUBO 末端 XYZ 点动"));
    auto* layout = new QVBoxLayout(this);
    auto* instruction = new QLabel(QString::fromUtf8("先使能，再按住方向按钮；松手减速停止。"));
    layout->addWidget(instruction);
    auto* settings = new QGridLayout;
    frame_ = new QComboBox;
    frame_->addItem(QString::fromUtf8("基坐标系"));
    frame_->addItem(QString::fromUtf8("工具坐标系"));
    speed_ = new QDoubleSpinBox;
    speed_->setRange(0.1, std::max(0.1, max_speed * 1000));
    speed_->setValue(std::min(10.0, max_speed * 1000));
    speed_->setSuffix(" mm/s");
    settings->addWidget(new QLabel(QString::fromUtf8("方向参考")), 0, 0);
    settings->addWidget(frame_, 0, 1);
    settings->addWidget(new QLabel(QString::fromUtf8("末端速度")), 1, 0);
    settings->addWidget(speed_, 1, 1);
    layout->addLayout(settings);
    auto* grid = new QGridLayout;
    const std::array<const char*, 6> labels{{"X−", "X+", "Y−", "Y+", "Z−", "Z+"}};
    for (int i = 0; i < 6; ++i)
    {
      auto* button = new QPushButton(QString::fromUtf8(labels[i]));
      button->setMinimumSize(110, 60);
      button->setAutoRepeat(false);
      button->setFocusPolicy(Qt::NoFocus);
      buttons_[i] = button;
      grid->addWidget(button, i / 2, i % 2);
      connect(button, &QPushButton::pressed, this, [this, i]() {
        if (ready_ && isActiveWindow())
        {
          active_button_ = i;
          frame_->setEnabled(false);
          speed_->setEnabled(false);
          publishCommand();
        }
      });
      connect(button, &QPushButton::released, this, [this]() { release(); });
    }
    layout->addLayout(grid);
    auto* enable = new QPushButton(QString::fromUtf8("使能点动"));
    auto* stop = new QPushButton(QString::fromUtf8("停止并禁用"));
    auto* reset = new QPushButton(QString::fromUtf8("复位故障"));
    layout->addWidget(enable);
    layout->addWidget(stop);
    layout->addWidget(reset);
    status_ = new QLabel(QString::fromUtf8("等待控制器状态……"));
    status_->setWordWrap(true);
    layout->addWidget(status_);
    result_ = new QLabel;
    result_->setWordWrap(true);
    layout->addWidget(result_);
    auto* note = new QLabel(QString::fromUtf8("停止按钮是软件减速停止；硬件急停请使用示教器/控制柜。"));
    note->setWordWrap(true);
    layout->addWidget(note);

    publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("/cartesian_jog/command", 1);
    subscriber_ = nh_.subscribe<std_msgs::String>("/cartesian_jog/state", 1,
        [this](const std_msgs::String::ConstPtr& message) {
          last_state_.restart();
          ready_ = message->data == "ENABLED_IDLE" || message->data == "JOGGING";
          status_->setText(QString::fromUtf8("控制器：") + QString::fromStdString(message->data));
          if (!ready_)
            release();
        });
    enable_client_ = nh_.serviceClient<std_srvs::SetBool>("/cartesian_jog/set_enabled");
    reset_client_ = nh_.serviceClient<std_srvs::Trigger>("/cartesian_jog/reset");
    connect(enable, &QPushButton::clicked, this, [this]() { setEnabled(true); });
    connect(stop, &QPushButton::clicked, this, [this]() { setEnabled(false); });
    connect(reset, &QPushButton::clicked, this, [this]() {
      release();
      std_srvs::Trigger service;
      const bool ok = reset_client_.exists() && reset_client_.call(service);
      result_->setText(ok ? QString::fromStdString(service.response.message)
                          : QString::fromUtf8("复位服务不可用"));
    });
    connect(qApp, &QGuiApplication::applicationStateChanged, this, [this](Qt::ApplicationState state) {
      if (state != Qt::ApplicationActive)
        release();
    });
    connect(frame_, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this, [this](int) { release(); });
    auto* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
      ros::spinOnce();
      if (!ros::ok())
      {
        close();
        return;
      }
      if (!last_state_.isValid() || last_state_.elapsed() > 500)
      {
        ready_ = false;
        release();
        status_->setText(QString::fromUtf8("控制器状态超时，已释放方向按钮"));
      }
      for (auto* button : buttons_)
        button->setEnabled(ready_);
      publishCommand();
    });
    timer->start(33);  // 约 30 Hz 输入心跳；200 Hz 路点由控制器独立生成。
    resize(330, 520);
  }

protected:
  bool event(QEvent* event) override
  {
    if (event->type() == QEvent::WindowDeactivate || event->type() == QEvent::Hide)
      release();
    return QWidget::event(event);
  }

  void closeEvent(QCloseEvent* event) override
  {
    release();
    // 不依赖服务完成才能关窗；若零指令丢包，控制器仍会在输入超时后减速。
    event->accept();
  }

private:
  void publishCommand()
  {
    if (!publisher_)
      return;
    geometry_msgs::TwistStamped command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = frame_->currentIndex() == 0 ? base_ : tool_;
    if (ready_ && active_button_ >= 0 && isActiveWindow())
    {
      // 界面使用 mm/s，ROS 消息统一使用 m/s；角速度保持零。
      const double velocity = speed_->value() / 1000.0 * (active_button_ % 2 == 0 ? -1 : 1);
      if (active_button_ / 2 == 0) command.twist.linear.x = velocity;
      if (active_button_ / 2 == 1) command.twist.linear.y = velocity;
      if (active_button_ / 2 == 2) command.twist.linear.z = velocity;
    }
    publisher_.publish(command);
  }

  void release()
  {
    active_button_ = -1;
    if (frame_) frame_->setEnabled(true);
    if (speed_) speed_->setEnabled(true);
    if (frame_ && speed_) publishCommand();
  }

  void setEnabled(bool value)
  {
    release();  // 服务调用可能阻塞，必须先发布零指令。
    std_srvs::SetBool service;
    service.request.data = value;
    const bool ok = enable_client_.exists() && enable_client_.call(service);
    result_->setText(ok ? QString::fromStdString(service.response.message)
                        : QString::fromUtf8("使能服务不可用"));
  }

  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  ros::ServiceClient enable_client_, reset_client_;
  std::string base_, tool_;
  QComboBox* frame_{nullptr};
  QDoubleSpinBox* speed_{nullptr};
  QLabel* status_{nullptr};
  QLabel* result_{nullptr};
  std::array<QPushButton*, 6> buttons_;
  QElapsedTimer last_state_;
  int active_button_{-1};
  bool ready_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aubo_cartesian_jog_panel");
  QApplication application(argc, argv);
  JogWindow window;
  window.show();
  return application.exec();
}
