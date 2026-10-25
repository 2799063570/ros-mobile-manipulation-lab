#include <aubo_ros_control/visual_servo_panel.h>
#include <aubo_ros_control/visual_servo_error_plot.h>
#include <gtest/gtest.h>
#include <QApplication>
#include <QLabel>
#include <QGroupBox>
#include <QTimer>
#include <boost/make_shared.hpp>
#include <chrono>
#include <thread>

namespace aubo_ros_control {
struct VisualServoPanelTestAccess {
  static void show(VisualServoPanel& p,const VisualServoStatus& s) {
    p.telemetryCallback(boost::make_shared<VisualServoStatus>(s));
    QApplication::processEvents();
  }
  static std::size_t samples(const VisualServoPanel& p) {return p.error_plot_->sampleCount();}
  static void stale(VisualServoPanel& p) { p.checkTelemetryFreshness(); }
  static void preparePreview(VisualServoPanel& p) {
    p.readiness_timer_->stop();
    p.showServoState("TRACKING"); p.showPerceptionState("DETECTED|red|离线合成数据");
    p.title_label_->setText(QString::fromUtf8("AUBO 视觉位置伺服 · 离线界面预览"));
    p.readiness_label_->setText(QString::fromUtf8("合成显示数据；未连接机械臂、相机或运动控制器"));
    p.target_pose_label_->setText("x=0.330 m  y=-0.090 m  z=0.270 m\nframe: base_link");
  }
  static void spacePreviewHistory(VisualServoPanel& p) {
    unsigned i=0;
    // Synthetic times are for the screenshot only; live samples use the GUI's
    // monotonic clock. Do not sleep a minute merely to render a preview.
    for (auto& sample:p.error_plot_->samples_) sample.time=i++*.09;
  }
};
}

TEST(VisualServoView, QueuedTelemetryInvalidationAndBoundedPlot) {
  using namespace aubo_ros_control;
  ros::NodeHandle nh;
  std::atomic<int> selections{0};
  auto selection_sub=nh.subscribe<std_msgs::String>("/visual_servo/target_selection",1,
      [&](const std_msgs::String::ConstPtr&){++selections;});
  VisualServoPanel panel;
  VisualServoPanelTestAccess::preparePreview(panel);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while (selection_sub.getNumPublishers()==0 && std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); QApplication::processEvents();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(0,selections.load());  // Opening the view must not change the target.
  VisualServoStatus s;
  s.header.frame_id="base_link"; s.error_frame="base_link";
  s.state="TRACKING"; s.feedback_valid=s.target_valid=s.error_valid=true;
  s.tcp_position.x=.32; s.tcp_position.y=-.12; s.tcp_position.z=.28;
  s.goal_position.x=.34; s.goal_position.y=-.11; s.goal_position.z=.30;
  s.position_error.x=.02; s.position_error.y=.01; s.position_error.z=.02;
  s.position_error_norm=.03; s.position_deadband=.004;
  s.feedback_age=.012; s.target_age=.043;
  s.detail="只读几何参考；不代表控制器接收或运动许可";
  VisualServoPanelTestAccess::show(panel,s);
  auto* error=panel.findChild<QLabel*>("visual_position_error");
  auto* goal=panel.findChild<QLabel*>("visual_goal_position");
  ASSERT_NE(nullptr,error); ASSERT_NE(nullptr,goal);
  EXPECT_TRUE(error->text().contains("20.00"));
  EXPECT_TRUE(goal->text().contains("340.0"));
  s.error_valid=false;
  VisualServoPanelTestAccess::show(panel,s);
  EXPECT_EQ("--",error->text()); EXPECT_EQ("--",goal->text());
  s.error_valid=true;
  for (int i=0;i<650;++i) {
    s.position_error.x=.02*std::exp(-i/120.)*std::cos(i*.04);
    s.position_error.y=.01*std::exp(-i/160.)*std::sin(i*.025);
    s.position_error.z=.02*std::exp(-i/180.);
    s.tcp_position.x=s.goal_position.x-s.position_error.x;
    s.tcp_position.y=s.goal_position.y-s.position_error.y;
    s.tcp_position.z=s.goal_position.z-s.position_error.z;
    s.position_error_norm=std::sqrt(s.position_error.x*s.position_error.x+s.position_error.y*s.position_error.y+s.position_error.z*s.position_error.z);
    VisualServoPanelTestAccess::show(panel,s);
  }
  EXPECT_LE(VisualServoPanelTestAccess::samples(panel),600u);
  VisualServoPanelTestAccess::spacePreviewHistory(panel);
  panel.resize(900,1100); panel.show(); QApplication::processEvents();
  EXPECT_TRUE(panel.grab().save("/tmp/aubo_visual_servo_panel_preview.png"));
  EXPECT_TRUE(panel.findChild<QGroupBox*>("visual_geometry_monitor")->grab().save("/tmp/aubo_visual_servo_monitor_preview.png"));
  std::this_thread::sleep_for(std::chrono::milliseconds(750));
  VisualServoPanelTestAccess::stale(panel);
  EXPECT_EQ("--",error->text());
  EXPECT_TRUE(panel.findChild<QLabel*>("visual_monitor_status")->text().contains(QString::fromUtf8("失联")));
}

int main(int argc,char** argv) {
  ros::init(argc,argv,"visual_servo_panel_view_test");
  QApplication app(argc,argv);
  ros::AsyncSpinner spinner(1); spinner.start();
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
