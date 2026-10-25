#pragma once

#include <aubo_ros_control/VisualServoStatus.h>
#include <QElapsedTimer>
#include <QPainter>
#include <QPainterPath>
#include <QWidget>
#include <array>
#include <cmath>
#include <deque>

namespace aubo_ros_control {
// GUI-thread-only bounded history. This widget never publishes a ROS command.
class VisualServoErrorPlot : public QWidget {
public:
  explicit VisualServoErrorPlot(QWidget* parent=nullptr) : QWidget(parent) {
    setMinimumHeight(190); clock_.start();
  }
  void clear() { samples_.clear(); update(); }
  std::size_t sampleCount() const { return samples_.size(); }
  void append(const VisualServoStatus& status) {
    if (frame_!=status.error_frame) { clear(); frame_=status.error_frame; }
    Sample s;
    s.time=clock_.elapsed()/1000.;
    s.values={{status.position_error.x*1000,status.position_error.y*1000,status.position_error.z*1000}};
    s.valid=status.error_valid;
    for (double v:s.values) s.valid &= std::isfinite(v);
    if (std::isfinite(status.position_deadband)) deadband_=status.position_deadband*1000;
    samples_.push_back(s);
    while (samples_.size()>600 || (!samples_.empty() && s.time-samples_.front().time>60)) samples_.pop_front();
    update();
  }
  void markStale() {
    if (samples_.empty() || !samples_.back().valid) return;
    Sample s; s.time=clock_.elapsed()/1000.; s.valid=false;
    samples_.push_back(s);
    while (samples_.size()>600) samples_.pop_front();
    update();
  }
protected:
  void paintEvent(QPaintEvent*) override {
    QPainter p(this); p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(),QColor("#14202e"));
    const QRectF area(52,30,width()-68,height()-62);
    if (area.width()<20 || area.height()<20) return;
    const QColor colors[]={QColor("#ff7676"),QColor("#79d69b"),QColor("#72b7ff")};
    p.setPen(QColor("#dce6f2"));
    p.drawText(10,18,QString::fromUtf8("XYZ几何误差 / mm"));
    for (int axis=0;axis<3;++axis) {
      p.setPen(colors[axis]); p.drawText(width()-96+axis*30,18,QString("XYZ").mid(axis,1));
    }
    double extent=std::max(5.,std::abs(deadband_)*1.5);
    for (const auto& s:samples_) if (s.valid)
      for (double v:s.values) extent=std::max(extent,std::abs(v)*1.1);
    const double end=std::max(60.,clock_.elapsed()/1000.),begin=end-60;
    auto x=[&](double t){return area.left()+(t-begin)/60*area.width();};
    auto y=[&](double v){return area.center().y()-v/extent*area.height()/2;};
    for (int i=0;i<=4;++i) {
      const double value=extent*(1-i/2.);
      const double row=area.top()+area.height()*i/4;
      p.setPen(QColor("#2e4053")); p.drawLine(QPointF(area.left(),row),QPointF(area.right(),row));
      p.setPen(QColor("#a9bacd")); p.drawText(QRectF(0,row-8,46,16),Qt::AlignRight,QString::number(value,'f',1));
    }
    p.setPen(QPen(QColor("#d5c78b"),1,Qt::DashLine));
    for (double v:{-deadband_,deadband_}) p.drawLine(QPointF(area.left(),y(v)),QPointF(area.right(),y(v)));
    p.save(); p.setClipRect(area);
    for (int axis=0;axis<3;++axis) {
      QPainterPath path; bool connected=false;
      for (const auto& s:samples_) {
        if (!s.valid) { connected=false; continue; }
        const QPointF at(x(s.time),y(s.values[axis]));
        if (connected) path.lineTo(at); else path.moveTo(at);
        connected=true;
      }
      p.setPen(QPen(colors[axis],1.8)); p.drawPath(path);
    }
    p.restore(); p.setPen(QColor("#a9bacd"));
    for (int i=0;i<3;++i) p.drawText(QRectF(std::min<double>(width()-52,area.left()+i*area.width()/2-25),area.bottom()+7,50,18),Qt::AlignCenter,QString::number(begin+i*30,'f',0)+" s");
    if (samples_.empty() || !samples_.back().valid) {
      p.setPen(QColor("#ffcb85"));
      p.drawText(area,Qt::AlignCenter,QString::fromUtf8("等待有效数据；历史曲线保留"));
    }
  }
private:
  friend struct VisualServoPanelTestAccess;
  struct Sample { double time{0}; std::array<double,3> values{}; bool valid{false}; };
  std::deque<Sample> samples_;
  QElapsedTimer clock_;
  std::string frame_;
  double deadband_{4};
};
}
