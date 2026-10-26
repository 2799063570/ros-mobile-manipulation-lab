#pragma once
#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <cmath>
#include <algorithm>
#include <stdexcept>

static constexpr double PI = 3.14159265358979323846;

struct Curve
{
  std::string shape, plane;
  double radius, amplitude, length, duration, kp, ki, correction_fraction, speed, error, extent, tolerance, settle_speed, settle_timeout;
  int waves, cycles;
  // Relative curve starts exactly at the captured TCP; quintic phase gives smooth endpoints.
  void sample(double t, KDL::Vector& p, KDL::Vector& v) const
  {
    const double u = std::max(0.0, std::min(1.0, t/duration));
    const double s = u*u*u*(10+u*(-15+6*u));
    const double ds = 30*u*u*(1-u)*(1-u)/duration;
    double a, b, da, db;
    if (shape == "circle") {
      const double theta = 2*PI*cycles*s, omega = 2*PI*cycles*ds;
      a = radius*(std::cos(theta)-1); b = radius*std::sin(theta);
      da = -radius*std::sin(theta)*omega; db = radius*std::cos(theta)*omega;
    } else {
      const double theta = 2*PI*waves*s;
      a = length*s; b = amplitude*std::sin(theta);
      da = length*ds; db = amplitude*std::cos(theta)*2*PI*waves*ds;
    }
    p = plane == "xy" ? KDL::Vector(a,b,0) : plane == "xz" ? KDL::Vector(a,0,b) : KDL::Vector(0,a,b);
    v = plane == "xy" ? KDL::Vector(da,db,0) : plane == "xz" ? KDL::Vector(da,0,db) : KDL::Vector(0,da,db);
  }
};

inline Curve loadCurve(ros::NodeHandle& pn)
{
  Curve c;
  pn.param<std::string>("shape",c.shape,"circle"); pn.param<std::string>("plane",c.plane,"xy");
  pn.param("radius",c.radius,0.03); pn.param("amplitude",c.amplitude,0.02);
  pn.param("length",c.length,0.10); pn.param("waves",c.waves,1); pn.param("cycles",c.cycles,1);
  pn.param("duration",c.duration,40.0); pn.param("kp",c.kp,1.0);
  pn.param("ki",c.ki,0.0); pn.param("correction_fraction",c.correction_fraction,0.2);
  pn.param("max_speed",c.speed,0.015); pn.param("max_error",c.error,0.04); pn.param("max_extent",c.extent,0.25);
  pn.param("settle_tolerance",c.tolerance,0.002); pn.param("settle_speed",c.settle_speed,0.003);
  pn.param("settle_timeout",c.settle_timeout,15.0);
  for(double x : {c.radius,c.amplitude,c.length,c.duration,c.kp,c.speed,c.error,c.extent,c.tolerance,c.settle_speed,c.settle_timeout})
    if(!std::isfinite(x) || x<=0) throw std::runtime_error("所有数值参数必须是有限正数");
  if(!std::isfinite(c.ki) || c.ki<0 || c.ki>2 || !std::isfinite(c.correction_fraction) || c.correction_fraction<=0 || c.correction_fraction>.2)
    throw std::runtime_error("积分增益必须在 0..2，纠偏速度比例必须在 0..0.2");
  if((c.shape!="circle" && c.shape!="sine") || (c.plane!="xy" && c.plane!="xz" && c.plane!="yz") ||
     c.waves<1 || c.waves>20 || c.cycles<1 || c.cycles>20 || c.duration<2 || c.duration>3600 || c.speed>0.10 || c.extent>0.5)
    throw std::runtime_error("轨迹类型、平面或参数范围无效，请检查尺寸、圈数和时长");
  // Analytic upper bounds (max quintic phase speed = 1.875/T), not a sparse speed check.
  const double path_bound = c.shape=="circle" ? 2*c.radius : std::hypot(c.length,c.amplitude);
  const double speed_bound = 1.875/c.duration*(c.shape=="circle" ? 2*PI*c.cycles*c.radius : std::hypot(c.length,2*PI*c.waves*c.amplitude));
  if(path_bound>c.extent)
    throw std::runtime_error("轨迹相对起点最大位移 "+std::to_string(path_bound)+" m，超过 max_extent="+std::to_string(c.extent)+" m，请减小尺寸或调整允许范围");
  if(speed_bound>0.8*c.speed)
    throw std::runtime_error("参考速度超过 80% 速度预算，建议总时长至少 "+std::to_string(c.duration*speed_bound/(0.8*c.speed))+" s，或减小尺寸");
  if(c.tolerance>=c.error) throw std::runtime_error("到位容差必须小于误差上限");
  return c;
}
