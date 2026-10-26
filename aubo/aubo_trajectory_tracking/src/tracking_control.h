#pragma once
#include <kdl/frames.hpp>
#include <ros/time.h>
#include <deque>
#include <algorithm>
#include <cmath>

// Signed least-squares velocity over a short timestamped window. Keep the raw
// two-sample speed separately: this estimate must never conceal a safety spike.
class TcpVelocityWindow
{
  struct Point { double t; KDL::Vector p; };
  std::deque<Point> points_;
public:
  KDL::Vector add(double t, const KDL::Vector& p, double window)
  {
    if(!points_.empty() && (t<=points_.back().t || t-points_.back().t>window*2)) points_.clear();
    points_.push_back({t,p});
    while(points_.size()>2 && t-points_.front().t>window) points_.pop_front();
    if(points_.size()<2) return KDL::Vector::Zero();
    double mean_t=0; KDL::Vector mean_p=KDL::Vector::Zero();
    for(const auto& point:points_) {mean_t+=point.t-t; mean_p=mean_p+point.p;}
    mean_t/=points_.size(); mean_p=mean_p/double(points_.size());
    double variance=0; KDL::Vector covariance=KDL::Vector::Zero();
    for(const auto& point:points_) {
      const double dt=point.t-t-mean_t;
      variance+=dt*dt; covariance=covariance+(point.p-mean_p)*dt;
    }
    return variance>1e-12 ? covariance/variance : KDL::Vector::Zero();
  }
};

inline KDL::Vector accelerationFeedforward(const KDL::Vector& velocity,const KDL::Vector& acceleration,
                                           double lead_time,double speed)
{
  auto lead=acceleration*lead_time;
  if(lead.Norm()>.1*speed) lead=lead*(.1*speed/lead.Norm());
  auto result=velocity+lead;
  if(result.Norm()>.8*speed) result=result*(.8*speed/result.Norm());
  return result;
}

// Integral is a velocity correction, bounded by the same correction reserve as P.
// Conditional integration prevents windup when correction or total speed saturates.
class TrackingCorrection
{
  KDL::Vector integral_{KDL::Vector::Zero()};
  bool holding_{false};
  static KDL::Vector limited(const KDL::Vector& value,double limit)
  {
    const double norm=value.Norm(); return norm>limit ? value*(limit/norm) : value;
  }
public:
  void reset() {integral_=KDL::Vector::Zero(); holding_=false;}
  KDL::Vector step(const KDL::Vector& feedforward,const KDL::Vector& error,double dt,
                   double kp,double ki,double speed,double fraction,bool settling,double tolerance,
                   const KDL::Vector& velocity_error=KDL::Vector::Zero(),double kd=0.)
  {
    const double budget=speed*fraction;
    if(settling) {
      integral_=integral_*std::exp(-dt/.5);
      if(error.Norm()<=.5*tolerance) holding_=true;
      else if(error.Norm()>tolerance) holding_=false;
      if(holding_) {integral_=KDL::Vector::Zero(); return KDL::Vector::Zero();}
    } else {
      const auto candidate=limited(integral_+error*(ki*dt),budget);
      const auto correction=error*kp+candidate+velocity_error*kd;
      const auto total=feedforward+correction;
      const bool saturating=correction.Norm()>budget || total.Norm()>speed;
      // Permit unwinding an existing integrator even while a limit is active.
      if(!saturating || KDL::dot(error,integral_)<0) integral_=candidate;
    }
    return limited(feedforward+limited(error*kp+integral_+velocity_error*kd,budget),speed);
  }
};
