#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/MarkerArray.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
static double age(Time t) { return std::chrono::duration<double>(Clock::now()-t).count(); }
static constexpr double PI = 3.14159265358979323846;

struct Curve
{
  std::string shape, plane;
  double radius, amplitude, length, duration, kp, speed, error, extent, tolerance, settle_speed, settle_timeout;
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

class Tracker
{
  ros::NodeHandle nh_, pn_{"~"};
  ros::Publisher cmd_, state_pub_, metrics_, reference_pub_, actual_pub_, markers_;
  ros::Subscriber joints_, jog_state_sub_;
  ros::ServiceClient enable_;
  ros::ServiceServer preview_srv_, start_srv_, stop_srv_;
  ros::WallTimer timer_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::vector<std::string> joint_names_;
  KDL::Frame actual_;
  KDL::Vector origin_, target_, previous_actual_;
  Curve curve_;
  nav_msgs::Path reference_, trace_;
  std::string base_, tool_, jog_ns_, jog_state_, state_{"IDLE"}, detail_{"Preview a curve to begin"};
  Time feedback_time_{}, state_time_{}, tick_time_{Clock::now()}, warmup_time_{}, settle_time_{}, stable_time_{};
  ros::Time feedback_stamp_;
  double feedback_timeout_, state_timeout_, elapsed_{0}, peak_{0}, sum2_{0}, actual_speed_{0};
  size_t samples_{0};
  bool feedback_valid_{false}, have_feedback_{false}, previewed_{false}, active_{false}, owned_{false};

  Curve load()
  {
    Curve c;
    pn_.param<std::string>("shape",c.shape,"circle"); pn_.param<std::string>("plane",c.plane,"xy");
    pn_.param("radius",c.radius,0.03); pn_.param("amplitude",c.amplitude,0.02);
    pn_.param("length",c.length,0.10); pn_.param("waves",c.waves,1); pn_.param("cycles",c.cycles,1);
    pn_.param("duration",c.duration,40.0); pn_.param("kp",c.kp,0.5);
    pn_.param("max_speed",c.speed,0.015); pn_.param("max_error",c.error,0.04); pn_.param("max_extent",c.extent,0.15);
    pn_.param("settle_tolerance",c.tolerance,0.002); pn_.param("settle_speed",c.settle_speed,0.003);
    pn_.param("settle_timeout",c.settle_timeout,15.0);
    for(double x : {c.radius,c.amplitude,c.length,c.duration,c.kp,c.speed,c.error,c.extent,c.tolerance,c.settle_speed,c.settle_timeout})
      if(!std::isfinite(x) || x<=0) throw std::runtime_error("All numeric parameters must be finite and positive");
    if((c.shape!="circle" && c.shape!="sine") || (c.plane!="xy" && c.plane!="xz" && c.plane!="yz") ||
       c.waves<1 || c.waves>20 || c.cycles<1 || c.cycles>20 || c.duration<2 || c.duration>3600 || c.speed>0.10 || c.extent>0.5)
      throw std::runtime_error("Invalid shape/plane or parameters outside supported bounds");
    // Analytic upper bounds (max quintic phase speed = 1.875/T), not a sparse speed check.
    const double path_bound = c.shape=="circle" ? 2*c.radius : std::hypot(c.length,c.amplitude);
    const double speed_bound = 1.875/c.duration*(c.shape=="circle" ? 2*PI*c.cycles*c.radius : std::hypot(c.length,2*PI*c.waves*c.amplitude));
    if(path_bound>c.extent || speed_bound>0.8*c.speed || c.tolerance>=c.error)
      throw std::runtime_error("Curve exceeds extent/speed budget (reserve 20% for correction); increase duration or reduce size");
    return c;
  }
  bool healthy() const
  {
    const double stamp_age = (ros::Time::now()-feedback_stamp_).toSec();
    return feedback_valid_ && age(feedback_time_)<=feedback_timeout_ && stamp_age>=-0.05 && stamp_age<=feedback_timeout_ &&
           age(state_time_)<=state_timeout_;
  }
  geometry_msgs::PoseStamped pose(const KDL::Vector& p) const
  {
    geometry_msgs::PoseStamped out; out.header.frame_id=base_; out.header.stamp=ros::Time::now();
    out.pose.position.x=p.x(); out.pose.position.y=p.y(); out.pose.position.z=p.z();
    actual_.M.GetQuaternion(out.pose.orientation.x,out.pose.orientation.y,out.pose.orientation.z,out.pose.orientation.w);
    return out;
  }
  void command(const KDL::Vector& v=KDL::Vector::Zero())
  {
    geometry_msgs::TwistStamped msg; msg.header.frame_id=base_; msg.header.stamp=ros::Time::now();
    msg.twist.linear.x=v.x(); msg.twist.linear.y=v.y(); msg.twist.linear.z=v.z(); cmd_.publish(msg);
  }
  bool disable()
  {
    if(!owned_) return true;
    std_srvs::SetBool srv; srv.request.data=false;
    if(!enable_.call(srv) || !srv.response.success) return false;
    owned_=false; return true;
  }
  void finish(const std::string& state, const std::string& why)
  {
    active_=false; previewed_=false;
    if(owned_) command();
    actual_pub_.publish(trace_);
    state_=state; detail_=why;
    if(!disable()) { state_="FAULT"; detail_="Disable not acknowledged; retrying: "+why; }
    ROS_INFO_STREAM("[tracking] "<<state_<<": "<<detail_);
  }
  void feedback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    feedback_valid_=false;
    const double delay=(ros::Time::now()-msg->header.stamp).toSec();
    if(msg->header.stamp.isZero() || delay>feedback_timeout_ || delay < -0.05 ||
       msg->name.size()!=msg->position.size() || (have_feedback_ && msg->header.stamp<=feedback_stamp_)) return;
    KDL::JntArray q(joint_names_.size());
    for(size_t i=0;i<joint_names_.size();++i) {
      auto it=std::find(msg->name.begin(),msg->name.end(),joint_names_[i]);
      if(it==msg->name.end()) return;
      q(i)=msg->position[it-msg->name.begin()]; if(!std::isfinite(q(i))) return;
    }
    KDL::Frame frame;
    if(fk_->JntToCart(q,frame)<0) return;
    actual_speed_=have_feedback_ ? (frame.p-previous_actual_).Norm()/(msg->header.stamp-feedback_stamp_).toSec() : 0;
    if(!have_feedback_) target_=frame.p;
    actual_=frame; previous_actual_=frame.p; feedback_stamp_=msg->header.stamp;
    feedback_time_=Clock::now(); have_feedback_=feedback_valid_=true;
  }
  bool preview(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    if(active_ || owned_) {res.message="Stop the current run first"; return true;}
    previewed_=false;
    if(!healthy() || jog_state_!="DISABLED") {res.message="Fresh feedback and DISABLED jog state required"; return true;}
    try {curve_=load();} catch(const std::exception& e) {res.message=e.what(); return true;}
    origin_=actual_.p; target_=origin_;
    reference_.header.frame_id=trace_.header.frame_id=base_; reference_.header.stamp=ros::Time::now();
    reference_.poses.clear(); trace_.poses.clear();
    for(int i=0;i<=1000;++i) {KDL::Vector p,v; curve_.sample(curve_.duration*i/1000.0,p,v); reference_.poses.push_back(pose(origin_+p));}
    reference_pub_.publish(reference_); actual_pub_.publish(trace_);
    elapsed_=peak_=sum2_=0; samples_=0;
    previewed_=true; state_="PREVIEW"; detail_="Preview only; Start explicitly to move";
    res.success=true; res.message="Preview captured at current TCP; configuration frozen until next Preview"; return true;
  }
  bool start(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    if(active_ || owned_ || !previewed_ || !healthy() || jog_state_!="DISABLED" || (actual_.p-origin_).Norm()>curve_.tolerance) {
      res.message="Preview required, TCP must remain at preview origin, jog must be DISABLED with fresh feedback"; return true;
    }
    if(cmd_.getNumSubscribers()!=1) {res.message="Expected exactly one jog command subscriber"; return true;}
    std_srvs::SetBool srv; srv.request.data=true;
    // A lost service response does not prove the request was never applied.
    owned_=true;
    if(!enable_.call(srv) || !srv.response.success) {
      finish("FAULT","Jog enable failed: "+srv.response.message); res.message=detail_; return true;
    }
    owned_=active_=true; previewed_=false; state_="ARMING"; detail_="Sending neutral commands; waiting for fresh enable confirmation";
    warmup_time_=Clock::now(); tick_time_=Clock::now(); command(); res.success=true; res.message=detail_; return true;
  }
  bool stop(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    finish("STOPPED","Stop requested; queued SDK points and deceleration remain");
    res.success=!owned_; res.message=detail_; return true;
  }
  void visualize()
  {
    std_msgs::String status; status.data=state_+": "+detail_; state_pub_.publish(status);
    if(!have_feedback_) return;
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker m; m.header.frame_id=base_; m.header.stamp=ros::Time::now(); m.ns="tracking";
    m.action=m.ADD; m.pose.orientation.w=1; m.id=0; m.type=m.SPHERE; m.pose.position=pose(target_).pose.position;
    m.scale.x=m.scale.y=m.scale.z=0.009; m.color.r=1; m.color.g=0.7; m.color.a=1; arr.markers.push_back(m);
    m.id=1; m.type=m.LINE_LIST; m.pose.position=geometry_msgs::Point(); m.scale.x=0.002;
    m.color.r=1; m.color.g=0.25; m.points={pose(target_).pose.position,pose(actual_.p).pose.position}; arr.markers.push_back(m);
    m.id=2; m.type=m.TEXT_VIEW_FACING; m.points.clear(); m.pose.position=pose(actual_.p+KDL::Vector(0,0,0.10)).pose.position;
    m.scale.z=0.025; m.color.r=m.color.g=m.color.b=1;
    m.text=state_+"  "+std::to_string(static_cast<int>(1000*(target_-actual_.p).Norm()))+" mm"; arr.markers.push_back(m); markers_.publish(arr);
    std_msgs::Float64MultiArray metrics;
    metrics.data={elapsed_,(target_-actual_.p).Norm(),peak_,samples_?std::sqrt(sum2_/samples_):0,actual_speed_}; metrics_.publish(metrics);
  }
  void tick(const ros::WallTimerEvent&)
  {
    const double dt=age(tick_time_); tick_time_=Clock::now();
    if(!active_) { if(owned_) {command(); disable();} visualize(); return; }
    if(dt>0.10 || !healthy()) {finish("FAULT","Tick stalled or feedback/controller state stale"); visualize(); return;}
    if(state_=="ARMING") {
      command();
      if(jog_state_.find("FAULT")==0 || age(warmup_time_)>1.0) finish("FAULT","Enable confirmation failed");
      else if(age(warmup_time_)>=0.3 && (jog_state_=="ENABLED_IDLE" || jog_state_=="JOGGING") && state_time_>warmup_time_) {
        state_="RUNNING"; detail_="Tracking with feedforward + TCP position correction";
      }
      visualize(); return;
    }
    if(jog_state_!="ENABLED_IDLE" && jog_state_!="JOGGING") {finish("FAULT","Jog controller left enabled state: "+jog_state_); visualize(); return;}
    elapsed_=std::min(curve_.duration,elapsed_+dt);
    KDL::Vector p,v; curve_.sample(elapsed_,p,v); target_=origin_+p;
    const KDL::Vector error=target_-actual_.p; const double e=error.Norm();
    peak_=std::max(peak_,e); sum2_+=e*e; ++samples_;
    if(e>curve_.error) {finish("FAULT","TCP tracking error exceeded limit"); visualize(); return;}
    v=v+error*curve_.kp; if(v.Norm()>curve_.speed) v=v*(curve_.speed/v.Norm());
    command(v);
    trace_.header.stamp=ros::Time::now(); trace_.poses.push_back(pose(actual_.p));
    if(trace_.poses.size()>10000) trace_.poses.erase(trace_.poses.begin());
    if(samples_%5==0) actual_pub_.publish(trace_);
    if(elapsed_>=curve_.duration) {
      if(state_!="SETTLING") {state_="SETTLING"; settle_time_=Clock::now(); stable_time_=Time{};}
      const bool stable=e<=curve_.tolerance && actual_speed_<=curve_.settle_speed;
      if(!stable) stable_time_=Time{};
      else if(stable_time_==Time{}) stable_time_=Clock::now();
      if(stable && age(stable_time_)>=0.5) finish("COMPLETED","Final TCP within tolerance for 0.5 s; disable requested");
      else if(age(settle_time_)>curve_.settle_timeout) finish("FAULT","Final TCP convergence timed out");
    }
    visualize();
  }
public:
  Tracker()
  {
    pn_.param<std::string>("base_link",base_,"base_link"); pn_.param<std::string>("tool_link",tool_,"tcp_link");
    pn_.param<std::string>("jog_ns",jog_ns_,"/cartesian_jog");
    pn_.param("feedback_timeout",feedback_timeout_,0.15); pn_.param("state_timeout",state_timeout_,0.35);
    if(!std::isfinite(feedback_timeout_) || feedback_timeout_<=0 || !std::isfinite(state_timeout_) || state_timeout_<=0)
      throw std::runtime_error("Invalid watchdog timeouts");
    std::string xml; nh_.getParam("robot_description",xml); KDL::Tree tree;
    if(!kdl_parser::treeFromString(xml,tree) || !tree.getChain(base_,tool_,chain_)) throw std::runtime_error("Cannot construct base-to-TCP KDL chain");
    for(unsigned i=0;i<chain_.getNrOfSegments();++i) if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None)
      joint_names_.push_back(chain_.getSegment(i).getJoint().getName());
    fk_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    cmd_=nh_.advertise<geometry_msgs::TwistStamped>(jog_ns_+"/command",1);
    state_pub_=pn_.advertise<std_msgs::String>("state",1,true); metrics_=pn_.advertise<std_msgs::Float64MultiArray>("metrics",1);
    reference_pub_=pn_.advertise<nav_msgs::Path>("reference_path",1,true); actual_pub_=pn_.advertise<nav_msgs::Path>("actual_path",1,true);
    markers_=pn_.advertise<visualization_msgs::MarkerArray>("markers",1);
    std::string topic; pn_.param<std::string>("joint_states_topic",topic,"/cartesian_jog/sdk_joint_states");
    joints_=nh_.subscribe(topic,1,&Tracker::feedback,this);
    jog_state_sub_=nh_.subscribe<std_msgs::String>(jog_ns_+"/state",1,[this](const std_msgs::String::ConstPtr& s){jog_state_=s->data; state_time_=Clock::now();});
    enable_=nh_.serviceClient<std_srvs::SetBool>(jog_ns_+"/set_enabled");
    preview_srv_=pn_.advertiseService("preview",&Tracker::preview,this); start_srv_=pn_.advertiseService("start",&Tracker::start,this);
    stop_srv_=pn_.advertiseService("stop",&Tracker::stop,this); timer_=nh_.createWallTimer(ros::WallDuration(0.02),&Tracker::tick,this);
  }
  ~Tracker() { if(owned_) {command(); disable();} }
};
int main(int argc,char** argv)
{
  ros::init(argc,argv,"trajectory_tracking");
  try {Tracker node; ros::spin();} catch(const std::exception& e) {ROS_FATAL("%s",e.what()); return 1;}
  return 0;
}
