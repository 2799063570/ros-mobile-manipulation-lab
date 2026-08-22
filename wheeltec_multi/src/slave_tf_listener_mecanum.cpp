#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>

namespace
{
double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double clampAbs(double value, double limit)
{
  limit = std::fabs(limit);
  return std::max(-limit, std::min(value, limit));
}

double approach(double current, double target, double max_change)
{
  const double delta = target - current;
  max_change = std::fabs(max_change);
  if (delta > max_change)
    return current + max_change;
  if (delta < -max_change)
    return current - max_change;
  return target;
}
}  // namespace

class MecanumFollower
{
public:
  MecanumFollower()
    : nh_(), private_nh_("~"), leader_received_(false),
      localization_initialized_(false), require_initial_pose_(true),
      leader_x_(0.0), leader_y_(0.0), leader_yaw_(0.0),
      leader_vx_(0.0), leader_vy_(0.0), leader_wz_(0.0),
      current_vx_(0.0), current_vy_(0.0), current_wz_(0.0)
  {
    private_nh_.param("multi_mode", multi_mode_, 2);
    private_nh_.param("slave_x", slave_x_, -0.8);
    private_nh_.param("slave_y", slave_y_, 0.8);
    private_nh_.param("k_x", k_x_, 1.0);
    private_nh_.param("k_y", k_y_, 1.0);
    private_nh_.param("k_yaw", k_yaw_, 1.0);
    private_nh_.param("max_vel_x", max_vel_x_, 0.8);
    private_nh_.param("max_vel_y", max_vel_y_, 0.8);
    private_nh_.param("max_vel_theta", max_vel_theta_, 0.8);
    private_nh_.param("max_linear_vel", max_linear_vel_, 0.8);
    private_nh_.param("min_vel_x", min_vel_x_, 0.0);
    private_nh_.param("min_vel_y", min_vel_y_, 0.0);
    private_nh_.param("min_vel_theta", min_vel_theta_, 0.0);
    private_nh_.param("acc_lim_x", acc_lim_x_, 0.5);
    private_nh_.param("acc_lim_y", acc_lim_y_, 0.5);
    private_nh_.param("acc_lim_theta", acc_lim_theta_, 1.0);
    private_nh_.param("decel_lim_x", decel_lim_x_, 0.8);
    private_nh_.param("decel_lim_y", decel_lim_y_, 0.8);
    private_nh_.param("decel_lim_theta", decel_lim_theta_, 1.5);
    private_nh_.param("leader_timeout", leader_timeout_, 0.4);
    private_nh_.param("prediction_horizon", prediction_horizon_, 0.15);
    private_nh_.param("position_tolerance", position_tolerance_, 0.03);
    private_nh_.param("yaw_tolerance", yaw_tolerance_, 0.03);
    private_nh_.param("tf_timeout", tf_timeout_, 0.2);
    private_nh_.param("control_rate", control_rate_, 20.0);
    private_nh_.param("require_initial_pose", require_initial_pose_, true);
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");

    control_rate_ = std::max(1.0, control_rate_);
    leader_timeout_ = std::max(0.0, leader_timeout_);
    prediction_horizon_ = std::max(0.0, prediction_horizon_);
    position_tolerance_ = std::max(0.0, position_tolerance_);
    yaw_tolerance_ = std::max(0.0, yaw_tolerance_);
    tf_timeout_ = std::max(0.0, tf_timeout_);
    if (multi_mode_ != 1 && multi_mode_ != 2)
    {
      ROS_WARN("Invalid multi_mode=%d; using mode 2", multi_mode_);
      multi_mode_ = 2;
    }

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_ori", 10);
    leader_sub_ = nh_.subscribe("/multfodom", 1, &MecanumFollower::leaderCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1,
                                     &MecanumFollower::initialPoseCallback, this);
    last_loop_time_ = ros::Time::now();
    ROS_INFO("Mecanum formation controller enabled (mode=%d)", multi_mode_);
  }

  ~MecanumFollower()
  {
    publishStop();
  }

  void spin()
  {
    ros::Rate rate(control_rate_);
    while (ros::ok())
    {
      ros::spinOnce();
      const ros::Time now = ros::Time::now();
      double dt = (now - last_loop_time_).toSec();
      if (dt <= 0.0 || dt > 1.0)
        dt = 1.0 / control_rate_;
      last_loop_time_ = now;

      if (!leader_received_ || (now - last_leader_time_).toSec() > leader_timeout_)
      {
        stopImmediately("leader /multfodom timeout");
        rate.sleep();
        continue;
      }

      if (require_initial_pose_ && !localization_initialized_)
      {
        stopImmediately("waiting for an explicit /initialpose");
        rate.sleep();
        continue;
      }

      tf::StampedTransform map_to_base;
      try
      {
        listener_.waitForTransform(map_frame_, base_frame_, ros::Time(0),
                                   ros::Duration(tf_timeout_));
        listener_.lookupTransform(map_frame_, base_frame_, ros::Time(0), map_to_base);
      }
      catch (const tf::TransformException& ex)
      {
        ROS_WARN_THROTTLE(1.0, "Mecanum follower TF unavailable: %s", ex.what());
        stopImmediately("TF unavailable");
        rate.sleep();
        continue;
      }

      const double follower_x = map_to_base.getOrigin().x();
      const double follower_y = map_to_base.getOrigin().y();
      const double follower_yaw = tf::getYaw(map_to_base.getRotation());

      // Extrapolate the 15 Hz UDP leader state between packets. The horizon
      // prevents a stale packet from being projected too far into the future.
      const double packet_age = std::max(0.0, (now - last_leader_time_).toSec());
      const double prediction_dt = std::min(packet_age, prediction_horizon_);
      const double prediction_yaw = leader_yaw_ + 0.5 * leader_wz_ * prediction_dt;
      const double leader_map_vx = leader_vx_ * std::cos(prediction_yaw)
                                 - leader_vy_ * std::sin(prediction_yaw);
      const double leader_map_vy = leader_vx_ * std::sin(prediction_yaw)
                                 + leader_vy_ * std::cos(prediction_yaw);
      const double predicted_leader_x = leader_x_ + leader_map_vx * prediction_dt;
      const double predicted_leader_y = leader_y_ + leader_map_vy * prediction_dt;
      const double predicted_leader_yaw = normalizeAngle(
          leader_yaw_ + leader_wz_ * prediction_dt);

      // slave_x is forward and slave_y is left. Mode 1 rotates the formation
      // with the leader; mode 2 keeps the positional offset fixed in map.
      double offset_map_x = slave_x_;
      double offset_map_y = slave_y_;
      if (multi_mode_ == 1)
      {
        offset_map_x = std::cos(predicted_leader_yaw) * slave_x_
                     - std::sin(predicted_leader_yaw) * slave_y_;
        offset_map_y = std::sin(predicted_leader_yaw) * slave_x_
                     + std::cos(predicted_leader_yaw) * slave_y_;
      }
      const double target_x = predicted_leader_x + offset_map_x;
      const double target_y = predicted_leader_y + offset_map_y;
      const double error_map_x = target_x - follower_x;
      const double error_map_y = target_y - follower_y;
      const double error_body_x = std::cos(follower_yaw) * error_map_x
                                + std::sin(follower_yaw) * error_map_y;
      const double error_body_y = -std::sin(follower_yaw) * error_map_x
                                + std::cos(follower_yaw) * error_map_y;

      // In mode 1, the rotating formation point also has omega cross offset
      // velocity. In mode 2, only the leader translational feed-forward applies.
      double feedforward_map_x = leader_map_vx;
      double feedforward_map_y = leader_map_vy;
      if (multi_mode_ == 1)
      {
        feedforward_map_x -= leader_wz_ * offset_map_y;
        feedforward_map_y += leader_wz_ * offset_map_x;
      }
      const double feedforward_body_x = std::cos(follower_yaw) * feedforward_map_x
                                      + std::sin(follower_yaw) * feedforward_map_y;
      const double feedforward_body_y = -std::sin(follower_yaw) * feedforward_map_x
                                      + std::cos(follower_yaw) * feedforward_map_y;

      const double controlled_error_x = std::fabs(error_body_x) > position_tolerance_
                                      ? error_body_x : 0.0;
      const double controlled_error_y = std::fabs(error_body_y) > position_tolerance_
                                      ? error_body_y : 0.0;
      const double raw_yaw_error = normalizeAngle(predicted_leader_yaw - follower_yaw);
      const double controlled_yaw_error = std::fabs(raw_yaw_error) > yaw_tolerance_
                                        ? raw_yaw_error : 0.0;

      double target_vx = feedforward_body_x + k_x_ * controlled_error_x;
      double target_vy = feedforward_body_y + k_y_ * controlled_error_y;
      double target_wz = leader_wz_ + k_yaw_ * controlled_yaw_error;

      target_vx = clampAbs(target_vx, max_vel_x_);
      target_vy = clampAbs(target_vy, max_vel_y_);
      target_wz = clampAbs(target_wz, max_vel_theta_);
      const double linear_speed = std::hypot(target_vx, target_vy);
      if (max_linear_vel_ > 0.0 && linear_speed > max_linear_vel_)
      {
        const double scale = max_linear_vel_ / linear_speed;
        target_vx *= scale;
        target_vy *= scale;
      }

      current_vx_ = rateLimited(current_vx_, target_vx, acc_lim_x_, decel_lim_x_, dt);
      current_vy_ = rateLimited(current_vy_, target_vy, acc_lim_y_, decel_lim_y_, dt);
      current_wz_ = rateLimited(current_wz_, target_wz, acc_lim_theta_, decel_lim_theta_, dt);

      if (std::fabs(current_vx_) < min_vel_x_ && std::fabs(target_vx) < min_vel_x_)
        current_vx_ = 0.0;
      if (std::fabs(current_vy_) < min_vel_y_ && std::fabs(target_vy) < min_vel_y_)
        current_vy_ = 0.0;
      if (std::fabs(current_wz_) < min_vel_theta_ && std::fabs(target_wz) < min_vel_theta_)
        current_wz_ = 0.0;

      geometry_msgs::Twist cmd;
      cmd.linear.x = current_vx_;
      cmd.linear.y = current_vy_;
      cmd.angular.z = current_wz_;
      cmd_pub_.publish(cmd);
      ROS_INFO_THROTTLE(1.0,
                        "formation error body=(%.3f, %.3f, %.3f), cmd=(%.3f, %.3f, %.3f)",
                        error_body_x, error_body_y, raw_yaw_error,
                        current_vx_, current_vy_, current_wz_);
      rate.sleep();
    }
  }

private:
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    if (msg->header.frame_id.empty())
    {
      ROS_WARN_THROTTLE(1.0, "Ignoring /initialpose with an empty frame_id");
      return;
    }
    localization_initialized_ = true;
    ROS_INFO("Follower localization initialized from /initialpose in frame %s",
             msg->header.frame_id.c_str());
  }

  void leaderCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
  {
    if (msg->data.size() < 6)
    {
      ROS_WARN_THROTTLE(1.0, "/multfodom requires 6 values; received %zu", msg->data.size());
      return;
    }
    for (std::size_t i = 0; i < 6; ++i)
    {
      if (!std::isfinite(msg->data[i]))
      {
        ROS_WARN_THROTTLE(1.0, "/multfodom contains a non-finite value");
        return;
      }
    }
    leader_x_ = msg->data[0];
    leader_y_ = msg->data[1];
    leader_yaw_ = msg->data[2];
    leader_vx_ = msg->data[3];
    leader_vy_ = msg->data[4];
    leader_wz_ = msg->data[5];
    last_leader_time_ = ros::Time::now();
    leader_received_ = true;
  }

  double rateLimited(double current, double target, double acceleration,
                     double deceleration, double dt) const
  {
    const bool slowing = std::fabs(target) < std::fabs(current) || current * target < 0.0;
    const double limit = slowing ? std::fabs(deceleration) : std::fabs(acceleration);
    return approach(current, target, limit * dt);
  }

  void publishStop()
  {
    current_vx_ = current_vy_ = current_wz_ = 0.0;
    cmd_pub_.publish(geometry_msgs::Twist());
  }

  void stopImmediately(const char* reason)
  {
    ROS_WARN_THROTTLE(1.0, "Mecanum follower stopped: %s", reason);
    publishStop();
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber leader_sub_, initial_pose_sub_;
  tf::TransformListener listener_;
  bool leader_received_, localization_initialized_, require_initial_pose_;
  ros::Time last_leader_time_, last_loop_time_;
  double leader_x_, leader_y_, leader_yaw_, leader_vx_, leader_vy_, leader_wz_;
  double current_vx_, current_vy_, current_wz_;
  int multi_mode_;
  double slave_x_, slave_y_, k_x_, k_y_, k_yaw_;
  double max_vel_x_, max_vel_y_, max_vel_theta_, max_linear_vel_;
  double min_vel_x_, min_vel_y_, min_vel_theta_;
  double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
  double decel_lim_x_, decel_lim_y_, decel_lim_theta_;
  double leader_timeout_, prediction_horizon_, position_tolerance_, yaw_tolerance_;
  double tf_timeout_, control_rate_;
  std::string map_frame_, base_frame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slave_tf_listener_mecanum");
  MecanumFollower follower;
  follower.spin();
  return 0;
}
