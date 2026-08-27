#include <algorithm>
#include <cstdint>
#include <cmath>
#include <string>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <wheeltec_multi/FormationStatus.h>
#include <wheeltec_multi/LeaderState.h>

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
      initial_pose_received_(false), localization_converged_(false),
      require_initial_pose_(true), require_localization_convergence_(true),
      avoidance_active_(false), sequence_received_(false),
      leader_source_stamp_valid_(false),
      leader_x_(0.0), leader_y_(0.0), leader_yaw_(0.0),
      leader_vx_(0.0), leader_vy_(0.0), leader_wz_(0.0),
      current_vx_(0.0), current_vy_(0.0), current_wz_(0.0),
      integral_map_x_(0.0), integral_map_y_(0.0), leader_sequence_(0)
  {
    private_nh_.param("multi_mode", multi_mode_, 2);
    private_nh_.param("slave_x", slave_x_, -0.8);
    private_nh_.param("slave_y", slave_y_, 0.8);
    private_nh_.param("k_x", k_x_, 1.0);
    private_nh_.param("k_y", k_y_, 1.0);
    private_nh_.param("k_yaw", k_yaw_, 1.0);
    private_nh_.param("k_i_x", k_i_x_, 0.08);
    private_nh_.param("k_i_y", k_i_y_, 0.08);
    private_nh_.param("integral_limit", integral_limit_, 0.35);
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
    private_nh_.param("recovery_error", recovery_error_, 0.25);
    private_nh_.param("recovery_gain_scale", recovery_gain_scale_, 1.4);
    private_nh_.param("tf_timeout", tf_timeout_, 0.2);
    private_nh_.param("control_rate", control_rate_, 20.0);
    private_nh_.param("require_initial_pose", require_initial_pose_, true);
    private_nh_.param("require_localization_convergence",
                      require_localization_convergence_, true);
    private_nh_.param("max_position_stddev", max_position_stddev_, 0.35);
    private_nh_.param("max_yaw_stddev", max_yaw_stddev_, 0.35);
    private_nh_.param<std::string>("formation_frame", formation_frame_, "leader");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");

    control_rate_ = std::max(1.0, control_rate_);
    leader_timeout_ = std::max(0.0, leader_timeout_);
    prediction_horizon_ = std::max(0.0, prediction_horizon_);
    position_tolerance_ = std::max(0.0, position_tolerance_);
    yaw_tolerance_ = std::max(0.0, yaw_tolerance_);
    integral_limit_ = std::max(0.0, integral_limit_);
    recovery_error_ = std::max(position_tolerance_, recovery_error_);
    recovery_gain_scale_ = std::max(1.0, recovery_gain_scale_);
    max_position_stddev_ = std::max(0.0, max_position_stddev_);
    max_yaw_stddev_ = std::max(0.0, max_yaw_stddev_);
    tf_timeout_ = std::max(0.0, tf_timeout_);
    if (multi_mode_ != 1 && multi_mode_ != 2)
    {
      ROS_WARN("Invalid multi_mode=%d; using mode 2", multi_mode_);
      multi_mode_ = 2;
    }

    if (formation_frame_ != "leader" && formation_frame_ != "map")
    {
      ROS_WARN("Invalid formation_frame=%s; using leader", formation_frame_.c_str());
      formation_frame_ = "leader";
    }

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_ori", 10);
    status_pub_ = nh_.advertise<wheeltec_multi::FormationStatus>("formation_status", 10);
    leader_sub_ = nh_.subscribe("leader_state", 1, &MecanumFollower::leaderCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1,
                                     &MecanumFollower::initialPoseCallback, this);
    amcl_pose_sub_ = nh_.subscribe("amcl_pose", 1,
                                  &MecanumFollower::amclPoseCallback, this);
    avoidance_sub_ = nh_.subscribe("avoidance_active", 1,
                                  &MecanumFollower::avoidanceCallback, this);
    last_loop_time_ = ros::Time::now();
    ROS_INFO("Mecanum formation controller enabled (formation_frame=%s, mode=%d)",
             formation_frame_.c_str(), multi_mode_);
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

      const bool receive_stale = leader_received_
          && (now - last_leader_receive_time_).toSec() > leader_timeout_;
      const bool measurement_stale = leader_received_ && leader_source_stamp_valid_
          && (now - leader_measurement_time_).toSec() > leader_timeout_;
      if (!leader_received_ || receive_stale || measurement_stale)
      {
        stopImmediately("leader_state timeout or stale measurement");
        publishWaitingStatus(now);
        rate.sleep();
        continue;
      }

      if ((require_initial_pose_ && !initial_pose_received_) ||
          (require_localization_convergence_ && !localization_converged_))
      {
        stopImmediately("waiting for initialized and converged localization");
        publishWaitingStatus(now);
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

      // Extrapolate the UDP leader state between packets. The horizon
      // prevents a stale packet from being projected too far into the future.
      const double packet_age = std::max(0.0, (now - leader_measurement_time_).toSec());
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

      // slave_x is forward and slave_y is left when formation_frame=leader.
      // formation_frame=map intentionally keeps the offset fixed in map.
      double offset_map_x = slave_x_;
      double offset_map_y = slave_y_;
      if (formation_frame_ == "leader")
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

      // A leader-relative rotating slot also has omega cross offset velocity.
      double feedforward_map_x = leader_map_vx;
      double feedforward_map_y = leader_map_vy;
      if (formation_frame_ == "leader")
      {
        feedforward_map_x -= leader_wz_ * offset_map_y;
        feedforward_map_y += leader_wz_ * offset_map_x;
      }
      const double feedforward_body_x = std::cos(follower_yaw) * feedforward_map_x
                                      + std::sin(follower_yaw) * feedforward_map_y;
      const double feedforward_body_y = -std::sin(follower_yaw) * feedforward_map_x
                                      + std::cos(follower_yaw) * feedforward_map_y;

      const double position_error = std::hypot(error_map_x, error_map_y);
      const double controlled_error_x = position_error > position_tolerance_
                                      ? error_body_x : 0.0;
      const double controlled_error_y = position_error > position_tolerance_
                                      ? error_body_y : 0.0;
      const double raw_yaw_error = normalizeAngle(predicted_leader_yaw - follower_yaw);
      const double controlled_yaw_error = std::fabs(raw_yaw_error) > yaw_tolerance_
                                        ? raw_yaw_error : 0.0;

      const bool recovering = position_error > recovery_error_;
      const double gain_scale = recovering ? recovery_gain_scale_ : 1.0;

      if (avoidance_active_)
      {
        integral_map_x_ = 0.0;
        integral_map_y_ = 0.0;
      }
      else
      {
        const double controlled_map_x = position_error > position_tolerance_
                                      ? error_map_x : 0.0;
        const double controlled_map_y = position_error > position_tolerance_
                                      ? error_map_y : 0.0;
        updateIntegral(integral_map_x_, controlled_map_x, dt);
        updateIntegral(integral_map_y_, controlled_map_y, dt);
      }
      const double integral_body_x = std::cos(follower_yaw) * integral_map_x_
                                   + std::sin(follower_yaw) * integral_map_y_;
      const double integral_body_y = -std::sin(follower_yaw) * integral_map_x_
                                   + std::cos(follower_yaw) * integral_map_y_;

      double target_vx = feedforward_body_x
                       + gain_scale * k_x_ * controlled_error_x
                       + k_i_x_ * integral_body_x;
      double target_vy = feedforward_body_y
                       + gain_scale * k_y_ * controlled_error_y
                       + k_i_y_ * integral_body_y;
      double target_wz = leader_wz_ + k_yaw_ * controlled_yaw_error;

      // Outside the tolerance, overcome the chassis dead zone instead of
      // permanently stopping with a small residual position error.
      target_vx = enforceMinimumCorrection(target_vx, controlled_error_x, min_vel_x_);
      target_vy = enforceMinimumCorrection(target_vy, controlled_error_y, min_vel_y_);

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
      publishStatus(now, avoidance_active_ ? wheeltec_multi::FormationStatus::AVOIDING
                                           : (recovering ? wheeltec_multi::FormationStatus::RECOVERING
                                                         : wheeltec_multi::FormationStatus::TRACKING),
                    packet_age, target_x, target_y, predicted_leader_yaw,
                    follower_x, follower_y, follower_yaw,
                    error_body_x, error_body_y, raw_yaw_error, cmd);
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
    initial_pose_received_ = true;
    ROS_INFO("Follower localization initialized from /initialpose in frame %s",
             msg->header.frame_id.c_str());
  }

  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    const double position_stddev = std::sqrt(std::max(
        0.0, std::max(msg->pose.covariance[0], msg->pose.covariance[7])));
    const double yaw_stddev = std::sqrt(std::max(0.0, msg->pose.covariance[35]));
    localization_converged_ = position_stddev <= max_position_stddev_
                           && yaw_stddev <= max_yaw_stddev_;
    if (!localization_converged_)
      ROS_WARN_THROTTLE(2.0, "AMCL not converged: position_stddev=%.3f yaw_stddev=%.3f",
                        position_stddev, yaw_stddev);
  }

  void avoidanceCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    avoidance_active_ = msg->data;
  }

  void leaderCallback(const wheeltec_multi::LeaderState::ConstPtr& msg)
  {
    const double values[] = {msg->pose.x, msg->pose.y, msg->pose.theta,
                             msg->twist.linear.x, msg->twist.linear.y,
                             msg->twist.angular.z};
    for (std::size_t i = 0; i < 6; ++i)
    {
      if (!std::isfinite(values[i]))
      {
        ROS_WARN_THROTTLE(1.0, "leader_state contains a non-finite value");
        return;
      }
    }
    const bool out_of_order = msg->sequence != 0 && sequence_received_
        && static_cast<std::int32_t>(msg->sequence - leader_sequence_) <= 0;
    const bool sender_may_have_restarted = leader_received_
        && (ros::Time::now() - last_leader_receive_time_).toSec() > leader_timeout_;
    if (out_of_order && !sender_may_have_restarted)
    {
      ROS_WARN_THROTTLE(1.0, "dropping stale/out-of-order leader packet");
      return;
    }
    if (out_of_order)
      ROS_WARN("leader sequence restarted after a timeout; accepting new sequence");
    leader_x_ = values[0];
    leader_y_ = values[1];
    leader_yaw_ = values[2];
    leader_vx_ = values[3];
    leader_vy_ = values[4];
    leader_wz_ = values[5];
    leader_sequence_ = msg->sequence;
    sequence_received_ = msg->sequence != 0;
    leader_source_stamp_valid_ = msg->source_stamp_valid;
    last_leader_receive_time_ = ros::Time::now();
    leader_measurement_time_ = msg->source_stamp_valid ? msg->header.stamp
                                                       : msg->received_at;
    leader_received_ = true;
  }

  void updateIntegral(double& integral, double error, double dt)
  {
    if (error == 0.0)
      integral = 0.0;
    else
      integral = std::max(-integral_limit_,
                          std::min(integral_limit_, integral + error * dt));
  }

  double enforceMinimumCorrection(double target, double controlled_error,
                                  double minimum) const
  {
    minimum = std::fabs(minimum);
    if (controlled_error != 0.0 && std::fabs(target) < minimum)
      return std::copysign(minimum, controlled_error);
    return target;
  }

  void publishWaitingStatus(const ros::Time& now)
  {
    geometry_msgs::Twist stop;
    publishStatus(now, wheeltec_multi::FormationStatus::WAITING, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, stop);
  }

  void publishStatus(const ros::Time& now, std::uint8_t state, double leader_age,
                     double target_x, double target_y, double target_yaw,
                     double follower_x, double follower_y, double follower_yaw,
                     double error_x, double error_y, double error_yaw,
                     const geometry_msgs::Twist& command)
  {
    wheeltec_multi::FormationStatus status;
    status.header.stamp = now;
    status.header.frame_id = map_frame_;
    status.state = state;
    status.leader_sequence = leader_sequence_;
    status.leader_age = leader_age;
    status.localization_initialized = (!require_initial_pose_ || initial_pose_received_)
                                   && (!require_localization_convergence_
                                       || localization_converged_);
    status.target.x = target_x;
    status.target.y = target_y;
    status.target.theta = target_yaw;
    status.follower.x = follower_x;
    status.follower.y = follower_y;
    status.follower.theta = follower_yaw;
    status.error_body.x = error_x;
    status.error_body.y = error_y;
    status.error_body.z = error_yaw;
    status.command = command;
    status_pub_.publish(status);
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
    integral_map_x_ = integral_map_y_ = 0.0;
    cmd_pub_.publish(geometry_msgs::Twist());
  }

  void stopImmediately(const char* reason)
  {
    ROS_WARN_THROTTLE(1.0, "Mecanum follower stopped: %s", reason);
    publishStop();
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher cmd_pub_, status_pub_;
  ros::Subscriber leader_sub_, initial_pose_sub_, amcl_pose_sub_, avoidance_sub_;
  tf::TransformListener listener_;
  bool leader_received_, initial_pose_received_, localization_converged_;
  bool require_initial_pose_, require_localization_convergence_, avoidance_active_;
  bool sequence_received_, leader_source_stamp_valid_;
  ros::Time last_leader_receive_time_, leader_measurement_time_, last_loop_time_;
  double leader_x_, leader_y_, leader_yaw_, leader_vx_, leader_vy_, leader_wz_;
  double current_vx_, current_vy_, current_wz_, integral_map_x_, integral_map_y_;
  std::uint32_t leader_sequence_;
  int multi_mode_;
  double slave_x_, slave_y_, k_x_, k_y_, k_yaw_, k_i_x_, k_i_y_, integral_limit_;
  double max_vel_x_, max_vel_y_, max_vel_theta_, max_linear_vel_;
  double min_vel_x_, min_vel_y_, min_vel_theta_;
  double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
  double decel_lim_x_, decel_lim_y_, decel_lim_theta_;
  double leader_timeout_, prediction_horizon_, position_tolerance_, yaw_tolerance_;
  double recovery_error_, recovery_gain_scale_, max_position_stddev_, max_yaw_stddev_;
  double tf_timeout_, control_rate_;
  std::string formation_frame_, map_frame_, base_frame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slave_tf_listener_mecanum");
  MecanumFollower follower;
  follower.spin();
  return 0;
}
