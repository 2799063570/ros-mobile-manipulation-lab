#include <aubo_ros_control/visual_servo.h>
#include <Eigen/Geometry>
#include <kdl/jntarray.hpp>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <shape_msgs/SolidPrimitive.h>
#include <algorithm>
#include <cmath>
#include <set>

namespace aubo_ros_control {
namespace {
using visual_servo_internal::kDof;
using visual_servo_internal::JointPoint;
double jointDistance(const JointPoint &a, const JointPoint &b) {
  double error = 0.0;
  for (std::size_t i = 0; i < kDof; ++i)
    error = std::max(error, std::abs(a[i] - b[i]));
  return error;
}
Eigen::Vector3d translation(const geometry_msgs::Pose &p) {
  return Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
}
} // namespace

bool VisualServo::loadHybridParameters() {
  private_nh_.param("hybrid_enabled", hybrid_enabled_, false);
  if (!hybrid_enabled_)
    return true;
  private_nh_.param<std::string>("planning_group", planning_group_, "aubo_i5");
  private_nh_.param<std::string>("planning_service", planning_service_, "/plan_kinematic_path");
  private_nh_.param("hybrid_enter_distance", hybrid_enter_, 0.10);
  private_nh_.param("hybrid_exit_distance", hybrid_exit_, 0.16);
  private_nh_.param("hybrid_standoff", hybrid_standoff_, 0.06);
  private_nh_.param("hybrid_target_drift", hybrid_target_drift_, 0.04);
  private_nh_.param("hybrid_joint_error", hybrid_joint_error_, 0.08);
  private_nh_.param("hybrid_planning_time", hybrid_planning_time_, 3.0);
  private_nh_.param("hybrid_execution_timeout", hybrid_execution_timeout_, 30.0);
  for (double value : {hybrid_enter_, hybrid_exit_, hybrid_standoff_,
                       hybrid_target_drift_, hybrid_joint_error_,
                       hybrid_planning_time_, hybrid_execution_timeout_}) {
    if (!std::isfinite(value) || value <= 0.0) {
      ROS_ERROR("[hybrid] All hybrid limits must be finite and positive");
      return false;
    }
  }
  if (hybrid_standoff_ >= hybrid_enter_ || hybrid_enter_ >= hybrid_exit_ ||
      planning_group_.empty() || planning_service_.empty()) {
    ROS_ERROR("[hybrid] Require 0 < standoff < enter_distance < exit_distance");
    return false;
  }
  return true;
}

// Called only under control_mutex_ (or during construction). The generation
// invalidates a blocking planning response after reset, disable, or target loss.
void VisualServo::resetHybrid() {
  ++hybrid_generation_;
  hybrid_pending_ = hybrid_fault_ = hybrid_near_ = false;
  hybrid_points_.clear();
  hybrid_times_.clear();
  hybrid_elapsed_ = 0.0;
  hybrid_settle_since_ = ros::Time();
  hybrid_plan_started_ = ros::Time();
  alignment_candidate_ = aligned_latched_ = false;
}

void VisualServo::holdFeedback(const JointPoint &feedback) {
  queue_.clear();
  command_position_ = last_output_ = feedback;
  command_velocity_.fill(0.0);
  backend_velocity_.fill(0.0);
  last_tracking_velocity_.setZero();
  queue_.push(feedback);
}

bool VisualServo::hybridGoal(const JointPoint &feedback,
                            const geometry_msgs::Pose &target,
                            geometry_msgs::Pose &goal, double &distance) {
  KDL::JntArray joints(kDof);
  for (std::size_t i = 0; i < kDof; ++i)
    joints(i) = feedback[i];
  KDL::Frame frame;
  if (fk_solver_->JntToCart(joints, frame) < 0)
    return false;
  Eigen::Matrix3d rotation;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      rotation(r, c) = frame.M(r, c);
  Eigen::Vector3d current(frame.p.x(), frame.p.y(), frame.p.z());
  Eigen::Matrix3d goal_rotation = use_orientation_control_ ? desired_rotation_ : rotation;
  Eigen::Vector3d destination;
  if (servo_mode_ == "eye_in_hand") {
    if (use_orientation_control_) {
      Eigen::Quaterniond observed(target.orientation.w, target.orientation.x,
                                  target.orientation.y, target.orientation.z);
      if (!observed.coeffs().allFinite() || observed.norm() < 1e-6)
        return false;
      goal_rotation = rotation * observed.normalized().toRotationMatrix() *
                      desired_rotation_.transpose();
    }
    destination = current + rotation * translation(target) - goal_rotation * desired_position_;
  } else {
    destination = translation(target) + target_offset_;
  }
  distance = (destination - current).norm();
  if (!destination.allFinite() || !goal_rotation.allFinite())
    return false;
  goal.position.x = destination.x();
  goal.position.y = destination.y();
  goal.position.z = destination.z();
  Eigen::Quaterniond q(goal_rotation);
  goal.orientation.x = q.x(); goal.orientation.y = q.y();
  goal.orientation.z = q.z(); goal.orientation.w = q.w();
  return true;
}

// This callback may block on MoveIt, but never holds the control mutex while
// calling the service. The other spinner threads continue control and stopping.
void VisualServo::hybridPlanner(const ros::TimerEvent &) {
  moveit_msgs::GetMotionPlan call;
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    if (!hybrid_pending_)
      return;
    hybrid_pending_ = false;
    call = hybrid_request_;
    generation = hybrid_generation_;
  }
  const bool ok = hybrid_plan_client_.waitForExistence(ros::Duration(1.0)) &&
                  hybrid_plan_client_.call(call);
  std::lock_guard<std::mutex> lock(control_mutex_);
  if (generation != hybrid_generation_ || !enabled_ || hybrid_fault_)
    return;
  const auto &trajectory = call.response.motion_plan_response.trajectory.joint_trajectory;
  bool valid = ok && call.response.motion_plan_response.error_code.val ==
                        moveit_msgs::MoveItErrorCodes::SUCCESS &&
               trajectory.joint_names.size() == kDof && trajectory.points.size() >= 2 &&
               call.response.motion_plan_response.trajectory.multi_dof_joint_trajectory.points.empty();
  std::array<std::size_t, kDof> order{};
  std::set<std::string> names(trajectory.joint_names.begin(), trajectory.joint_names.end());
  valid = valid && names.size() == kDof;
  for (std::size_t i = 0; valid && i < kDof; ++i) {
    auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), joint_names_[i]);
    valid = it != trajectory.joint_names.end();
    order[i] = std::distance(trajectory.joint_names.begin(), it);
  }
  std::vector<JointPoint> points;
  std::vector<double> times;
  for (const auto &point : trajectory.points) {
    if (!valid) break;
    const double t = point.time_from_start.toSec();
    valid = point.positions.size() == kDof && std::isfinite(t) &&
            (times.empty() ? std::abs(t) < 1e-6 : t > times.back());
    if (!valid) break;
    JointPoint q{};
    for (std::size_t i = 0; i < kDof; ++i) {
      q[i] = point.positions[order[i]];
      valid = valid && std::isfinite(q[i]) &&
              q[i] >= lower_limits_[i] + joint_limit_margin_ &&
              q[i] <= upper_limits_[i] - joint_limit_margin_;
      if (!times.empty())
        valid = valid && std::abs(q[i] - points.back()[i]) /
                            (t - times.back()) <= velocity_limits_[i];
    }
    points.push_back(q);
    times.push_back(t);
  }
  JointPoint feedback;
  {
    std::lock_guard<std::mutex> joint_lock(joint_mutex_);
    feedback = feedback_position_;
    valid = valid && have_joint_state_ &&
            (ros::Time::now() - last_joint_time_).toSec() <= 0.5;
  }
  valid = valid && !points.empty() && times.back() < hybrid_execution_timeout_ &&
          jointDistance(points.front(), feedback) <= 0.01;
  if (valid) {
    KDL::JntArray end(kDof);
    for (std::size_t i = 0; i < kDof; ++i) end(i) = points.back()[i];
    KDL::Frame frame;
    valid = fk_solver_->JntToCart(end, frame) >= 0;
    const auto &expected = call.request.motion_plan_request.goal_constraints.front();
    const auto &pose = expected.position_constraints.front().constraint_region.primitive_poses.front();
    valid = valid && (Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z()) -
                      translation(pose)).norm() <= 0.01;
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    valid = valid && Eigen::Quaterniond(w, x, y, z).angularDistance(
        Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                           pose.orientation.y, pose.orientation.z)) <= 0.05;
  }
  if (!valid) {
    hybrid_fault_ = true;
    ROS_ERROR("[hybrid] Planning failed or returned an invalid/stale trajectory; reset required");
    return;
  }
  hybrid_points_ = std::move(points);
  hybrid_times_ = std::move(times);
  hybrid_elapsed_ = 0.0;
  hybrid_execution_started_ = ros::Time::now();
  hybrid_plan_started_ = ros::Time();
  holdFeedback(feedback);
  transitionTo(ServoState::APPROACH);
}

bool VisualServo::hybridControl(const JointPoint &feedback,
                              const geometry_msgs::Pose &target,
                              bool fresh_target, double dt) {
  const ros::Time now = ros::Time::now();
  if (!enabled_ || safety_stop_.load() || hybrid_fault_ || !fresh_target) {
    const bool fault = hybrid_fault_;
    resetHybrid();
    hybrid_fault_ = fault;
    transitionTo(!enabled_ ? ServoState::DISABLED :
                 (fault || safety_stop_.load() ? ServoState::HOLD : ServoState::WAITING));
    holdFeedback(feedback);
    return true;
  }
  geometry_msgs::Pose goal;
  double distance;
  if (!hybridGoal(feedback, target, goal, distance)) {
    hybrid_fault_ = true;
    transitionTo(ServoState::HOLD);
    holdFeedback(feedback);
    return true;
  }
  if ((!hybrid_plan_started_.isZero() || !hybrid_points_.empty()) &&
      ((translation(goal) - translation(hybrid_goal_)).norm() > hybrid_target_drift_ ||
       (use_orientation_control_ &&
        Eigen::Quaterniond(goal.orientation.w, goal.orientation.x, goal.orientation.y,
                           goal.orientation.z).angularDistance(
          Eigen::Quaterniond(hybrid_goal_.orientation.w, hybrid_goal_.orientation.x,
                             hybrid_goal_.orientation.y, hybrid_goal_.orientation.z)) > 0.15))) {
    resetHybrid(); // Stop first, then plan again from measured joints.
    transitionTo(ServoState::WAITING);
    holdFeedback(feedback);
    return true;
  }
  if (!hybrid_points_.empty()) {
    if ((now - hybrid_execution_started_).toSec() > hybrid_execution_timeout_) {
      hybrid_fault_ = true;
      transitionTo(ServoState::HOLD);
      holdFeedback(feedback);
      return true;
    }
    const double t = std::min(hybrid_elapsed_, hybrid_times_.back());
    auto upper = std::upper_bound(hybrid_times_.begin(), hybrid_times_.end(), t);
    const std::size_t b = std::min<std::size_t>(std::distance(hybrid_times_.begin(), upper), hybrid_times_.size() - 1);
    const std::size_t a = b ? b - 1 : 0;
    const double fraction = a == b ? 1.0 : (t - hybrid_times_[a]) / (hybrid_times_[b] - hybrid_times_[a]);
    JointPoint command;
    for (std::size_t i = 0; i < kDof; ++i)
      command[i] = hybrid_points_[a][i] + fraction * (hybrid_points_[b][i] - hybrid_points_[a][i]);
    if (jointDistance(command, feedback) > hybrid_joint_error_) {
      hybrid_fault_ = true;
      ROS_ERROR("[hybrid] Planned path tracking error exceeded limit");
      transitionTo(ServoState::HOLD);
      holdFeedback(feedback);
      return true;
    }
    // Only one control period of lookahead; do not preload an entire stale path.
    const int steps = std::max(1, static_cast<int>(std::round(dt * output_rate_)));
    for (int step = 1; step <= steps; ++step) {
      JointPoint sample;
      for (std::size_t i = 0; i < kDof; ++i)
        sample[i] = command_position_[i] + (command[i] - command_position_[i]) * step / steps;
      queue_.push(sample);
    }
    command_position_ = command;
    hybrid_elapsed_ += dt;
    if (t >= hybrid_times_.back() && jointDistance(feedback, hybrid_points_.back()) < 0.01) {
      if (hybrid_settle_since_.isZero()) hybrid_settle_since_ = now;
      if ((now - hybrid_settle_since_).toSec() >= 0.3) {
        resetHybrid();
        hybrid_near_ = distance <= hybrid_enter_;
        holdFeedback(feedback);
        transitionTo(ServoState::WAITING);
      }
    } else {
      hybrid_settle_since_ = ros::Time();
    }
    return true;
  }
  if (!hybrid_plan_started_.isZero()) {
    if ((now - hybrid_plan_started_).toSec() > hybrid_planning_time_ + 2.0) {
      ++hybrid_generation_;
      hybrid_pending_ = false;
      hybrid_fault_ = true;
      transitionTo(ServoState::HOLD);
    }
    holdFeedback(feedback);
    return true;
  }
  if (hybrid_near_ && distance > hybrid_exit_) {
    resetHybrid();
    holdFeedback(feedback);
    transitionTo(ServoState::WAITING);
    return true;
  }
  if (hybrid_near_ || distance <= hybrid_enter_) {
    hybrid_near_ = true;
    return false; // Existing PBVS -> damped Jacobian inverse -> integration -> queue.
  }
  // Require a stationary start before requesting a collision-aware path.
  if (jointDistance(hybrid_settle_position_, feedback) > 0.002)
    hybrid_settle_since_ = ros::Time();
  if (hybrid_settle_since_.isZero()) {
    hybrid_settle_since_ = now;
    hybrid_settle_position_ = feedback;
  }
  holdFeedback(feedback);
  transitionTo(ServoState::WAITING);
  if ((now - hybrid_settle_since_).toSec() < 0.3) return true;

  hybrid_goal_ = goal;
  KDL::JntArray joints(kDof);
  for (std::size_t i = 0; i < kDof; ++i) joints(i) = feedback[i];
  KDL::Frame frame;
  if (fk_solver_->JntToCart(joints, frame) < 0) {
    hybrid_fault_ = true;
    return true;
  }
  const Eigen::Vector3d current(frame.p.x(), frame.p.y(), frame.p.z());
  const Eigen::Vector3d approach = translation(goal) -
      hybrid_standoff_ * (translation(goal) - current).normalized();
  goal.position.x = approach.x(); goal.position.y = approach.y(); goal.position.z = approach.z();
  hybrid_request_ = moveit_msgs::GetMotionPlan();
  auto &request = hybrid_request_.request.motion_plan_request;
  request.group_name = planning_group_;
  request.num_planning_attempts = 3;
  request.allowed_planning_time = hybrid_planning_time_;
  request.max_velocity_scaling_factor = 0.1;
  request.max_acceleration_scaling_factor = 0.1;
  request.start_state.is_diff = true;
  request.start_state.joint_state.name = joint_names_;
  request.start_state.joint_state.position.assign(feedback.begin(), feedback.end());
  request.start_state.joint_state.header.stamp = now;
  moveit_msgs::Constraints constraints;
  moveit_msgs::PositionConstraint position;
  position.header.frame_id = base_link_;
  position.link_name = control_link_;
  position.weight = 1.0;
  shape_msgs::SolidPrimitive sphere;
  sphere.type = shape_msgs::SolidPrimitive::SPHERE;
  sphere.dimensions.push_back(0.003);
  position.constraint_region.primitives.push_back(sphere);
  position.constraint_region.primitive_poses.push_back(goal);
  constraints.position_constraints.push_back(position);
  moveit_msgs::OrientationConstraint orientation;
  orientation.header.frame_id = base_link_;
  orientation.link_name = control_link_;
  orientation.orientation = goal.orientation;
  orientation.absolute_x_axis_tolerance = 0.02;
  orientation.absolute_y_axis_tolerance = 0.02;
  orientation.absolute_z_axis_tolerance = 0.02;
  orientation.weight = 1.0;
  constraints.orientation_constraints.push_back(orientation);
  request.goal_constraints.push_back(constraints);
  hybrid_pending_ = true;
  hybrid_plan_started_ = now;
  hybrid_settle_since_ = ros::Time();
  transitionTo(ServoState::PLANNING);
  return true;
}
} // namespace aubo_ros_control
