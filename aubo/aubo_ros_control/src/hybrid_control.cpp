#include <aubo_ros_control/visual_servo.h>
#include <Eigen/Geometry>
#include <kdl/jntarray.hpp>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <shape_msgs/SolidPrimitive.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <string>

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
  private_nh_.param("hybrid_execution_timeout", hybrid_execution_timeout_, 60.0);
  private_nh_.param("hybrid_require_scene_ready",
                    hybrid_require_scene_ready_, false);
  private_nh_.param("hybrid_min_tcp_z", hybrid_min_tcp_z_, -1e9);
  private_nh_.param("hybrid_use_orientation_control",
                    hybrid_orientation_control_, false);
  std::vector<double> hybrid_rpy;
  if (!private_nh_.getParam("hybrid_desired_tcp_rpy", hybrid_rpy))
    hybrid_rpy = {3.141592653589793, 0.0, 0.0};
  if (hybrid_rpy.size() != 3 ||
      !std::all_of(hybrid_rpy.begin(), hybrid_rpy.end(),
                   [](double value) { return std::isfinite(value); })) {
    ROS_ERROR("[hybrid] hybrid_desired_tcp_rpy must contain three finite values");
    return false;
  }
  hybrid_desired_rotation_ =
      (Eigen::AngleAxisd(hybrid_rpy[2], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(hybrid_rpy[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(hybrid_rpy[0], Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  for (double value : {hybrid_enter_, hybrid_exit_, hybrid_standoff_,
                       hybrid_target_drift_, hybrid_joint_error_,
                       hybrid_planning_time_, hybrid_execution_timeout_}) {
    if (!std::isfinite(value) || value <= 0.0) {
      ROS_ERROR("[hybrid] All hybrid limits must be finite and positive");
      return false;
    }
  }
  if (!std::isfinite(hybrid_min_tcp_z_)) {
    ROS_ERROR("[hybrid] hybrid_min_tcp_z must be finite");
    return false;
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
  Eigen::Matrix3d goal_rotation =
      hybrid_orientation_control_ ? hybrid_desired_rotation_ :
      (use_orientation_control_ ? desired_rotation_ : rotation);
  Eigen::Vector3d destination;
  if (servo_mode_ == "eye_in_hand") {
    if (use_orientation_control_ && !hybrid_orientation_control_) {
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
  if (destination.z() < hybrid_min_tcp_z_) {
    ROS_ERROR_THROTTLE(1.0,
        "[hybrid] Requested TCP height %.3f m is below safety floor %.3f m; "
        "holding to protect the table", destination.z(), hybrid_min_tcp_z_);
    return false;
  }
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
  std::string invalid_reason;
  auto invalidate = [&](const std::string &reason) {
    if (invalid_reason.empty()) invalid_reason = reason;
    valid = false;
  };
  if (!ok) invalid_reason = "planning service call failed";
  else if (call.response.motion_plan_response.error_code.val !=
           moveit_msgs::MoveItErrorCodes::SUCCESS)
    invalid_reason = "MoveIt returned error code " +
        std::to_string(call.response.motion_plan_response.error_code.val);
  else if (trajectory.joint_names.size() != kDof)
    invalid_reason = "trajectory does not contain exactly six joints";
  else if (trajectory.points.size() < 2)
    invalid_reason = "trajectory contains fewer than two points";
  else if (!call.response.motion_plan_response.trajectory.
                multi_dof_joint_trajectory.points.empty())
    invalid_reason = "multi-DOF trajectory is unsupported";
  std::array<std::size_t, kDof> order{};
  std::set<std::string> names(trajectory.joint_names.begin(), trajectory.joint_names.end());
  if (valid && names.size() != kDof)
    invalidate("trajectory contains duplicate joint names");
  for (std::size_t i = 0; valid && i < kDof; ++i) {
    auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), joint_names_[i]);
    if (it == trajectory.joint_names.end()) {
      invalidate("trajectory is missing joint " + joint_names_[i]);
      break;
    }
    order[i] = std::distance(trajectory.joint_names.begin(), it);
  }
  std::vector<JointPoint> points;
  std::vector<double> times;
  double time_scale = 1.0;
  const bool sparse_trajectory = trajectory.points.size() == 2;
  for (const auto &point : trajectory.points) {
    if (!valid) break;
    const double t = point.time_from_start.toSec();
    if (point.positions.size() != kDof) {
      invalidate("trajectory point does not contain six positions");
      break;
    }
    if (!std::isfinite(t) ||
        (times.empty() ? std::abs(t) >= 1e-6 : t <= times.back())) {
      invalidate("trajectory timestamps are invalid or not strictly increasing");
      break;
    }
    JointPoint q{};
    for (std::size_t i = 0; i < kDof; ++i) {
      q[i] = point.positions[order[i]];
      if (!std::isfinite(q[i])) {
        invalidate("trajectory contains a non-finite joint position");
        break;
      }
      if (q[i] < lower_limits_[i] + joint_limit_margin_ ||
          q[i] > upper_limits_[i] - joint_limit_margin_) {
        invalidate("trajectory violates the configured limit of joint " +
                   joint_names_[i]);
        break;
      }
      if (!times.empty()) {
        const double duration = t - times.back();
        const double displacement = std::abs(q[i] - points.back()[i]);
        // A two-point path uses smoothstep, whose peak normalized speed is
        // 1.5 and acceleration is 6. Dense MoveIt trajectories retain their
        // time-parameterized linear segments; stopping at every generated
        // sample would inflate a normal path to tens of seconds.
        const double velocity_duration =
            (sparse_trajectory ? 3.0 : 2.0) * displacement /
            velocity_limits_[i];
        const double acceleration_duration = sparse_trajectory
            ? std::sqrt(12.0 * displacement / acceleration_limits_[i])
            : 0.0;
        time_scale = std::max(
            time_scale, std::max(velocity_duration, acceleration_duration) /
                            duration);
      }
    }
    if (!valid) break;
    points.push_back(q);
    times.push_back(t);
  }
  // MoveIt may be configured with permissive model dynamics (this project
  // historically uses 100 rad/s). Preserve its collision-checked path but
  // retime it to this controller's stricter velocity and acceleration limits.
  if (valid && time_scale > 1.0) {
    for (double &time : times) time *= time_scale;
    ROS_WARN("[hybrid] Retimed MoveIt trajectory by %.2fx to %.2f s for safe "
             "velocity and acceleration tracking", time_scale, times.back());
  }
  JointPoint feedback;
  {
    std::lock_guard<std::mutex> joint_lock(joint_mutex_);
    feedback = feedback_position_;
    if (valid && (!have_joint_state_ ||
        (ros::Time::now() - last_joint_time_).toSec() > 0.5))
      invalidate("joint feedback is missing or stale after planning");
  }
  if (valid && points.empty()) invalidate("trajectory contains no usable points");
  if (valid && times.back() >= hybrid_execution_timeout_)
    invalidate("safely retimed trajectory duration " +
               std::to_string(times.back()) + " s exceeds execution timeout " +
               std::to_string(hybrid_execution_timeout_) + " s");
  if (valid && jointDistance(points.front(), feedback) > 0.01)
    invalidate("trajectory start differs from current feedback by more than 0.01 rad");
  if (valid) {
    KDL::JntArray end(kDof);
    for (std::size_t i = 0; i < kDof; ++i) end(i) = points.back()[i];
    KDL::Frame frame;
    if (fk_solver_->JntToCart(end, frame) < 0)
      invalidate("FK failed for trajectory endpoint");
    const auto &expected = call.request.motion_plan_request.goal_constraints.front();
    const auto &pose = expected.position_constraints.front().constraint_region.primitive_poses.front();
    if (valid && (Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z()) -
                  translation(pose)).norm() > 0.01)
      invalidate("trajectory endpoint position misses the requested approach pose");
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    if (valid && Eigen::Quaterniond(w, x, y, z).angularDistance(
          Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z)) > 0.05)
      invalidate("trajectory endpoint orientation misses the requested approach pose");
  }
  if (!valid) {
    hybrid_fault_ = true;
    ROS_ERROR("[hybrid] Rejected MoveIt trajectory: %s; reset required",
              invalid_reason.empty() ? "unknown validation failure" :
                                       invalid_reason.c_str());
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
  if (!enabled_ || safety_stop_.load() || hybrid_fault_) {
    const bool fault = hybrid_fault_;
    resetHybrid();
    hybrid_fault_ = fault;
    transitionTo(!enabled_ ? ServoState::DISABLED :
                 (fault || safety_stop_.load() ? ServoState::HOLD : ServoState::WAITING));
    holdFeedback(feedback);
    return true;
  }
  if (hybrid_require_scene_ready_ && !hybrid_scene_ready_.load()) {
    if (hybrid_pending_ || !hybrid_plan_started_.isZero() ||
        !hybrid_points_.empty())
      resetHybrid();
    ROS_WARN_THROTTLE(1.0,
        "[hybrid] MoveIt table collision scene is not ready; motion inhibited");
    transitionTo(ServoState::WAITING);
    holdFeedback(feedback);
    return true;
  }
  // Eye-in-hand has a mandatory preparation step: reach the configured wrist
  // camera viewpoint before accepting even an already visible target.
  if (!hybrid_observation_complete_) {
    if (servo_mode_ != "eye_in_hand" || !initial_search_enabled_) {
      hybrid_observation_complete_ = true;
    } else if (jointDistance(feedback, initial_search_posture_) > 0.02) {
      return false;  // controlLoop selects SEARCH_INITIAL and drives the posture.
    } else {
      hybrid_observation_complete_ = true;
      holdFeedback(feedback);
      transitionTo(ServoState::WAITING);
      return true;
    }
  }
  if (!fresh_target) {
    // Once the mandatory observation posture has been reached, target loss
    // always stops and holds instead of moving the arm blindly.
    resetHybrid();
    transitionTo(ServoState::WAITING);
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
       ((use_orientation_control_ || hybrid_orientation_control_) &&
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
    const double linear_fraction = a == b ? 1.0 :
        (t - hybrid_times_[a]) / (hybrid_times_[b] - hybrid_times_[a]);
    // Smooth only a truly sparse start/goal trajectory. MoveIt-generated
    // intermediate points already describe a time-parameterized path and
    // must not be treated as mandatory full stops.
    const double fraction = hybrid_points_.size() == 2
        ? linear_fraction * linear_fraction * (3.0 - 2.0 * linear_fraction)
        : linear_fraction;
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
  // MoveIt limits now match the output controller. A 0.5 scale leaves the
  // same 50% tracking reserve enforced again by trajectory validation.
  request.max_velocity_scaling_factor = 0.5;
  request.max_acceleration_scaling_factor = 0.5;
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
