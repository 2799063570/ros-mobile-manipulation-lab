#include <aubo_sorting_core/color_sorting_task.hpp>
#include "task_utils.hpp"
#include <tf/transform_datatypes.h>
#include <chrono>
#include <cmath>
#include <sstream>

namespace aubo_sorting_core
{
void ColorSortingTask::resetInstanceQueue()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  instance_queue_.clear();
  pending_detection_.reset();
  queue_epoch_ = ros::Time::now();
  queue_last_stamp_ = ros::Time();
  queue_last_frame_ = queue_empty_since_ = ros::WallTime();
  queue_empty_frames_ = 0;
}

void ColorSortingTask::targetWorker()
{
  while (!target_worker_shutdown_.load() && ros::ok())
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_condition_.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return target_worker_shutdown_.load() || static_cast<bool>(pending_detection_);
      });
      if (target_worker_shutdown_.load()) return;
      if (!pending_detection_) continue;
      const auto message = pending_detection_;
      pending_detection_.reset();
      // Serializes workspace changes too. TF lookups are non-waiting, and no robot
      // planning, motion, or disk I/O is permitted under this mutex.
      try { processInstanceFrame(*message); }
      catch (const std::exception& error) {
        queue_last_frame_ = ros::WallTime();
        queue_empty_since_ = ros::WallTime();
        queue_empty_frames_ = 0;
        ROS_WARN_THROTTLE(2.0, "Instance tracker rejected frame: %s", error.what());
      }
    }
    publishInstanceQueue();
  }
}

void ColorSortingTask::processInstanceFrame(const aubo_perception::DetectedObjectArray& message)
{
  State state;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state = state_;
  }
  const bool observing = observation_ready_.load() &&
      (state == State::OBSERVING || state == State::DETECTING || state == State::READY);
  const bool active = state == State::PICKING || state == State::DETECTING || state == State::SORTING;
  if (stop_requested_.load() || (!observing && !active)) return;
  const auto stamp = message.header.stamp;
  const double age = (ros::Time::now() - stamp).toSec();
  if (stamp.isZero() || stamp <= queue_epoch_ || stamp <= queue_last_stamp_ ||
      !std::isfinite(age) || age < 0 || age > queue_frame_max_age_) return;
  queue_last_stamp_ = stamp;
  if (!message.observation_valid || message.header.frame_id.empty()) {
    queue_last_frame_ = ros::WallTime();
    queue_empty_frames_ = 0;
    queue_empty_since_ = ros::WallTime();
    return;
  }
  tf::StampedTransform source_to_target, target_to_table, place_to_target, gripper, camera;
  const auto lookup = [this, &stamp](const std::string& to, const std::string& from,
                                   tf::StampedTransform& transform) {
    transform.setIdentity();
    if (to != from) tf_listener_.lookupTransform(to, from, stamp, transform);
  };
  lookup(target_frame_, message.header.frame_id, source_to_target);
  lookup(table_frame_, target_frame_, target_to_table);
  lookup(target_frame_, place_frame_, place_to_target);
  // Even a zero-object frame needs a valid camera transform in the perception node.
  // During motion, discard the gripper neighbourhood rather than tracking carried objects.
  tf::Vector3 gripper_projection;
  bool has_projection = false;
  if (state == State::PICKING) {
    lookup(target_frame_, end_effector_link_, gripper);
    if (message.sensor_frame.empty()) {
      queue_last_frame_ = ros::WallTime();
      return;
    }
    lookup(target_frame_, message.sensor_frame, camera);
    const auto ray = gripper.getOrigin() - camera.getOrigin();
    if (std::abs(ray.z()) > 1e-6) {
      const double scale = (table_z_ + object_height_ - camera.getOrigin().z()) / ray.z();
      has_projection = scale > 0;
      gripper_projection = camera.getOrigin() + scale * ray;
    }
  }
  std::vector<ObjectQueue::Sample> samples;
  bool valid = true;
  for (const auto& object : message.objects)
  {
    if (std::find(sort_colors_.begin(), sort_colors_.end(), object.color) == sort_colors_.end()) continue;
    const auto& p = object.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) { valid = false; continue; }
    const tf::Vector3 point = source_to_target * tf::Vector3(p.x, p.y, p.z);
    const tf::Vector3 table_point = target_to_table * point;
    if (std::abs(table_point.x() - table_center_[0]) > table_size_[0] / 2 ||
        std::abs(table_point.y() - table_center_[1]) > table_size_[1] / 2) continue;
    bool excluded = false;
    for (const auto& place : place_targets_) {
      const auto destination = place_to_target * tf::Vector3(place.second[0], place.second[1], 0);
      if (std::hypot(point.x()-destination.x(), point.y()-destination.y()) < queue_place_exclusion_radius_)
        excluded = true;
    }
    if (state == State::PICKING && (point - gripper.getOrigin()).length() < queue_gripper_exclusion_radius_)
      excluded = true;
    if (has_projection && std::hypot(point.x()-gripper_projection.x(), point.y()-gripper_projection.y()) <
        queue_gripper_exclusion_radius_) excluded = true;
    if (excluded) continue;
    ObjectQueue::Sample sample;
    sample.color = object.color;
    sample.x = point.x(); sample.y = point.y(); sample.z = point.z();
    sample.payload = object;
    sample.payload.pose.position.x = point.x();
    sample.payload.pose.position.y = point.y();
    sample.payload.pose.position.z = point.z();
    const double height = object.object_height > 0 ? object.object_height : object_height_;
    sample.eligible = std::isfinite(height) && height > 0 &&
        std::abs(point.z() - (table_z_ + height / 2)) <= queue_height_tolerance_ &&
        (height_mode_ != "depth" || (object.depth_valid &&
         std::abs(point.z() - (table_z_ + height / 2)) <= height_tolerance_)) &&
        (!(use_detected_angle_ || use_detected_width_) || object.grasp_geometry_valid);
    if (use_detected_angle_) {
      const auto axis = source_to_target.getBasis() * tf::Vector3(std::cos(object.grasp_angle), std::sin(object.grasp_angle), 0);
      sample.payload.grasp_angle = std::atan2(axis.y(), axis.x());
      sample.eligible = sample.eligible && std::isfinite(object.grasp_angle) && std::abs(axis.z()) < 0.1;
    }
    if (use_detected_width_)
      sample.eligible = sample.eligible && std::isfinite(object.grasp_width) && object.grasp_width > 0 &&
          object.grasp_width <= gripper_width_open_ &&
          object.grasp_width * width_close_scale_ >= gripper_width_closed_;
    samples.push_back(sample); // Invalid grasp geometry still blocks false "table empty" completion.
  }
  const auto now = ros::WallTime::now();
  if (!valid) {
    queue_last_frame_ = queue_empty_since_ = ros::WallTime();
    queue_empty_frames_ = 0;
    instance_queue_.invalidate();
    return;
  }
  const bool observation_gap = queue_last_frame_.isZero() ||
      (now-queue_last_frame_).toSec() > queue_frame_max_age_;
  instance_queue_.update(samples, now.toSec());
  queue_last_frame_ = now;
  if (observing && valid && samples.empty()) {
    instance_queue_.invalidate();
    if (observation_gap) { queue_empty_since_ = ros::WallTime(); queue_empty_frames_ = 0; }
    if (queue_empty_since_.isZero()) queue_empty_since_ = now;
    ++queue_empty_frames_;
  } else {
    queue_empty_since_ = ros::WallTime();
    queue_empty_frames_ = 0;
  }
}

bool ColorSortingTask::validateReservedTarget()
{
  if (!continuous_sorting_ || active_instance_id_ == 0) return true;
  bool valid;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const auto now = ros::WallTime::now();
    valid = instance_queue_.executionValid(active_instance_id_, now.toSec()) &&
        !queue_last_frame_.isZero() && (now-queue_last_frame_).toSec() <= queue_frame_max_age_;
  }
  if (!valid) setFailure("DETECTION_FAILED", "reserved target moved, expired, or observation unavailable before descent");
  return valid && !stop_requested_.load();
}

void ColorSortingTask::publishInstanceQueue()
{
  std::ostringstream stream;
  stream.precision(15);
  stream << "{\"schema_version\":2,\"frame_id\":\"" << detail::jsonEscape(target_frame_) << "\",\"targets\":{";
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    bool first = true;
    const double now = ros::WallTime::now().toSec();
    for (const auto& item : instance_queue_.tracks()) {
      const auto& track = item.second;
      if (!first) stream << ',';
      first = false;
      stream << '"' << item.first << "\":{\"id\":" << item.first
             << ",\"color\":\"" << detail::jsonEscape(track.sample.color)
             << "\",\"status\":\"" << ObjectQueue::statusName(track.status)
             << "\",\"position\":[" << track.sample.x << ',' << track.sample.y << ',' << track.sample.z
             << "],\"observations\":" << track.observations
             << ",\"confidence\":" << std::min(1.0, static_cast<double>(track.observations)/instance_queue_.min_observations)
             << ",\"age\":" << std::max(0.0, now-track.last_seen)
             << ",\"picked\":" << (track.status == ObjectQueue::Status::DONE ? "true" : "false") << '}';
    }
  }
  stream << "}}";
  std_msgs::String message;
  message.data = stream.str();
  target_cache_publisher_.publish(message);
}

bool ColorSortingTask::continuousSortingOperation()
{
  // A panel can wait with the base unlocked after observing; begin a fresh epoch.
  resetInstanceQueue();
  publishState(State::DETECTING, "building instance queue");
  auto deadline = ros::WallTime::now() + ros::WallDuration(detection_timeout_);
  ros::WallRate rate(20);
  while (ros::ok() && !stop_requested_.load()) {
    ObjectQueue::Track target;
    bool reserved = false, empty = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      const auto now = ros::WallTime::now();
      // No new command after camera loss, even if a previously stable track survives.
      if (!queue_last_frame_.isZero() && (now-queue_last_frame_).toSec() <= queue_frame_max_age_)
        reserved = instance_queue_.reserve(sort_colors_, now.toSec(), target);
      empty = observation_ready_.load() && !queue_empty_since_.isZero() &&
          (now-queue_empty_since_).toSec() >= queue_empty_confirmation_ &&
          (now-queue_last_frame_).toSec() <= queue_frame_max_age_ && queue_empty_frames_ >= queue_empty_min_frames_;
    }
    if (reserved) {
      observation_ready_.store(false);
      publishState(State::PICKING, target.sample.color + " #" + std::to_string(target.id));
      publishInstanceQueue();
      auto detected = target.sample.payload;
      detected.pose.position.x = target.sample.x;
      detected.pose.position.y = target.sample.y;
      detected.pose.position.z = target.sample.z;
      // Keep the scene refresh independent from a return to the observation pose.
      bool success = false;
      active_instance_id_ = target.id;
      const auto clear_active = [this](void*) { active_instance_id_ = 0; };
      std::unique_ptr<void, decltype(clear_active)> active_guard(this, clear_active);
      try { success = refreshOctomap() && addTableCollision() && pickAndPlace(detected); }
      catch (...) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        instance_queue_.finish(target.id, false, ros::WallTime::now().toSec());
        throw;
      }
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        instance_queue_.finish(target.id, success, ros::WallTime::now().toSec());
        // Reject frames captured before release/retreat; they may show the carried object.
        queue_epoch_ = ros::Time::now();
        queue_last_frame_ = ros::WallTime();
        queue_empty_since_ = ros::WallTime();
        queue_empty_frames_ = 0;
      }
      publishInstanceQueue();
      if (!success) return false; // Existing stop/recovery flow owns uncertain gripper state.
      publishState(State::DETECTING, "next cached instance");
      deadline = ros::WallTime::now() + ros::WallDuration(target_cache_fallback_delay_);
      continue;
    }
    if (empty) {
      observation_ready_.store(false);
      if (!finish_named_target_.empty()) {
        publishState(State::HOMING, finish_named_target_);
        return moveNamed(finish_named_target_);
      }
      return true;
    }
    if (ros::WallTime::now() >= deadline) {
      if (observation_ready_.load()) {
        setFailure("DETECTION_FAILED", "no stable instance or valid sustained empty observation");
        return false;
      }
      publishState(State::OBSERVING, "queue needs a fresh observation");
      if (!observation()) return false;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        instance_queue_.invalidate();
        queue_epoch_ = ros::Time::now();
        queue_last_frame_ = queue_empty_since_ = ros::WallTime();
        queue_empty_frames_ = 0;
      }
      publishState(State::DETECTING, "confirming remaining instances");
      deadline = ros::WallTime::now() + ros::WallDuration(detection_timeout_);
    }
    rate.sleep();
  }
  return false;
}
} // namespace aubo_sorting_core
