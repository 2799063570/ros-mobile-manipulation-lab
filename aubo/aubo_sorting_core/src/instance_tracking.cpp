#include <aubo_sorting_core/sorting_task.hpp>
#include <aubo_sorting_core/observation_tf.hpp>
#include <aubo_sorting_core/occlusion_geometry.hpp>
#include "task_utils.hpp"
#include <tf/transform_datatypes.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

namespace aubo_sorting_core
{
void SortingTask::resetInstanceQueue()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  instance_queue_.clear();
  pending_detection_.reset();
  queue_epoch_ = ros::Time::now();
  queue_last_stamp_ = ros::Time();
  queue_last_frame_ = queue_last_source_frame_ = queue_empty_since_ = ros::WallTime();
  queue_empty_frames_ = 0;
  queue_target_occluded_ = false;
  queue_occlusion_checked_at_ = ros::WallTime();
}

void SortingTask::setQueueVisionPhase(QueueVisionPhase phase)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (queue_vision_phase_.load()==phase) return;
  // Frames captured before a phase boundary must not arrive late and mutate
  // the queue in the following phase (inference can take hundreds of ms).
  queue_epoch_=ros::Time::now();
  pending_detection_.reset();
  queue_target_occluded_=false;
  queue_occlusion_checked_at_=ros::WallTime();
  if (phase==QueueVisionPhase::MASKED) queue_last_frame_=ros::WallTime();
  queue_vision_phase_.store(phase);
}

bool SortingTask::reservedTargetOccluded(const aubo_perception::DetectedObjectArray& message)
{
  const auto found=std::find_if(instance_queue_.tracks().begin(),instance_queue_.tracks().end(),
      [](const std::pair<const std::uint64_t,ObjectQueue::Track>& item) {
        return item.second.status==ObjectQueue::Status::RESERVED;
      });
  if (found==instance_queue_.tracks().end() || message.sensor_frame.empty()) return false;
  tf::StampedTransform transform;
  lookupObservationTransform(tf_listener_,target_frame_,message.sensor_frame,message.header.stamp,false,transform);
  const Point3 camera{{transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()}};
  std::vector<Point3> links;
  for (const auto& link:queue_occlusion_links_) {
    lookupObservationTransform(tf_listener_,target_frame_,link,message.header.stamp,false,transform);
    links.push_back({{transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()}});
  }
  const auto& sample=found->second.sample;
  return robotOccludesTarget(camera,Point3{{sample.x,sample.y,sample.z}},links,queue_occlusion_radius_);
}

bool SortingTask::waitForInstanceTransforms(const aubo_perception::DetectedObjectArray& message,std::string& error)
{
  error.clear();
  std::vector<ObservationTransformRequest> requests;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const auto phase=queue_vision_phase_.load();
    const double age=(ros::Time::now()-message.header.stamp).toSec();
    // These frames need no TF. The normal frame gate handles rejection and
    // masked heartbeats without accidentally querying Time(0).
    if (message.header.stamp.isZero() || message.header.stamp<=queue_epoch_ ||
        message.header.stamp<=queue_last_stamp_ || age<0 || age>queue_frame_max_age_ ||
        phase==QueueVisionPhase::DISABLED || phase==QueueVisionPhase::MASKED) return true;
    State state;
    { std::lock_guard<std::mutex> data_lock(data_mutex_); state=state_; }
    if (state!=State::OBSERVING && state!=State::PICKING && state!=State::DETECTING &&
        state!=State::READY && state!=State::SORTING) return true;
    const auto add=[&requests](const std::string& to,const std::string& from,bool world) {
      if (to!=from && !from.empty()) requests.emplace_back(to,from,world);
    };
    if (phase==QueueVisionPhase::APPROACH && !message.header.frame_id.empty() && !message.sensor_frame.empty()) {
      add(target_frame_,message.sensor_frame,false);
      for (const auto& link:queue_occlusion_links_) add(target_frame_,link,false);
    }
    if (message.observation_valid && !message.header.frame_id.empty()) {
      add(target_frame_,message.header.frame_id,false);
      add(table_frame_,target_frame_,true);
      add(target_frame_,place_frame_,true);
      if (state==State::PICKING && grasp_secured_.load()) {
        add(target_frame_,end_effector_link_,false);
        add(target_frame_,message.sensor_frame,false);
      }
    }
  }
  const auto cancelled=[this,&message] {
    if (target_worker_shutdown_.load() || stop_requested_.load() || !ros::ok()) return true;
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const double age=(ros::Time::now()-message.header.stamp).toSec();
    return message.header.stamp<=queue_epoch_ || message.header.stamp<=queue_last_stamp_ ||
        age<0 || age>queue_frame_max_age_ || queue_vision_phase_.load()==QueueVisionPhase::DISABLED;
  };
  return waitForObservationTransforms(tf_listener_,requests,message.header.stamp,queue_tf_wait_timeout_,cancelled,error);
}

void SortingTask::targetWorker()
{
  while (!target_worker_shutdown_.load() && ros::ok())
  {
    aubo_perception::DetectedObjectArrayConstPtr message;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_condition_.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return target_worker_shutdown_.load() || static_cast<bool>(pending_detection_);
      });
      if (target_worker_shutdown_.load()) return;
      if (!pending_detection_) continue;
      message = pending_detection_;
      pending_detection_.reset();
    }
    // TF may lag the camera by one joint-feedback interval. Keep one in-flight
    // frame while the callback retains only the latest pending frame. No queue
    // mutex is held while waiting, so stop/phase changes remain responsive.
    std::string tf_error;
    const bool transforms_ready=waitForInstanceTransforms(*message,tf_error);
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      if (!transforms_ready) {
        if (!tf_error.empty() && message->header.stamp>queue_epoch_ &&
            message->header.stamp>queue_last_stamp_ && !stop_requested_.load()) {
          queue_last_frame_=ros::WallTime(); queue_target_occluded_=false;
          queue_empty_since_=ros::WallTime(); queue_empty_frames_=0;
          ROS_WARN_THROTTLE(2.0,"Instance tracker TF unavailable after %.0f ms at source time: %s",
                            queue_tf_wait_timeout_*1000.,tf_error.c_str());
        }
        lock.unlock();
        publishInstanceQueue();
        continue;
      }
      // Phase/epoch/source age are checked again inside processInstanceFrame.
      // The lookups here are non-waiting; no robot motion or IO under this lock.
      try { processInstanceFrame(*message); }
      catch (const std::exception& error) {
        queue_last_frame_ = ros::WallTime();
        queue_target_occluded_=false;
        queue_empty_since_ = ros::WallTime();
        queue_empty_frames_ = 0;
        ROS_WARN_THROTTLE(2.0, "Instance tracker rejected frame: %s", error.what());
      }
    }
    publishInstanceQueue();
  }
}

void SortingTask::processInstanceFrame(const aubo_perception::DetectedObjectArray& message)
{
  const QueueVisionPhase vision_phase = queue_vision_phase_.load();
  if (stop_requested_.load() || vision_phase == QueueVisionPhase::DISABLED) return;
  State state;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state = state_;
  }
  // OBSERVING is intentionally accepted before observation_ready_: frames seen
  // while moving to the observation pose can already build stable world tracks.
  const bool collecting_state = state == State::OBSERVING || state == State::PICKING ||
      state == State::DETECTING || state == State::READY || state == State::SORTING;
  if (!collecting_state) return;
  const bool observing = observation_ready_.load() &&
      (state == State::OBSERVING || state == State::DETECTING || state == State::READY);
  const auto stamp = message.header.stamp;
  const double age = (ros::Time::now() - stamp).toSec();
  if (stamp.isZero() || stamp <= queue_epoch_ || stamp <= queue_last_stamp_ ||
      !std::isfinite(age) || age < 0 || age > queue_frame_max_age_) return;
  queue_last_stamp_ = stamp;
  const auto received=ros::WallTime::now();
  const bool source_frame=!message.header.frame_id.empty() && !message.sensor_frame.empty();
  if (vision_phase==QueueVisionPhase::MASKED) {
    // A source-stamped frame with a known optical frame is still a camera
    // heartbeat when the robot deliberately obscures depth/target geometry.
    if (source_frame) queue_last_source_frame_=received;
    queue_empty_frames_=0; queue_empty_since_=ros::WallTime();
    return;
  }
  queue_target_occluded_=false;
  if (vision_phase==QueueVisionPhase::APPROACH && source_frame) {
    queue_target_occluded_=reservedTargetOccluded(message);
    queue_occlusion_checked_at_=received;
    if (queue_target_occluded_) {
      queue_last_source_frame_=received;
      queue_empty_frames_=0; queue_empty_since_=ros::WallTime();
      return;
    }
  }
  if (!message.observation_valid || message.header.frame_id.empty()) {
    queue_last_frame_=ros::WallTime();
    queue_empty_frames_ = 0;
    queue_empty_since_ = ros::WallTime();
    return;
  }
  // Unmasked frames still require valid geometry. Masked heartbeats above
  // never supply fresh geometry credit for dispatching the following target.
  queue_last_source_frame_ = ros::WallTime::now();
  tf::StampedTransform source_to_target, target_to_table, place_to_target, gripper, camera;
  const auto lookup = [this, &stamp](const std::string& to, const std::string& from,
                                     bool allow_recent_world_lag,
                                     tf::StampedTransform& transform) {
    lookupObservationTransform(tf_listener_, to, from, stamp,
                               allow_recent_world_lag, transform);
  };
  lookup(target_frame_, message.header.frame_id, false, source_to_target);// 获取坐标系变换
  lookup(table_frame_, target_frame_, true, target_to_table);
  lookup(target_frame_, place_frame_, true, place_to_target);
  // 无遮挡的接近阶段继续跟踪预留目标；确认自身遮挡时在上方冻结队列。
  // 夹紧后额外屏蔽夹爪邻域，避免手中物体再次入队。
  tf::Vector3 gripper_projection;
  bool has_projection = false;
  // 是否需要屏蔽夹爪邻域 ： 抓取状态下 且 夹紧后
  const bool exclude_gripper = (state == State::PICKING && grasp_secured_.load());
  if (exclude_gripper) {
    lookup(target_frame_, end_effector_link_, false, gripper);
    if (message.sensor_frame.empty()) {
      queue_last_frame_ = ros::WallTime();
      return;
    }
    lookup(target_frame_, message.sensor_frame, false, camera);
    // 夹爪在目标坐标系 - 相机在目标坐标系下的射线方向  向量方向
    const auto ray = gripper.getOrigin() - camera.getOrigin();
    if (std::abs(ray.z()) > 1e-6) {
      const double scale = (table_z_ + object_height_ - camera.getOrigin().z()) / ray.z();
      has_projection = scale > 0;
      gripper_projection = camera.getOrigin() + scale * ray;// 求夹爪在目标坐标系下的投影点（与相机共线）
    }
  }
  std::vector<ObjectQueue::Sample> samples;// 目标队列样本
  bool valid = true;
  for (const auto& object : message.objects)
  {
    const std::string& category = objectCategory(object);
    if (std::find(sort_classes_.begin(), sort_classes_.end(), category) == sort_classes_.end()) continue;// 判断是否是需要抓取的类别
    const auto& p = object.pose.position;// 目标在源坐标系下的位置
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) { valid = false; continue; }
    const tf::Vector3 point = source_to_target * tf::Vector3(p.x, p.y, p.z);
    const tf::Vector3 table_point = target_to_table * point;// 目标在桌子坐标系下的位置
    if (std::abs(table_point.x() - table_center_[0]) > table_size_[0] / 2 ||
        std::abs(table_point.y() - table_center_[1]) > table_size_[1] / 2) continue;
    bool excluded = false;
    for (const auto& place : place_targets_) {
      const auto destination = place_to_target * tf::Vector3(place.second[0], place.second[1], 0);
      // 判断目标与目标放置点的距离是否小于放置排除半径，如果小于则排除该目标
      if (std::hypot(point.x()-destination.x(), point.y()-destination.y()) < queue_place_exclusion_radius_)
        excluded = true;
    }
    // 判断目标是否在夹爪邻域内，如果在则排除该目标
    if (exclude_gripper && (point - gripper.getOrigin()).length() < queue_gripper_exclusion_radius_)
      excluded = true;
    // 判断目标是否在夹爪投影点邻域内，如果在则排除该目标
    if (has_projection && std::hypot(point.x()-gripper_projection.x(), point.y()-gripper_projection.y()) <
        queue_gripper_exclusion_radius_) excluded = true;
    if (excluded) continue;
    ObjectQueue::Sample sample;// 创建目标队列样本 
    sample.category = category;
    if (!std::isfinite(object.confidence) || object.confidence<0 || object.confidence>1) { valid=false; continue; }
    sample.confidence=object.confidence;
    sample.x = point.x(); sample.y = point.y(); sample.z = point.z();
    for (const auto& corner:object.footprint.points) {
      if (!std::isfinite(corner.x) || !std::isfinite(corner.y) || !std::isfinite(corner.z)) {valid=false; break;}
      const tf::Vector3 projected=source_to_target*tf::Vector3(corner.x,corner.y,corner.z);
      sample.footprint.push_back({{projected.x(),projected.y()}});
    }
    if (!object.footprint.points.empty() && (sample.footprint.size()!=4 || std::abs(signedArea(sample.footprint))<1e-10)) {
      valid=false; continue;
    }
    sample.payload = object;
    sample.payload.pose.position.x = point.x();
    sample.payload.pose.position.y = point.y();
    sample.payload.pose.position.z = point.z();
    // 使用观测高度还是默认高度
    const double height = object.object_height > 0 ? object.object_height : object_height_;
    sample.eligible = std::isfinite(height) && height > 0 &&
        std::abs(point.z() - (table_z_ + height / 2)) <= queue_height_tolerance_ &&
        (height_mode_ != "depth" || (object.depth_valid &&
         std::abs(point.z() - (table_z_ + height / 2)) <= height_tolerance_)) &&
        (!(use_detected_angle_ || use_detected_width_) || object.grasp_geometry_valid);
    // Cached geometry is displayed in target_frame even when angle control is disabled.
    const auto axis = source_to_target.getBasis() * tf::Vector3(std::cos(object.grasp_angle), std::sin(object.grasp_angle), 0);
    const bool angle_valid = std::isfinite(object.grasp_angle) && std::abs(axis.z()) < 0.1;
    sample.payload.grasp_angle = angle_valid ? std::atan2(axis.y(), axis.x()) :
        std::numeric_limits<double>::quiet_NaN();
    if (use_detected_angle_) sample.eligible = sample.eligible && angle_valid;
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
  if (observing && valid && samples.empty()) {// 如果是观察模式，且检测到有效目标，且目标队列为空
    instance_queue_.invalidate();
    if (observation_gap) { queue_empty_since_ = ros::WallTime(); queue_empty_frames_ = 0; }
    if (queue_empty_since_.isZero()) queue_empty_since_ = now;
    ++queue_empty_frames_;
  } else {
    queue_empty_since_ = ros::WallTime();
    queue_empty_frames_ = 0;
  }
}

bool SortingTask::validateReservedTarget()
{
  if (!continuous_sorting_ || active_instance_id_ == 0) return true;
  std::string reason;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const auto now = ros::WallTime::now();
    // 已领走的目标允许在机械臂接近时短暂遮挡；仍要求检测流新鲜且不能出现位移证据。
    const auto found = instance_queue_.tracks().find(active_instance_id_);
    if (found == instance_queue_.tracks().end() || found->second.status != ObjectQueue::Status::RESERVED)
      reason = "reservation no longer exists";
    else if (found->second.disturbed) {
      std::ostringstream detail;
      detail << "reserved target association changed: distance="
             << found->second.disturbance_distance << " m, observed category="
             << found->second.disturbance_category;
      reason = detail.str();
    }
    else if (now.toSec() - found->second.last_seen > queue_reserved_max_age_) {
      std::ostringstream detail;
      detail << "reserved target last seen " << now.toSec() - found->second.last_seen
             << " s ago (limit " << queue_reserved_max_age_ << " s)";
      reason = detail.str();
    } else {
      const bool known_occlusion=queue_vision_phase_.load()==QueueVisionPhase::APPROACH && queue_target_occluded_ &&
          !queue_occlusion_checked_at_.isZero() && (now-queue_occlusion_checked_at_).toSec()<=queue_frame_max_age_;
      const auto heartbeat=known_occlusion ? queue_last_source_frame_:queue_last_frame_;
      if (heartbeat.isZero() || (now-heartbeat).toSec()>queue_frame_max_age_)
        reason="valid observation stream unavailable before descent";
    }
  }
  if (!reason.empty()) setFailure("DETECTION_FAILED", reason);
  return reason.empty() && !stop_requested_.load();
}

void SortingTask::publishInstanceQueue()
{
  std::ostringstream stream;
  stream.precision(15);
  stream << "{\"schema_version\":2,\"frame_id\":\"" << detail::jsonEscape(target_frame_) << '"';
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const auto phase=queue_vision_phase_.load();
    stream << ",\"vision_phase\":\"" << (phase==QueueVisionPhase::MASKED ? "masked" :
        (phase==QueueVisionPhase::APPROACH ? "approach" : (phase==QueueVisionPhase::COLLECT ? "collect":"disabled")))
        << "\",\"target_occluded\":" << (queue_target_occluded_ ? "true":"false")
        << ",\"merged_detections_total\":" << instance_queue_.mergedDetectionsTotal() << ",\"targets\":{";
    bool first = true;
    const double now = ros::WallTime::now().toSec();
    for (const auto& item : instance_queue_.tracks()) {
      const auto& track = item.second;
      if (!first) stream << ',';
      first = false;
      stream << '"' << item.first << "\":{\"id\":" << item.first
             << ",\"category\":\"" << detail::jsonEscape(track.sample.category)
             << "\",\"color\":\"" << detail::jsonEscape(track.sample.category)
             << "\",\"status\":\"" << ObjectQueue::statusName(track.status)
             << "\",\"position\":[" << track.sample.x << ',' << track.sample.y << ',' << track.sample.z
             << "],\"grasp_geometry_valid\":" << (track.sample.payload.grasp_geometry_valid ? "true" : "false")
             << ",\"grasp_angle\":";
      if (track.sample.payload.grasp_geometry_valid && std::isfinite(track.sample.payload.grasp_angle))
        stream << track.sample.payload.grasp_angle;
      else stream << "null";
      stream << ",\"grasp_width\":";
      if (track.sample.payload.grasp_geometry_valid && std::isfinite(track.sample.payload.grasp_width) &&
          track.sample.payload.grasp_width > 0)
        stream << track.sample.payload.grasp_width;
      else stream << "null";
      stream << ",\"observations\":" << track.observations
             << ",\"detection_confidence\":" << track.sample.confidence
             << ",\"confidence\":" << std::min(1.0, static_cast<double>(track.observations)/instance_queue_.min_observations)
             << ",\"stability\":" << std::min(1.0, static_cast<double>(track.observations)/instance_queue_.min_observations)
             << ",\"age\":" << std::max(0.0, now-track.last_seen)
             << ",\"picked\":" << (track.status == ObjectQueue::Status::DONE ? "true" : "false") << '}';
    }
  }
  stream << "}}";
  std_msgs::String message;
  message.data = stream.str();
  target_cache_publisher_.publish(message);
}

bool SortingTask::continuousSortingOperation()
{
  // The observation operation owns the epoch reset.  Preserve the tracks built
  // while travelling to, and settling at, the observation pose.
  queue_vision_phase_.store(QueueVisionPhase::COLLECT);
  publishState(State::DETECTING, "building instance queue");
  auto deadline = ros::WallTime::now() + ros::WallDuration(detection_timeout_);
  ros::WallRate rate(20);
  while (ros::ok() && !stop_requested_.load()) {
    ObjectQueue::Track target;
    bool reserved = false, empty = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      const auto now = ros::WallTime::now();
      // A masked heartbeat does not validate the next pick. After retreat,
      // wait for a fresh unmasked frame before reserving another instance.
      // reserve() separately enforces each track's max_age.
      if (!queue_last_source_frame_.isZero() &&
          (now-queue_last_source_frame_).toSec() <= queue_frame_max_age_ &&
          !queue_last_frame_.isZero() &&
          (now-queue_last_frame_).toSec() <= queue_frame_max_age_)
        reserved = instance_queue_.reserve(sort_classes_, now.toSec(), target);
      empty = observation_ready_.load() && !queue_empty_since_.isZero() &&
          (now-queue_empty_since_).toSec() >= queue_empty_confirmation_ &&
          (now-queue_last_frame_).toSec() <= queue_frame_max_age_ && queue_empty_frames_ >= queue_empty_min_frames_;
    }
    if (reserved) {
      setQueueVisionPhase(QueueVisionPhase::APPROACH);
      observation_ready_.store(false);
      publishState(State::PICKING, target.sample.category + " #" + std::to_string(target.id));
      publishInstanceQueue();
      auto detected = target.sample.payload;
      detected.pose.position.x = target.sample.x;
      detected.pose.position.y = target.sample.y;
      detected.pose.position.z = target.sample.z;
      // Keep the scene refresh independent from a return to the observation pose.
      bool success = false;
      active_instance_id_ = target.id;
      grasp_secured_.store(false);
      const auto clear_active = [this](void*) { active_instance_id_ = 0; };
      std::unique_ptr<void, decltype(clear_active)> active_guard(this, clear_active);
      try { success = refreshOctomap() && addTableCollision() && pickAndPlace(detected); }
      catch (...) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        instance_queue_.finish(target.id, false, ros::WallTime::now().toSec());
        queue_vision_phase_.store(QueueVisionPhase::DISABLED);
        throw;
      }
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        instance_queue_.finish(target.id, success, ros::WallTime::now().toSec());
        queue_target_occluded_=false;
        // Reject any masked frame still waiting in the one-frame worker slot,
        // while preserving safe tracks collected earlier in this pick/place.
        queue_epoch_ = ros::Time::now();
        pending_detection_.reset();
        queue_empty_since_ = ros::WallTime();
        queue_empty_frames_ = 0;
      }
      publishInstanceQueue();
      if (!success) {
        queue_vision_phase_.store(QueueVisionPhase::DISABLED);
        return false; // Existing stop/recovery flow owns uncertain gripper state.
      }
      queue_vision_phase_.store(QueueVisionPhase::COLLECT);
      publishState(State::DETECTING, "next cached instance");
      deadline = ros::WallTime::now() + ros::WallDuration(target_cache_fallback_delay_);
      continue;
    }
    if (empty) {
      observation_ready_.store(false);
      queue_vision_phase_.store(QueueVisionPhase::DISABLED);
      if (!finish_named_target_.empty()) {
        publishState(State::HOMING, finish_named_target_);
        return moveNamed(finish_named_target_);
      }
      return true;
    }
    if (ros::WallTime::now() >= deadline) {
      if (observation_ready_.load()) {
        queue_vision_phase_.store(QueueVisionPhase::DISABLED);
        setFailure("DETECTION_FAILED", "no stable instance or valid sustained empty observation");
        return false;
      }
      publishState(State::OBSERVING, "queue needs a fresh observation");
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        // Start the new epoch before moving, so detections gathered on the way
        // to the observation pose contribute to the rebuilt queue.
        instance_queue_.invalidate();
        pending_detection_.reset();
        queue_epoch_ = ros::Time::now();
        queue_last_stamp_ = ros::Time();
        queue_last_frame_ = queue_last_source_frame_ = queue_empty_since_ = ros::WallTime();
        queue_empty_frames_ = 0;
      }
      queue_vision_phase_.store(QueueVisionPhase::COLLECT);
      if (!observation()) {
        queue_vision_phase_.store(QueueVisionPhase::DISABLED);
        return false;
      }
      publishState(State::DETECTING, "confirming remaining instances");
      deadline = ros::WallTime::now() + ros::WallDuration(detection_timeout_);
    }
    rate.sleep();
  }
  queue_vision_phase_.store(QueueVisionPhase::DISABLED);
  return false;
}
} // namespace aubo_sorting_core
