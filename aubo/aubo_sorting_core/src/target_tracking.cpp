#include <aubo_sorting_core/color_sorting_task.hpp>
#include "task_utils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace aubo_sorting_core
{
using detail::join;
using detail::jsonEscape;

void ColorSortingTask::detectionCallback(const aubo_perception::DetectedObjectArrayConstPtr& message)
{
  State state;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    detections_ = message;// 保存检测到的对象 头+检测到的对象数组
    detections_wall_time_ = ros::WallTime::now();
    state = state_;
  }
  if (state == State::DETECTING || (state == State::OBSERVING && observation_ready_.load()))
    updateTargetCache(*message);// 特定阶段更新目标抓取信息

  std::vector<std::string> counts;
  for (const std::string& color : sort_colors_)// 颜色统计
  {
    const int count = static_cast<int>(std::count_if(
        message->objects.begin(), message->objects.end(),
        [&color](const aubo_perception::DetectedObject& item) { return item.color == color; }));
    counts.push_back(color + ":" + std::to_string(count));// 颜色:数量
  }
  std_msgs::String summary;
  summary.data = join(counts, "  ");
  detection_summary_publisher_.publish(summary);// 发布检测到的对象统计信息
}

bool ColorSortingTask::detectionInCacheFrame(
    const aubo_perception::DetectedObjectArray& message,
    const aubo_perception::DetectedObject& detected, double& x, double& y)
{
  geometry_msgs::PoseStamped pose;// 更新目标对象的位姿
  pose.header = message.header;// 消息头
  if (pose.header.frame_id.empty())
    pose.header.frame_id = target_frame_;
  pose.pose = detected.pose;
  pose.pose.orientation.w = 1.0;
  if (pose.header.frame_id == target_cache_frame_)
  {
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    return true;
  }
  try
  {
    geometry_msgs::PoseStamped transformed;
    try
    {
      tf_listener_.transformPose(target_cache_frame_, pose, transformed);
    }
    catch (const tf::ExtrapolationException&)
    {
      pose.header.stamp = ros::Time(0);
      tf_listener_.transformPose(target_cache_frame_, pose, transformed);
    }
    x = transformed.pose.position.x;
    y = transformed.pose.position.y;
    return true;
  }
  catch (const tf::TransformException& error)
  {
    ROS_WARN_THROTTLE(2.0, "Cannot cache detections in %s: %s",
                      target_cache_frame_.c_str(), error.what());
    return false;
  }
}

void ColorSortingTask::updateTargetCache(const aubo_perception::DetectedObjectArray& message)
{
  // 主要是为了更新target_tracks_下的目标对象信息
  // 1、先对消息中的对象进行遍历取抓取颜色相同的目标对象中面积最大的一个
  // 2、将面积最大的目标对象转换到目标缓存坐标系下
  // 3、target_tracks_ 中若无 直接添加  
  //    若有 则更新目标对象的平均位置、观测次数、离散程度、最后观测时间 （两次平滑过度）
  std::map<std::string, const aubo_perception::DetectedObject*> selected;// 颜色+目标对象
  for (const auto& detected : message.objects) // 遍历检测到的对象数组 aubo_perception/DetectedObject[]
  {
    const auto found = selected.find(detected.color);
    if (found == selected.end() || detected.contour_area > found->second->contour_area)// 如果没有找到|检测到的面积大于已选面积
      selected[detected.color] = &detected;// 存储面积最大的目标对象 颜色:目标对象
  }
  struct Update { std::string color; double x; double y; };
  std::vector<Update> updates;
  for (const auto& item : selected)
  {
    double x = 0.0;
    double y = 0.0;
    if (detectionInCacheFrame(message, *item.second, x, y))// 将检测到的目标对象转换到目标缓存坐标系下
      updates.push_back({item.first, x, y});// 颜色:目标对象的x,y坐标
  }
  if (updates.empty())
    return;

  bool changed = false;
  const ros::WallTime now = ros::WallTime::now();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const Update& update : updates)
    {
      auto found = target_tracks_.find(update.color);// 在目标跟踪中查找更新了颜色的目标对象
      if (found == target_tracks_.end())// 没找到  首次观测到
      {
        TargetTrack track;
        track.x = update.x;
        track.y = update.y;
        track.count = 1;
        track.last_seen = now;
        target_tracks_[update.color] = track;
        changed = true;
        continue;
      }
      TargetTrack& track = found->second;// 找到了 取TargetTrack结构体
      if (track.picked)
        continue;
      const double distance = std::hypot(update.x - track.x, update.y - track.y);// 欧几里得距离
      if (distance > target_cache_outlier_distance_)// 超出异常距离
      {
        ROS_WARN_THROTTLE(2.0, "Rejecting %s target-cache outlier %.3f m from track",
                          update.color.c_str(), distance);
        continue;
      }
      ++track.count;// 增加该目标被检测到的次数
      const double delta_x = update.x - track.x;
      const double delta_y = update.y - track.y;
      track.x += delta_x / track.count;
      track.y += delta_y / track.count;
      track.m2 += delta_x * (update.x - track.x) + delta_y * (update.y - track.y);
      track.last_seen = now;
      changed = true;
    }
  }
  if (changed)
    publishTargetCache();
}

void ColorSortingTask::publishTargetCache()
{
  std::map<std::string, TargetTrack> tracks;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    tracks = target_tracks_;
  }
  std::ostringstream stream;
  stream.precision(15);
  stream << "{\"frame_id\":\"" << jsonEscape(target_cache_frame_) << "\",\"targets\":{";
  const ros::WallTime now = ros::WallTime::now();
  bool first = true;
  for (const auto& item : tracks)
  {
    const TargetTrack& track = item.second;
    const double spread = std::sqrt(std::max(0.0, track.m2) / std::max(1, track.count));// 面积的平方根/被检测到的次数  表示目标的大小
    const double observation_confidence =
        std::min(1.0, static_cast<double>(track.count) / target_cache_min_observations_);// 观测置信度  被检测到的次数/最小观测次数
    const double stability_confidence =
        std::max(0.0, 1.0 - spread / target_cache_outlier_distance_);// 稳定置信度  1-目标的大小/异常距离
    if (!first)
      stream << ',';
    first = false;
    stream << '"' << jsonEscape(item.first) << "\":{"
           << "\"position\":[" << track.x << ',' << track.y << "],"
           << "\"observations\":" << track.count << ','
           << "\"confidence\":" << observation_confidence * stability_confidence << ','
           << "\"spread\":" << spread << ','
           << "\"age\":" << std::max(0.0, (now - track.last_seen).toSec()) << ','
           << "\"picked\":" << (track.picked ? "true" : "false") << '}';
  }
  stream << "}}";
  std_msgs::String message;
  message.data = stream.str();
  target_cache_publisher_.publish(message);
}

void ColorSortingTask::markTargetPicked(const std::string& color)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto found = target_tracks_.find(color);
    if (found != target_tracks_.end())
      found->second.picked = true;
  }
  publishTargetCache();
}

bool ColorSortingTask::cachedObject(const std::string& color,
                                    aubo_perception::DetectedObject& detected)
{
  // 使用目标缓存中的目标对象 存在时间|未被抓取|观测次数
  if (!target_cache_fallback_enabled_ || height_mode_ == "depth" || use_detected_angle_ || use_detected_width_)
    return false;
  TargetTrack track;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto found = target_tracks_.find(color);
    if (found == target_tracks_.end())
      return false;
    track = found->second;// 使用target_tracks_中保存的目标对象信息
  }
  const double age = (ros::WallTime::now() - track.last_seen).toSec();
  // 目标已被抓取|观测次数小于最小观测次数|目标年龄大于最大年龄
  if (track.picked || track.count < target_cache_min_observations_ || age > target_cache_max_age_)
    return false;
  double x = 0.0;
  double y = 0.0;
  if (!xyInTargetFrame(target_cache_frame_, {track.x, track.y}, x, y))// 将目标对象的平均位置转换到目标缓存坐标系下
    return false;
  detected = aubo_perception::DetectedObject();
  detected.color = color;
  detected.pose.position.x = x;
  detected.pose.position.y = y;
  detected.pose.position.z = table_z_ + 0.5 * object_height_;
  detected.pose.orientation.w = 1.0;
  ROS_WARN("Using cached %s target after %.1f s without a fresh detection: [%.3f, %.3f] in %s (%d observations)",
           color.c_str(), age, track.x, track.y, target_cache_frame_.c_str(), track.count);
  return true;
}

bool ColorSortingTask::waitForObject(const std::string& color, const ros::WallTime& not_before,
                                     aubo_perception::DetectedObject& detected)
{
  // |←── 新鲜检测优先 ──→|target_cache_fallback_delay_|←── 缓存开始兜底 ──→|detection_timeout_|←── 硬超时 ──→
  // 就是为了多帧图像来进行目标检测的平均位置计算
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(detection_timeout_);// 目标检测超时时间
  const ros::WallTime cache_deadline =
      ros::WallTime::now() + ros::WallDuration(target_cache_fallback_delay_);// 目标回退时间
  ros::WallTime last_receipt = not_before;
  std::vector<aubo_perception::DetectedObject> samples;
  ros::WallRate rate(10.0);
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    if (stop_requested_.load())
      return false;
    aubo_perception::DetectedObjectArrayConstPtr detections;
    ros::WallTime receipt;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      detections = detections_;
      receipt = detections_wall_time_;
    }
    if (detections && receipt > last_receipt && detections->header.frame_id == target_frame_ &&
        !detections->header.stamp.isZero() &&
        (ros::Time::now() - detections->header.stamp).toSec() >= 0.0 &&
        (ros::Time::now() - detections->header.stamp).toSec() <= 1.0)
    {
      last_receipt = receipt;
      const aubo_perception::DetectedObject* largest = nullptr;
      for (const auto& candidate : detections->objects)
        if (candidate.color == color && (!largest || candidate.contour_area > largest->contour_area))
          largest = &candidate;// 找到最大的目标且颜色对应的对象
      if (largest)
      {
        // 同类多个目标时避免把相隔较远的目标平均到两者中间。
        if (!samples.empty() && std::hypot(largest->pose.position.x - samples.back().pose.position.x,
                                         largest->pose.position.y - samples.back().pose.position.y) > 0.03)
          samples.clear();
        samples.push_back(*largest);// 进行累计采样 当采样次数达到指定值时 计算平均位置
        if (static_cast<int>(samples.size()) >= detection_samples_)
        {
          detected = samples.back();
          detected.pose.position.x = 0.0;
          detected.pose.position.y = 0.0;
          detected.pose.position.z = 0.0;
          for (const auto& sample : samples)
          {
            detected.pose.position.x += sample.pose.position.x;
            detected.pose.position.y += sample.pose.position.y;
            detected.pose.position.z += sample.pose.position.z;
          }
          detected.pose.position.x /= samples.size();// 计算平均位置
          detected.pose.position.y /= samples.size();
          detected.pose.position.z /= samples.size();
          ROS_INFO("Averaged %zu '%s' detections at [%.3f, %.3f]", samples.size(),
                   color.c_str(), detected.pose.position.x, detected.pose.position.y);
          return true;
        }
      }
    }
    if (ros::WallTime::now() >= cache_deadline && cachedObject(color, detected))// 目标回退时间到时 使用目标缓存中的目标对象
      return true;
    rate.sleep();
  }
  if (cachedObject(color, detected))
    return true;
  setFailure("DETECTION_FAILED", "no fresh or confident cached '" + color + "' target");
  ROS_ERROR("No fresh '%s' object detected within %.1f seconds", color.c_str(), detection_timeout_);
  return false;
}

bool ColorSortingTask::verifyVisibleColors()
{
  // 光照可能变化、物体可能被遮挡、机械臂移动可能导致相机视角偏移，需要确认确实能看清所有颜色再开始抓取
  // 就是检测当前得到的几帧图像中 是否包含了所有需要验证的颜色
  std::set<std::string> required;// 需要验证的颜色集合
  for (const std::string& color : sort_colors_)
    if (completed_colors_.count(color) == 0)
      required.insert(color);
  if (required.empty())
  {
    ROS_INFO("Observation verification skipped: all colors completed");
    return true;
  }
  const ros::WallTime start = ros::WallTime::now();
  const ros::WallTime deadline = start + ros::WallDuration(observation_verification_timeout_);
  ros::WallTime last_receipt = start;
  std::map<std::string, int> counts;
  for (const auto& color : required)
    counts[color] = 0;
  ros::WallRate rate(10.0);
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    if (stop_requested_.load())
      return false;
    aubo_perception::DetectedObjectArrayConstPtr detections;
    ros::WallTime receipt;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      detections = detections_;
      receipt = detections_wall_time_;
    }
    if (detections && receipt > last_receipt)
    {
      last_receipt = receipt;
      std::set<std::string> visible;
      for (const auto& item : detections->objects)
        visible.insert(item.color);// 记录当前帧中可见的颜色
      for (const auto& color : required)
        if (visible.count(color))
          ++counts[color];
      bool complete = true;
      for (const auto& color : required)
        complete = complete && counts[color] >= observation_verification_min_frames_;
      if (complete)
      {
        std::vector<std::string> values;
        for (const auto& item : counts)
          values.push_back(item.first + "=" + std::to_string(item.second));
        ROS_INFO_STREAM("Observation verified all colors across frames: " << join(values, ", "));
        return true;
      }
    }
    rate.sleep();
  }
  std::vector<std::string> count_values;
  std::vector<std::string> missing;
  for (const auto& item : counts)
  {
    count_values.push_back(item.first + ":" + std::to_string(item.second));
    if (item.second < observation_verification_min_frames_)
      missing.push_back(item.first);
  }
  ROS_WARN("Observation pose did not show all required colors within %.1f seconds: required_frames=%d, counts=[%s], missing=[%s]",
           observation_verification_timeout_, observation_verification_min_frames_,
           join(count_values, ", ").c_str(), join(missing, ", ").c_str());
  return false;
}

}  // namespace aubo_sorting_core
