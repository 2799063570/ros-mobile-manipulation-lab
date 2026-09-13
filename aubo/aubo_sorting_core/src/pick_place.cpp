#include <aubo_sorting_core/color_sorting_task.hpp>
#include "task_utils.hpp"
#include <algorithm>
#include <cmath>

namespace aubo_sorting_core
{
using detail::wallSleep;

bool ColorSortingTask::pickAndPlace(const aubo_perception::DetectedObject& detected)
{
  // Only a failed pick/place owns attachment cleanup. Other operations cannot release it.
  bool completed = false;
  bool attachment_attempted = false;
  const auto cleanup = [this, &completed, &attachment_attempted](void*) {
    if (!completed && attachment_attempted)
      releaseAttachedObjectNoWait();
  };
  std::unique_ptr<void, decltype(cleanup)> attachment_guard(this, cleanup);
  const std::string color = detected.color;
  const double object_x = detected.pose.position.x + grasp_offset_x_;// 抓取偏移值
  const double object_y = detected.pose.position.y + grasp_offset_y_;// 抓取偏移值
  const double height = detected.object_height > 0.0 ? detected.object_height : object_height_;
  const double expected_center = table_z_ + 0.5 * height;
  const double center_z = height_mode_ == "depth" ? detected.pose.position.z : expected_center;
  double close_command = gripper_closed_;
  if (!std::isfinite(object_x) || !std::isfinite(object_y) || !std::isfinite(center_z) ||
      !std::isfinite(height) || height <= 0.0 ||
      (height_mode_ == "depth" && (!detected.depth_valid || std::abs(center_z - expected_center) > height_tolerance_)) ||
      ((use_detected_angle_ || use_detected_width_) && !detected.grasp_geometry_valid) ||
      (use_detected_angle_ && !std::isfinite(detected.grasp_angle)))
  {
    setFailure("DETECTION_FAILED", "invalid or unvalidated grasp geometry/height");
    return false;
  }
  if (use_detected_width_)
  {
    const double opening = detected.grasp_width * width_close_scale_;
    if (!std::isfinite(opening) || detected.grasp_width <= 0.0 ||
        detected.grasp_width > gripper_width_open_ || opening < gripper_width_closed_ || opening > gripper_width_open_)
    {
      setFailure("DETECTION_FAILED", "object width outside calibrated gripper range");
      return false;
    }
    // 米制窄边乘闭合系数，再按实测两端开口换算为关节位置。
    close_command = gripper_open_ + (gripper_width_open_ - opening) /
        (gripper_width_open_ - gripper_width_closed_) * (gripper_closed_ - gripper_open_);
  }
  const double grasp_z = center_z + grasp_height_offset_;
  if (grasp_z <= table_z_ || grasp_z >= table_z_ + std::min(pregrasp_height_, lift_min_height_))
  {
    setFailure("DETECTION_FAILED", "grasp height outside approach/lift clearance");
    return false;
  }
  // 短边为夹爪闭合方向；grasp_rpy.yaw 用于补偿 TCP 与夹爪轴的标定偏差。
  active_grasp_angle_ = use_detected_angle_ ? detected.grasp_angle : 0.0;
  const auto reset_angle = [this](void*) { active_grasp_angle_ = 0.0; };
  std::unique_ptr<void, decltype(reset_angle)> angle_guard(this, reset_angle);
  const double preplace_z = table_z_ + preplace_height_;// 放置前及放置后的退离高度
  ROS_INFO("Picking %s at [%.3f, %.3f, %.3f]", color.c_str(), object_x, object_y, grasp_z);

  if (!validateReservedTarget() || !commandGripper(gripper_open_) ||
      !moveToPose(makePose(object_x, object_y, table_z_ + pregrasp_height_), color + " pre-grasp") ||
      !validateReservedTarget() ||
      !cartesianTo(makePose(object_x, object_y, grasp_z), color + " grasp"))// 夹爪张开 移动到目标位置上方 移动到目标位置
    return false;
  const auto model = grasp_model_names_.find(color);// 根据颜色查找抓取碰撞体名称
  std::string object_model_name =
      model == grasp_model_names_.end() ? color + "_block" : model->second;
  if (continuous_sorting_ && use_grasp_attachment_)
    object_model_name = "nearest:" + object_model_name;
  attachment_attempted = use_grasp_attachment_;
  if (!setGraspAttachment(object_model_name, true))// 夹爪吸附目标物体
    return false;
  if (continuous_sorting_ && use_grasp_attachment_) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    object_model_name = attached_model_; // Plugin resolved the actual same-class model instance.
  }
  if (!commandGripper(close_command))// 夹爪闭合
  {
    setGraspAttachment(object_model_name, false);// 夹爪闭合失败 释放吸附
    return false;
  }
  // 夹爪夹紧后目标可能随手移动；此时才排除夹爪附近的检测，防止手中物体重新入队。
  grasp_secured_.store(true);
  if (!wallSleep(0.5, stop_requested_) ||
      !liftWithRecovery(object_x, object_y, color + " lift"))// 抬升到指定高度
    return false;

  const auto place = place_targets_.find(color);
  if (place == place_targets_.end())
  {
    ROS_ERROR_STREAM("No place target configured for color '" << color << "'");
    return false;
  }
  double place_x = 0.0;
  double place_y = 0.0;
  if (!xyInTargetFrame(place_frame_, place->second, place_x, place_y) ||      // 将目标放置位置转换到目标坐标系下
      !moveToPose(makePose(place_x, place_y, preplace_z), color + " pre-place") ||  // 移动到目标放置位置上方
      !cartesianTo(makePose(place_x, place_y, expected_center + grasp_height_offset_ + place_clearance_), color + " place") || // 移动到目标放置位置
      !commandGripper(gripper_open_) || !setGraspAttachment(object_model_name, false) || // 夹爪张开 释放吸附
      !wallSleep(0.5, stop_requested_))
    return false;
  completed = cartesianTo(makePose(place_x, place_y, preplace_z), color + " retreat");
  return completed; // 移动到目标放置位置上方
}

bool ColorSortingTask::sortingOperation()
{
  if (continuous_sorting_)
    return continuousSortingOperation();
  bool all_complete = !sort_colors_.empty();// 分拣的颜色列表不为空
  for (const auto& color : sort_colors_)
    all_complete = all_complete && completed_colors_.count(color) != 0;// 检查所有颜色是否已经完成
  if (all_complete)
  {
    completed_colors_.clear();// 若都完成分拣了  清空已完成颜色列表
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      for (auto& item : target_tracks_)
        item.second.picked = false;// 重置所有目标为未拾取
    }
    publishTargetCache();
  }
  for (std::size_t index = 0; index < sort_colors_.size(); ++index)
  {
    const std::string& color = sort_colors_[index];
    if (stop_requested_.load())
      return false;
    if (completed_colors_.count(color))
    {
      ROS_INFO("Skipping completed color '%s' in workspace '%s'", color.c_str(), workspace_id_.c_str());
      continue;
    }
    publishState(State::DETECTING, color);
    aubo_perception::DetectedObject detected;// 检测到的目标对象消息：颜色、面积、位置
    if (!waitForObject(color, ros::WallTime::now(), detected))// 获取目标对象消息
      return false;
    observation_ready_.store(false);
    publishState(State::PICKING, color);
    if (!pickAndPlace(detected))
      return false;
    completed_colors_.insert(color);// 标记一下 该颜色物体已经完成抓取了
    markTargetPicked(color);
    if (index + 1 < sort_colors_.size())
    {
      publishState(State::OBSERVING, "next object");
      if (!observation())// 移动到观察位姿 并确认检测到的目标
        return false;
    }
  }
  observation_ready_.store(false);
  if (!finish_named_target_.empty())
  {
    publishState(State::HOMING, finish_named_target_);
    return moveNamed(finish_named_target_);
  }
  return true;
}

}  // namespace aubo_sorting_core
