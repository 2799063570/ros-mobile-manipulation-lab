#include <aubo_sorting_core/color_sorting_task.hpp>
#include "task_utils.hpp"
#include <aubo_sorting_core/height_recovery.hpp>
#include <urdf/model.h>
#include <cmath>
#include <stdexcept>

// 两个函数 一个是从参数服务器加载参数 另一个是验证上臂关节限制是否正确
namespace aubo_sorting_core
{
  using detail::xmlVector;

  void ColorSortingTask::loadParameters()
  {
    private_nh_.param<std::string>("planning_group", group_name_, "aubo_i5");
    private_nh_.param<std::string>("end_effector_link", end_effector_link_, "tcp_link");
    private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
    private_nh_.param<std::string>("detections_topic", detections_topic_, "/sorting/detections");
    private_nh_.param<std::string>("gripper_action", gripper_action_name_,
                                   "/gripper_controller/follow_joint_trajectory");
    private_nh_.param<std::string>("gripper_backend", gripper_backend_, "trajectory");
    private_nh_.param("inspire_speed", inspire_speed_, 500);
    private_nh_.param("inspire_force", inspire_force_, 100);
    private_nh_.param("inspire_motion_timeout", inspire_motion_timeout_, 5.0);
    if ((gripper_backend_ != "trajectory" && gripper_backend_ != "inspire") ||
        inspire_speed_ < 1 || inspire_speed_ > 1000 || inspire_force_ < 50 ||
        inspire_force_ > 1000 || !std::isfinite(inspire_motion_timeout_) || inspire_motion_timeout_ <= 0.0)
      throw std::runtime_error("invalid gripper backend/speed/force/settle time");
    private_nh_.param<std::string>("height_mode", height_mode_, "table");
    private_nh_.param("use_detected_angle", use_detected_angle_, false);
    private_nh_.param("use_detected_width", use_detected_width_, false);
    private_nh_.param("height_tolerance", height_tolerance_, 0.02);
    private_nh_.param("gripper_width_open", gripper_width_open_, -1.0);
    private_nh_.param("gripper_width_closed", gripper_width_closed_, -1.0);
    private_nh_.param("width_close_scale", width_close_scale_, 0.9);
    if ((height_mode_ != "table" && height_mode_ != "depth") ||
        !std::isfinite(height_tolerance_) || height_tolerance_ < 0.0)
      throw std::runtime_error("invalid height_mode / height_tolerance");
    if (use_detected_width_ && (gripper_backend_ != "trajectory" ||
                                !std::isfinite(gripper_width_open_) || !std::isfinite(gripper_width_closed_) ||
                                gripper_width_closed_ < 0.0 || gripper_width_open_ <= gripper_width_closed_ ||
                                !std::isfinite(width_close_scale_) || width_close_scale_ <= 0.0 || width_close_scale_ > 1.0))
      throw std::runtime_error("detected width requires trajectory backend and calibrated opening endpoints/scale");
    private_nh_.param("table_z", table_z_, 0.14);
    private_nh_.param<std::string>("table_frame", table_frame_, target_frame_);
    private_nh_.param("table_collision_margin", table_collision_margin_, 0.0);
    table_collision_margin_ = std::max(0.0, table_collision_margin_);
    private_nh_.param("object_height", object_height_, 0.04);
    private_nh_.param("grasp_height_offset", grasp_height_offset_, 0.01);
    private_nh_.param<std::string>("observation_named_target", observation_named_target_, "");
    private_nh_.param<std::string>("work_ready_named_target", work_ready_named_target_, "work_ready");
    private_nh_.param("pregrasp_height", pregrasp_height_, 0.25);
    private_nh_.param("lift_height", lift_height_, 0.30);
    private_nh_.param("lift_min_height", lift_min_height_, lift_height_);
    private_nh_.param("lift_height_step", lift_height_step_, 0.02);
    private_nh_.param("lift_max_attempts", lift_max_attempts_, 5);
    liftHeightCandidates(lift_height_, lift_min_height_, lift_height_step_, lift_max_attempts_);
    if (lift_min_height_ <= 0.5 * object_height_ + grasp_height_offset_)
      throw std::runtime_error("lift_min_height must exceed the grasp height above the table");
    private_nh_.param("preplace_height", preplace_height_, lift_height_);
    if (!std::isfinite(preplace_height_) || preplace_height_ <= 0.0)
      throw std::runtime_error("preplace_height must be finite and positive");
    private_nh_.param("place_clearance", place_clearance_, 0.02);
    private_nh_.param("cartesian_step", cartesian_step_, 0.01);
    private_nh_.param("minimum_cartesian_fraction", minimum_cartesian_fraction_, 0.90);
    private_nh_.param("gripper_open", gripper_open_, 0.0);
    private_nh_.param("gripper_closed", gripper_closed_, 0.28);
    private_nh_.param("gripper_motion_time", gripper_motion_time_, 0.8);
    if (!std::isfinite(gripper_motion_time_) || gripper_motion_time_ <= 0.0)
      throw std::runtime_error("gripper_motion_time must be positive");
    private_nh_.param("gripper_contact_tolerance", gripper_contact_tolerance_, 0.30);
    private_nh_.param("use_grasp_attachment", use_grasp_attachment_, true);
    private_nh_.param<std::string>("grasp_attach_topic", grasp_attach_topic_, "/sorting/grasp/attach");
    private_nh_.param<std::string>("grasp_detach_topic", grasp_detach_topic_, "/sorting/grasp/detach");
    private_nh_.param<std::string>("grasp_status_topic", grasp_status_topic_, "/sorting/grasp/status");
    private_nh_.param("grasp_attachment_timeout", grasp_attachment_timeout_, 3.0); // 抓取吸附插件加载超时时间
    private_nh_.param<std::string>("place_frame", place_frame_, target_frame_);
    private_nh_.param("detection_timeout", detection_timeout_, 15.0);
    private_nh_.param("detection_samples", detection_samples_, 8);
    detection_samples_ = std::max(1, detection_samples_);
    private_nh_.param("detection_settle_time", detection_settle_time_, 1.0);
    private_nh_.param("verify_observation_detections", verify_observation_detections_, false); // 是否在每次移动到观察位置 后验证检测到的目标颜色是否与预期一致
    private_nh_.param("observation_verification_timeout", observation_verification_timeout_, 4.0);
    private_nh_.param("observation_verification_min_frames", observation_verification_min_frames_, 1);
    observation_verification_min_frames_ = std::max(1, observation_verification_min_frames_);
    private_nh_.param("grasp_offset_x", grasp_offset_x_, 0.0);
    private_nh_.param("grasp_offset_y", grasp_offset_y_, 0.0);
    private_nh_.param("velocity_scaling", velocity_scaling_, 0.15);
    private_nh_.param("acceleration_scaling", acceleration_scaling_, 0.15);
    private_nh_.param("gripper_server_timeout", gripper_server_timeout_, 30.0);
    private_nh_.param<std::string>("finish_named_target", finish_named_target_, "down");
    private_nh_.param("scene_update_timeout", scene_update_timeout_, 10.0);
    private_nh_.param("auto_move_to_observation", auto_move_to_observation_, true);
    private_nh_.param("auto_start", auto_start_, false);
    private_nh_.param("require_octomap", require_octomap_, false);
    private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_,
                                   "/workspace_camera/depth/color/points");
    private_nh_.param<std::string>("planning_scene_topic", planning_scene_topic_,
                                   "/move_group/monitored_planning_scene");
    private_nh_.param<std::string>("clear_octomap_service", clear_octomap_service_, "/clear_octomap");
    private_nh_.param("octomap_wait_timeout", octomap_wait_timeout_, 30.0);
    private_nh_.param<std::string>("base_lock_topic", base_lock_topic_, "/sorting/base_locked");
    private_nh_.param<std::string>("target_cache_frame", target_cache_frame_, target_frame_);
    private_nh_.param("target_cache_min_observations", target_cache_min_observations_, 5);
    target_cache_min_observations_ = std::max(2, target_cache_min_observations_);
    private_nh_.param("target_cache_max_age", target_cache_max_age_, 30.0);
    target_cache_max_age_ = std::max(1.0, target_cache_max_age_);
    private_nh_.param("target_cache_outlier_distance", target_cache_outlier_distance_, 0.12);
    target_cache_outlier_distance_ = std::max(0.02, target_cache_outlier_distance_);
    private_nh_.param("target_cache_fallback_enabled", target_cache_fallback_enabled_, true);
    private_nh_.param("target_cache_fallback_delay", target_cache_fallback_delay_, 2.0);
    target_cache_fallback_delay_ = std::max(0.2, target_cache_fallback_delay_);
    private_nh_.param("continuous_sorting", continuous_sorting_, true);
    private_nh_.param("queue_match_distance", instance_queue_.match_distance, 0.04);
    private_nh_.param("queue_stable_distance", instance_queue_.stable_distance, 0.015);
    private_nh_.param("queue_duplicate_distance", instance_queue_.duplicate_distance, 0.008);
    private_nh_.param("queue_max_age", instance_queue_.max_age, 10.0);
    private_nh_.param("queue_confirmation_gap", instance_queue_.confirmation_gap, 1.0);
    private_nh_.param("queue_retention", instance_queue_.retention, 30.0);
    private_nh_.param("queue_done_hold", instance_queue_.done_hold, 2.0);
    instance_queue_.min_observations = target_cache_min_observations_;
    private_nh_.param("queue_frame_max_age", queue_frame_max_age_, 1.0);
    private_nh_.param("queue_empty_confirmation", queue_empty_confirmation_, 2.0);
    private_nh_.param("queue_empty_min_frames", queue_empty_min_frames_, 5);
    private_nh_.param("queue_place_exclusion_radius", queue_place_exclusion_radius_, 0.08);
    private_nh_.param("queue_gripper_exclusion_radius", queue_gripper_exclusion_radius_, 0.10);
    private_nh_.param("queue_height_tolerance", queue_height_tolerance_, 0.04);
    for (double value : {instance_queue_.match_distance, instance_queue_.stable_distance,
         instance_queue_.duplicate_distance, instance_queue_.max_age, instance_queue_.confirmation_gap,
         instance_queue_.retention, instance_queue_.done_hold, queue_frame_max_age_,
         queue_empty_confirmation_, queue_place_exclusion_radius_, queue_gripper_exclusion_radius_,
         queue_height_tolerance_})
      if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error("queue parameters must be finite and positive");
    if (instance_queue_.duplicate_distance >= instance_queue_.stable_distance ||
        instance_queue_.stable_distance >= instance_queue_.match_distance ||
        instance_queue_.retention < instance_queue_.max_age || queue_empty_min_frames_ < 2 ||
        queue_empty_confirmation_ >= detection_timeout_)
      throw std::runtime_error("invalid queue distance/retention/empty-confirmation bounds");
    private_nh_.param<std::string>("failure_topic", failure_topic_, "/sorting/failure");
    private_nh_.param<std::string>("workspace_config_param", workspace_config_param_,
                                   "/sorting/workspace_config");
    private_nh_.param<std::string>("workspace_update_topic", workspace_update_topic_,
                                   "/sorting/workspace_update");
    private_nh_.param("planning_time", planning_time_, 12.0);

    table_center_ = {0.80, 0.0, -0.06};
    table_size_ = {0.80, 1.20, 0.40};
    grasp_rpy_ = {3.14159265358979323846, 0.0, 0.0};
    observation_pose_ = {0.58, 0.0, 0.62};
    sort_colors_ = {"red", "green", "blue"}; // 要进行分拣的颜色列表
    private_nh_.getParam("table_center", table_center_);
    private_nh_.getParam("table_size", table_size_);
    private_nh_.getParam("grasp_rpy", grasp_rpy_);
    private_nh_.getParam("observation_pose", observation_pose_);
    private_nh_.getParam("sort_colors", sort_colors_);

    XmlRpc::XmlRpcValue mappings;
    if (private_nh_.getParam("grasp_model_names", mappings) &&
        mappings.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      for (auto iterator = mappings.begin(); iterator != mappings.end(); ++iterator)
        grasp_model_names_[iterator->first] = static_cast<std::string>(iterator->second);
    }
    if (!private_nh_.getParam("place_targets", mappings) ||
        mappings.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      throw std::runtime_error("required private parameter '~place_targets' is missing or invalid");
    for (auto iterator = mappings.begin(); iterator != mappings.end(); ++iterator)
      place_targets_[iterator->first] = xmlVector(iterator->second, 2, "place_targets." + iterator->first);

    if (table_center_.size() != 3 || table_size_.size() != 3 || grasp_rpy_.size() != 3 ||
        observation_pose_.size() != 3)
      throw std::runtime_error("table_center, table_size, grasp_rpy and observation_pose must each have 3 values");
  }

  bool ColorSortingTask::verifyLoadedUpperArmLimit() const
  {
    std::string description;
    if (!nh_.getParam("/robot_description", description))
    {
      ROS_ERROR("Unable to verify upperArm_joint limits: /robot_description is missing");
      return false;
    }
    urdf::Model model;
    if (!model.initString(description))
    {
      ROS_ERROR("Unable to parse /robot_description");
      return false;
    }
    const urdf::JointConstSharedPtr joint = model.getJoint("upperArm_joint");
    if (!joint || !joint->limits)
    {
      ROS_ERROR("Unable to verify upperArm_joint limits: joint or limits are missing");
      return false;
    }
    double moveit_lower = 0.0;
    double moveit_upper = 0.0;
    const std::string prefix = "/robot_description_planning/joint_limits/upperArm_joint/";
    if (!nh_.getParam(prefix + "min_position", moveit_lower) ||
        !nh_.getParam(prefix + "max_position", moveit_upper))
    {
      ROS_ERROR("Unable to verify upperArm_joint MoveIt limits");
      return false;
    }
    const double expected_lower = -1.0471976;
    const double expected_upper = 1.0471976;
    ROS_INFO("Loaded upperArm_joint limits: URDF [%.6f, %.6f], MoveIt [%.6f, %.6f] rad",
             joint->limits->lower, joint->limits->upper, moveit_lower, moveit_upper);
    const bool valid = std::abs(joint->limits->lower - expected_lower) <= 1.0e-5 &&
                       std::abs(joint->limits->upper - expected_upper) <= 1.0e-5 &&
                       std::abs(moveit_lower - expected_lower) <= 1.0e-5 &&
                       std::abs(moveit_upper - expected_upper) <= 1.0e-5;
    if (!valid)
      ROS_ERROR("Stale robot model detected; expected upperArm_joint limits [-1.047198, 1.047198] rad");
    return valid;
  }

} // namespace aubo_sorting_core
