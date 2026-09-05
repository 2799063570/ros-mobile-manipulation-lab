#include <aubo_sorting_core/color_sorting_task.hpp>

#include <actionlib_msgs/GoalStatus.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <control_msgs/JointTolerance.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace aubo_sorting_core
{
namespace
{
bool wallSleep(double seconds, const std::atomic<bool>& stop_requested)
{
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(seconds);
  ros::WallRate rate(50.0);
  while (ros::ok() && !stop_requested.load() && ros::WallTime::now() < deadline)
    rate.sleep();
  return ros::ok() && !stop_requested.load();
}

std::string join(const std::vector<std::string>& values, const std::string& separator)
{
  // 该函数的作用吧字符串数组中各个字符串连接起来 中间用{separator}分隔  返回连接后的字符串
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index)
  {
    if (index)
      stream << separator;
    stream << values[index];
  }
  return stream.str();
}

std::string jsonEscape(const std::string& value)
{
  std::ostringstream stream;
  for (const unsigned char character : value)
  {
    switch (character)
    {
      case '"': stream << "\\\""; break;
      case '\\': stream << "\\\\"; break;
      case '\b': stream << "\\b"; break;
      case '\f': stream << "\\f"; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default:
        if (character < 0x20)
        {
          stream << "\\u00";
          const char* digits = "0123456789abcdef";
          stream << digits[(character >> 4) & 0x0f] << digits[character & 0x0f];
        }
        else
          stream << character;
    }
  }
  return stream.str();
}

double xmlNumber(const XmlRpc::XmlRpcValue& value)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    return static_cast<double>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return static_cast<int>(value);
  throw std::runtime_error("expected a numeric value");
}

std::vector<double> xmlVector(const XmlRpc::XmlRpcValue& value, int expected_size,
                              const std::string& name)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != expected_size)
    throw std::runtime_error("'" + name + "' must contain " + std::to_string(expected_size) +
                             " numbers");
  std::vector<double> result;
  result.reserve(expected_size);
  for (int index = 0; index < expected_size; ++index)
    result.push_back(xmlNumber(value[index]));
  return result;
}
}  // namespace

ColorSortingTask::ColorSortingTask(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(std::move(nh)), private_nh_(std::move(private_nh)), tf_listener_(ros::Duration(30.0))
{
  loadParameters();
  robot_limits_valid_ = verifyLoadedUpperArmLimit();// 检证机械臂基座数第二个关节 

  arm_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));// 初始化MoveGroupInterface
  arm_->setEndEffectorLink(end_effector_link_);
  arm_->setPoseReferenceFrame(target_frame_);
  arm_->setPlanningTime(planning_time_);
  arm_->setNumPlanningAttempts(10);
  arm_->setMaxVelocityScalingFactor(velocity_scaling_);
  arm_->setMaxAccelerationScalingFactor(acceleration_scaling_);
  planning_frame_ = arm_->getPlanningFrame();
  gripper_client_.reset(new GripperClient(gripper_action_name_, true));// 初始化夹爪动作客户端

  state_publisher_ = nh_.advertise<std_msgs::String>("/sorting/state", 1, true);// 状态机
  detection_summary_publisher_ = nh_.advertise<std_msgs::String>("/sorting/detection_summary", 1, true);// 检测结果汇总 例如：红1,绿0,蓝1
  base_lock_publisher_ = nh_.advertise<std_msgs::Bool>(base_lock_topic_, 1, true);// 机械臂是否在执行任务 移动底盘是否需要被锁住
  failure_publisher_ = nh_.advertise<std_msgs::String>(failure_topic_, 1, true);// 任务失败原因
  target_cache_publisher_ = nh_.advertise<std_msgs::String>("/sorting/target_cache", 1, true);// 目标的信息目标颜色、平均位置、观测次数、置信度、离散程度、目标年龄以及是否已抓取
  grasp_attach_publisher_ = nh_.advertise<std_msgs::String>(grasp_attach_topic_, 1);// 将目标吸附到夹爪上
  grasp_detach_publisher_ = nh_.advertise<std_msgs::String>(grasp_detach_topic_, 1);// 从夹爪上分离目标

  detection_subscriber_ = nh_.subscribe(detections_topic_, 2, &ColorSortingTask::detectionCallback, this);
  grasp_status_subscriber_ = nh_.subscribe(grasp_status_topic_, 5,
                                           &ColorSortingTask::graspStatusCallback, this);// 抓取状态的反馈
  workspace_subscriber_ = nh_.subscribe(workspace_update_topic_, 1,
                                        &ColorSortingTask::workspaceUpdateCallback, this);// 动态接收新的工作区配置 例如换工作台、换桌子位置、换放置区域、换物体模型名称
  if (require_octomap_)
  {
    cloud_subscriber_ = nh_.subscribe(point_cloud_topic_, 1, &ColorSortingTask::cloudCallback, this);// 订阅点云话题
    planning_scene_subscriber_ = nh_.subscribe(planning_scene_topic_, 5,
                                               &ColorSortingTask::planningSceneCallback, this);// 订阅规划场景话题
    clear_octomap_client_ = nh_.serviceClient<std_srvs::Empty>(clear_octomap_service_);// 初始化清除Octomap服务客户端
  }

  services_.push_back(nh_.advertiseService("/sorting/move_to_observation",
                                            &ColorSortingTask::observeService, this));// 请求机械臂移动到观测位置
  services_.push_back(nh_.advertiseService("/sorting/start", &ColorSortingTask::startService, this));// 开始执行分拣任务请求
  services_.push_back(nh_.advertiseService("/sorting/stop", &ColorSortingTask::stopService, this));// 停止执行分拣任务请求
  services_.push_back(nh_.advertiseService("/sorting/open_gripper", &ColorSortingTask::openService, this));// 打开夹爪请求
  services_.push_back(nh_.advertiseService("/sorting/prepare_work",
                                            &ColorSortingTask::prepareWorkService, this));//让机械臂移动跑动模式
  services_.push_back(nh_.advertiseService("/sorting/home", &ColorSortingTask::homeService, this));// 请求机械臂移动到home
  services_.push_back(nh_.advertiseService("/sorting/configure_workspace",
                                            &ColorSortingTask::configureWorkspaceService, this));// 配置工作区请求

  std_msgs::Bool unlocked;
  unlocked.data = false;
  base_lock_publisher_.publish(unlocked);
  std_msgs::String empty_failure;
  failure_publisher_.publish(empty_failure);
  publishTargetCache();// 发布目前检测到的结果
  publishState("INITIALIZING", "waiting for Gazebo controllers");
}

ColorSortingTask::~ColorSortingTask()
{
  stop_requested_.store(true);
  if (arm_)
    arm_->stop();
  if (gripper_client_)
    gripper_client_->cancelAllGoals();
  if (initialization_thread_.joinable())
    initialization_thread_.join();
  if (operation_thread_.joinable())
    operation_thread_.join();
}

void ColorSortingTask::loadParameters()
{
  private_nh_.param<std::string>("planning_group", group_name_, "aubo_i5");
  private_nh_.param<std::string>("end_effector_link", end_effector_link_, "tcp_link");
  private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
  private_nh_.param<std::string>("detections_topic", detections_topic_, "/sorting/detections");
  private_nh_.param<std::string>("gripper_action", gripper_action_name_,
                                 "/gripper_controller/follow_joint_trajectory");
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
  private_nh_.param("place_clearance", place_clearance_, 0.02);
  private_nh_.param("cartesian_step", cartesian_step_, 0.01);
  private_nh_.param("minimum_cartesian_fraction", minimum_cartesian_fraction_, 0.90);
  private_nh_.param("gripper_open", gripper_open_, 0.0);
  private_nh_.param("gripper_closed", gripper_closed_, 0.28);
  private_nh_.param("gripper_motion_time", gripper_motion_time_, 2.5);
  private_nh_.param("gripper_contact_tolerance", gripper_contact_tolerance_, 0.30);
  private_nh_.param("use_grasp_attachment", use_grasp_attachment_, true);
  private_nh_.param<std::string>("grasp_attach_topic", grasp_attach_topic_, "/sorting/grasp/attach");
  private_nh_.param<std::string>("grasp_detach_topic", grasp_detach_topic_, "/sorting/grasp/detach");
  private_nh_.param<std::string>("grasp_status_topic", grasp_status_topic_, "/sorting/grasp/status");
  private_nh_.param("grasp_attachment_timeout", grasp_attachment_timeout_, 3.0);// 抓取吸附插件加载超时时间
  private_nh_.param<std::string>("place_frame", place_frame_, target_frame_);
  private_nh_.param("detection_timeout", detection_timeout_, 15.0);
  private_nh_.param("detection_samples", detection_samples_, 8);
  detection_samples_ = std::max(1, detection_samples_);
  private_nh_.param("detection_settle_time", detection_settle_time_, 1.0);
  private_nh_.param("verify_observation_detections", verify_observation_detections_, false);
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
  sort_colors_ = {"red", "green", "blue"};
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

void ColorSortingTask::start()
{
  initialization_thread_ = std::thread(&ColorSortingTask::initialize, this);
}

void ColorSortingTask::publishState(const std::string& state, const std::string& detail)
{
  // 负责更新状态机的状态为state并通过话题发布
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state_ = state;
  }
  std_msgs::String message;
  message.data = detail.empty() ? state : state + " | " + detail;
  state_publisher_.publish(message);
  ROS_INFO_STREAM("Sorting state: " << message.data);
}

void ColorSortingTask::setFailure(const std::string& category, const std::string& detail)
{
  std_msgs::String message;
  message.data = category + " | " + detail;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_failure_ = message.data;// 保存最近的失败信息  失败类型|失败详情
  }
  failure_publisher_.publish(message);
  ROS_ERROR_STREAM("Sorting failure: " << message.data);
}

void ColorSortingTask::detectionCallback(const aubo_perception::DetectedObjectArrayConstPtr& message)
{
  std::string state;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    detections_ = message;// 保存检测到的对象 头+检测到的对象数组
    detections_wall_time_ = ros::WallTime::now();
    state = state_;
  }
  if (state == "DETECTING" || (state == "OBSERVING" && observation_ready_.load()))
    updateTargetCache(*message);

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
  detection_summary_publisher_.publish(summary);
}

void ColorSortingTask::graspStatusCallback(const std_msgs::StringConstPtr& message)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  grasp_status_ = message->data;// 抓取吸附的状态
  ++grasp_status_sequence_;
}

void ColorSortingTask::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
{
  const std::size_t points = static_cast<std::size_t>(message->width) * message->height;
  if (!points)
    return;
  std::lock_guard<std::mutex> lock(data_mutex_);
  cloud_points_ = points;
  last_cloud_wall_time_ = ros::WallTime::now();
}

void ColorSortingTask::planningSceneCallback(const moveit_msgs::PlanningSceneConstPtr& message)
{
  if (message->world.octomap.octomap.data.empty())
    return;
  std::lock_guard<std::mutex> lock(data_mutex_);
  ++octomap_sequence_;
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
  std::map<std::string, const aubo_perception::DetectedObject*> selected;// 颜色+目标对象
  for (const auto& detected : message.objects) // 遍历检测到的对象数组 aubo_perception/DetectedObject[]
  {
    const auto found = selected.find(detected.color);
    if (found == selected.end() || detected.contour_area > found->second->contour_area)// 如果没有找到|检测到的面积大于已选面积
      selected[detected.color] = &detected;
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
      TargetTrack& track = found->second;// 找到了 已经完成拾取
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
  if (!target_cache_fallback_enabled_)
    return false;
  TargetTrack track;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto found = target_tracks_.find(color);
    if (found == target_tracks_.end())
      return false;
    track = found->second;
  }
  const double age = (ros::WallTime::now() - track.last_seen).toSec();
  if (track.picked || track.count < target_cache_min_observations_ || age > target_cache_max_age_)
    return false;
  double x = 0.0;
  double y = 0.0;
  if (!xyInTargetFrame(target_cache_frame_, {track.x, track.y}, x, y))
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

bool ColorSortingTask::workspaceFromParam(const XmlRpc::XmlRpcValue& value,
                                          WorkspaceConfig& workspace,
                                          std::string& error) const
{
  try
  {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      throw std::runtime_error("workspace configuration must be a mapping");
    if (!value.hasMember("id"))
      throw std::runtime_error("workspace 'id' is required");
    workspace.id = static_cast<std::string>(value["id"]);
    if (workspace.id.empty())
      throw std::runtime_error("workspace 'id' is required");
    workspace.table_center = xmlVector(value["table_center"], 3, "table_center");
    workspace.table_size = xmlVector(value["table_size"], 3, "table_size");
    workspace.table_frame = value.hasMember("table_frame") ?
        static_cast<std::string>(value["table_frame"]) : table_frame_;
    workspace.table_z = value.hasMember("table_z") ? xmlNumber(value["table_z"]) : table_z_;
    workspace.place_frame = value.hasMember("place_frame") ?
        static_cast<std::string>(value["place_frame"]) : place_frame_;
    workspace.place_targets = place_targets_;
    if (value.hasMember("place_targets"))
    {
      const XmlRpc::XmlRpcValue& targets = value["place_targets"];
      if (targets.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        throw std::runtime_error("workspace 'place_targets' must be a mapping");
      workspace.place_targets.clear();
      for (auto iterator = targets.begin(); iterator != targets.end(); ++iterator)
        workspace.place_targets[iterator->first] =
            xmlVector(iterator->second, 2, "place_targets." + iterator->first);
    }
    workspace.grasp_model_names = grasp_model_names_;
    if (value.hasMember("grasp_model_names"))
    {
      const XmlRpc::XmlRpcValue& names = value["grasp_model_names"];
      if (names.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        throw std::runtime_error("workspace 'grasp_model_names' must be a mapping");
      workspace.grasp_model_names.clear();
      for (auto iterator = names.begin(); iterator != names.end(); ++iterator)
        workspace.grasp_model_names[iterator->first] = static_cast<std::string>(iterator->second);
    }
    for (const std::string& color : sort_colors_)
      if (workspace.place_targets.count(color) == 0 || workspace.place_targets[color].size() != 2)
        throw std::runtime_error("workspace has no valid place target for '" + color + "'");
    return true;
  }
  catch (const std::exception& exception)
  {
    error = exception.what();
    return false;
  }
}

bool ColorSortingTask::workspaceFromJson(const std::string& json, WorkspaceConfig& workspace,
                                         std::string& error) const
{
  try
  {
    boost::property_tree::ptree root;
    std::istringstream stream(json);
    boost::property_tree::read_json(stream, root);
    workspace.id = root.get<std::string>("id");
    if (workspace.id.empty())
      throw std::runtime_error("workspace 'id' is required");
    auto read_vector = [&root](const std::string& key, std::size_t size) {
      std::vector<double> result;
      for (const auto& value : root.get_child(key))
        result.push_back(value.second.get_value<double>());
      if (result.size() != size)
        throw std::runtime_error("workspace '" + key + "' has the wrong number of values");
      return result;
    };
    workspace.table_center = read_vector("table_center", 3);
    workspace.table_size = read_vector("table_size", 3);
    workspace.table_frame = root.get<std::string>("table_frame", table_frame_);
    workspace.table_z = root.get<double>("table_z", table_z_);
    workspace.place_frame = root.get<std::string>("place_frame", place_frame_);
    workspace.place_targets = place_targets_;
    const auto targets = root.get_child_optional("place_targets");
    if (targets)
    {
      workspace.place_targets.clear();
      for (const auto& item : *targets)
      {
        std::vector<double> xy;
        for (const auto& value : item.second)
          xy.push_back(value.second.get_value<double>());
        if (xy.size() != 2)
          throw std::runtime_error("place target '" + item.first + "' must be [x, y]");
        workspace.place_targets[item.first] = xy;
      }
    }
    workspace.grasp_model_names = grasp_model_names_;
    const auto names = root.get_child_optional("grasp_model_names");
    if (names)
    {
      workspace.grasp_model_names.clear();
      for (const auto& item : *names)
        workspace.grasp_model_names[item.first] = item.second.get_value<std::string>();
    }
    for (const std::string& color : sort_colors_)
      if (workspace.place_targets.count(color) == 0 || workspace.place_targets[color].size() != 2)
        throw std::runtime_error("workspace has no valid place target for '" + color + "'");
    return true;
  }
  catch (const std::exception& exception)
  {
    error = exception.what();
    return false;
  }
}

void ColorSortingTask::workspaceUpdateCallback(const std_msgs::StringConstPtr& message)
{
  WorkspaceConfig workspace;
  std::string error;
  if (!workspaceFromJson(message->data, workspace, error))
  {
    ROS_ERROR_STREAM("Invalid workspace update: " << error);
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_workspace_ = workspace;
  has_pending_workspace_ = true;
}

bool ColorSortingTask::applyWorkspace(const WorkspaceConfig& workspace, std::string& error)
{
  table_center_ = workspace.table_center;
  table_size_ = workspace.table_size;
  table_frame_ = workspace.table_frame;
  table_z_ = workspace.table_z;
  place_frame_ = workspace.place_frame;
  place_targets_ = workspace.place_targets;
  grasp_model_names_ = workspace.grasp_model_names;
  workspace_id_ = workspace.id;
  completed_colors_.clear();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_tracks_.clear();
  }
  publishTargetCache();
  observation_ready_.store(false);
  if (!addTableCollision())
  {
    error = "MoveIt did not accept the workspace table";
    return false;
  }
  ROS_INFO_STREAM("Configured sorting workspace '" << workspace_id_ << "'");
  return true;
}

bool ColorSortingTask::configureWorkspaceService(std_srvs::Trigger::Request&,
                                                 std_srvs::Trigger::Response& response)
{
  std::lock_guard<std::mutex> operation_lock(operation_mutex_);
  if (busy_.load())
  {
    response.success = false;
    response.message = "sorting operation is running";
    return true;
  }
  WorkspaceConfig workspace;
  bool has_workspace = false;
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    if (has_pending_workspace_)
    {
      workspace = pending_workspace_;
      has_workspace = true;
    }
  }
  std::string error;
  XmlRpc::XmlRpcValue value;
  if (nh_.getParam(workspace_config_param_, value))
    has_workspace = workspaceFromParam(value, workspace, error);
  if (!has_workspace)
  {
    response.success = false;
    response.message = error.empty() ? "no workspace configuration received" : error;
    if (!error.empty())
      setFailure("CONFIGURATION_FAILED", error);
    return true;
  }
  response.success = applyWorkspace(workspace, error);
  response.message = response.success ? "workspace '" + workspace_id_ + "' configured" : error;
  if (!response.success)
    setFailure("CONFIGURATION_FAILED", error);
  return true;
}

void ColorSortingTask::initialize()
{
  if (!robot_limits_valid_) // 检证机械臂基座数第二个关节 是否在范围[-60, 60]内
  {
    busy_.store(false);
    publishState("ERROR", "loaded upperArm_joint limit is not +/-60 deg");
    return;
  }
  if (!waitForGraspPlugin()) // 等待抓取吸附插件加载完成
  {
    busy_.store(false);
    publishState("ERROR", "Gazebo grasp plugin unavailable");
    return;
  }
  publishState("INITIALIZING", "waiting for gripper action");
  if (!gripper_client_->waitForServer(ros::Duration(gripper_server_timeout_)))// 等待夹爪动作服务器启动
  {
    busy_.store(false);
    publishState("ERROR", "gripper action server unavailable");
    return;
  }
  if (!addTableCollision()) // 添加桌子到规划场景中
  {
    busy_.store(false);
    publishState("ERROR", "sorting table missing from planning scene");
    return;
  }
  if (!refreshOctomap()) // 刷新octomap
  {
    busy_.store(false);
    publishState("ERROR", "RGB-D cloud or MoveIt OctoMap unavailable");
    return;
  }
  initialized_.store(true);// 完成初始化
  busy_.store(false);
  publishState("IDLE", "controllers ready");
  if (auto_move_to_observation_)
    startOperation("OBSERVING", std::bind(&ColorSortingTask::initialObservationOperation, this));
  else if (auto_start_)
    startOperation("SORTING", std::bind(&ColorSortingTask::sortingOperation, this));
}

std::pair<bool, std::string> ColorSortingTask::startOperation(
    const std::string& state, const std::function<bool()>& operation)
{
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (busy_.load())
      return std::make_pair(false, "another operation is running");
    if (!initialized_.load())
      return std::make_pair(false, "sorting node is not initialized");
    busy_.store(true);
    stop_requested_.store(false);
    {
      std::lock_guard<std::mutex> data_lock(data_mutex_);
      last_failure_.clear();
    }
    std_msgs::String empty;
    failure_publisher_.publish(empty);
    std_msgs::Bool locked;
    locked.data = true;
    base_lock_publisher_.publish(locked);
  }
  // 如果操作线程已经存在并且可连接，则等待其完成
  if (operation_thread_.joinable())
    operation_thread_.join();
  // 启动一个新的线程来执行操作 捕获状态+操作函数
  operation_thread_ = std::thread([this, state, operation]() {
    bool success = false;
    publishState(state);
    try
    {
      success = operation();
    }
    catch (const std::exception& exception)
    {
      ROS_ERROR_STREAM("Sorting operation failed: " << exception.what());
      publishState("ERROR", exception.what());
    }
    if (!success)
      releaseAttachedObjectNoWait();// 如果操作失败，释放吸附的对象
    {
      std::lock_guard<std::mutex> lock(operation_mutex_);
      busy_.store(false);
    }
    std_msgs::Bool unlocked;
    unlocked.data = false;
    base_lock_publisher_.publish(unlocked);
    if (stop_requested_.load())
    {
      observation_ready_.store(false);
      publishState("STOPPED", "operation cancelled");
    }
    else if (success)
      publishState("READY", "waiting for panel command");
    else
      publishState("ERROR", "operation failed");
  });
  return std::make_pair(true, "command accepted");
}

bool ColorSortingTask::observeService(std_srvs::Trigger::Request&,
                                      std_srvs::Trigger::Response& response)
{
  const auto result = startOperation("OBSERVING",
      std::bind(&ColorSortingTask::observationOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::startService(std_srvs::Trigger::Request&,
                                    std_srvs::Trigger::Response& response)
{
  if (!observation_ready_.load())
  {
    response.success = false;
    response.message = "move to observation pose and confirm detections first";
    return true;
  }
  const auto result = startOperation("SORTING", std::bind(&ColorSortingTask::sortingOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::stopService(std_srvs::Trigger::Request&,
                                   std_srvs::Trigger::Response& response)
{
  stop_requested_.store(true);
  gripper_client_->cancelAllGoals();
  arm_->stop();
  releaseAttachedObjectNoWait();
  response.success = true;
  response.message = "stop requested";
  return true;
}

bool ColorSortingTask::openService(std_srvs::Trigger::Request&,
                                   std_srvs::Trigger::Response& response)
{
  const auto result = startOperation("OPENING", std::bind(&ColorSortingTask::openOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::prepareWorkService(std_srvs::Trigger::Request&,
                                          std_srvs::Trigger::Response& response)
{
  const auto result = startOperation("PREPARING",
      std::bind(&ColorSortingTask::prepareWorkOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::homeService(std_srvs::Trigger::Request&,
                                   std_srvs::Trigger::Response& response)
{
  const auto result = startOperation("HOMING", std::bind(&ColorSortingTask::homeOperation, this));
  response.success = result.first;
  response.message = result.second;
  return true;
}

bool ColorSortingTask::observationOperation()
{
  if (!observation())
    return false;
  if (verify_observation_detections_ && !verifyVisibleColors())
  {
    observation_ready_.store(false);
    return false;
  }
  return true;
}

bool ColorSortingTask::initialObservationOperation()
{
  if (!observationOperation())
    return false;
  return !auto_start_ || sortingOperation();
}

bool ColorSortingTask::openOperation()
{
  if (!commandGripper(gripper_open_))
    return false;
  std::string model;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    model = attached_model_;
  }
  return model.empty() || setGraspAttachment(model, false);
}

bool ColorSortingTask::homeOperation()
{
  observation_ready_.store(false);
  return moveNamed(finish_named_target_);
}

bool ColorSortingTask::prepareWorkOperation()
{
  observation_ready_.store(false);
  return refreshOctomap() && addTableCollision() && moveNamed(work_ready_named_target_);
}

bool ColorSortingTask::observation()
{
  observation_ready_.store(false);
  if (!refreshOctomap() || !addTableCollision())  // 需要先更新octomap和场景碰撞模型
    return false;
  const bool success = observation_named_target_.empty() ?
      moveToPose(makePose(observation_pose_[0], observation_pose_[1], observation_pose_[2]),
                 "camera observation pose") :
      moveNamed(observation_named_target_);
  if (!success || !wallSleep(detection_settle_time_, stop_requested_))
    return false;
  observation_ready_.store(true);
  return true;
}

geometry_msgs::PoseStamped ColorSortingTask::makePose(double x, double y, double z) const
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = target_frame_;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  tf::Quaternion quaternion;
  quaternion.setRPY(grasp_rpy_[0], grasp_rpy_[1], grasp_rpy_[2]);
  tf::quaternionTFToMsg(quaternion, pose.pose.orientation);
  return pose;
}

bool ColorSortingTask::xyInTargetFrame(const std::string& source_frame,
                                       const std::vector<double>& xy, double& x, double& y)
{
  if (source_frame == target_frame_)
  {
    x = xy[0];
    y = xy[1];
    return true;
  }
  geometry_msgs::PoseStamped source;
  source.header.frame_id = source_frame;
  source.header.stamp = ros::Time(0);
  source.pose.position.x = xy[0];
  source.pose.position.y = xy[1];
  source.pose.orientation.w = 1.0;
  try
  {
    tf_listener_.waitForTransform(target_frame_, source_frame, ros::Time(0),
                                  ros::Duration(scene_update_timeout_));
    geometry_msgs::PoseStamped target;
    tf_listener_.transformPose(target_frame_, source, target);
    x = target.pose.position.x;
    y = target.pose.position.y;
    return true;
  }
  catch (const tf::TransformException& error)
  {
    ROS_ERROR("Cannot transform place target from %s to %s: %s", source_frame.c_str(),
              target_frame_.c_str(), error.what());
    return false;
  }
}

bool ColorSortingTask::moveToPose(const geometry_msgs::PoseStamped& pose,
                                  const std::string& description)
{
  if (stop_requested_.load())
    return false;
  ROS_INFO_STREAM("Planning arm to " << description);
  arm_->setPoseTarget(pose, end_effector_link_);// 设置目标姿态(末端执行器的笛卡尔空间位姿)
  return planAndExecute(description) && !stop_requested_.load();
}

bool ColorSortingTask::planAndExecute(const std::string& description)
{
  arm_->setStartStateToCurrentState();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planned = arm_->plan(plan);
  if (planned != moveit::planning_interface::MoveItErrorCode::SUCCESS ||
      plan.trajectory_.joint_trajectory.points.empty())
  {
    arm_->stop();
    arm_->clearPoseTargets();
    setFailure("PLANNING_FAILED", description);
    return false;
  }
  const auto executed = arm_->execute(plan);
  arm_->stop();
  arm_->clearPoseTargets();
  const bool success = executed == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  if (!success)
    setFailure("EXECUTION_FAILED", description);
  return success;
}

bool ColorSortingTask::moveNamed(const std::string& target)
{
  // 先判断这个目标是否已经在命名目标中
  // 判断当前关节值是否已经满足阈值
  // 设置目标为当前命名位置 
  if (target.empty())
    return true;
  if (stop_requested_.load())
    return false;
  const std::vector<std::string> names = arm_->getNamedTargets();// 获取所有命名目标
  if (std::find(names.begin(), names.end(), target) == names.end()) // 验证target是否在命名目标中
  {
    setFailure("CONFIGURATION_FAILED", "unknown named target '" + target + "'");
    return false;
  }
  const std::map<std::string, double> target_values = arm_->getNamedTargetValues(target);
  const std::vector<std::string> active_joints = arm_->getActiveJoints();
  const std::vector<double> current = arm_->getCurrentJointValues();
  std::map<std::string, double> current_values;// 当前关节值映射
  for (std::size_t index = 0; index < active_joints.size() && index < current.size(); ++index)
    current_values[active_joints[index]] = current[index];
  bool already_there = !target_values.empty();
  for (const auto& value : target_values)
  {
    const auto found = current_values.find(value.first);// 在当前关节值中查找目标关节
    if (found == current_values.end() || std::abs(found->second - value.second) > 1.0e-3) // 判断误差
    {
      already_there = false;
      break;
    }
  }
  if (already_there)
  {
    ROS_INFO_STREAM("Arm is already at named target " << target);
    return true;
  }
  ROS_INFO_STREAM("Moving arm to named target " << target);
  arm_->setNamedTarget(target);
  return planAndExecute("named target '" + target + "'") && !stop_requested_.load();
}

bool ColorSortingTask::cartesianTo(const geometry_msgs::PoseStamped& target_pose,
                                   const std::string& description)
{
  if (stop_requested_.load())
    return false;
  std::vector<geometry_msgs::Pose> waypoints(1, target_pose.pose);
  moveit_msgs::RobotTrajectory trajectory;
  auto compute_path = [this, &waypoints, &trajectory]() {
    arm_->setStartStateToCurrentState();
    return arm_->computeCartesianPath(waypoints, cartesian_step_, 0.0, trajectory, true);
  };
  double fraction = compute_path();
  ROS_INFO("Cartesian path to %s: %.1f%%", description.c_str(), 100.0 * fraction);
  if (fraction < minimum_cartesian_fraction_ && require_octomap_)
  {
    ROS_WARN("Cartesian fraction too low; refreshing OctoMap and retrying once");
    if (refreshOctomap() && !stop_requested_.load())
    {
      fraction = compute_path();
      ROS_INFO("Cartesian path retry to %s: %.1f%%", description.c_str(), 100.0 * fraction);
    }
  }
  if (fraction < minimum_cartesian_fraction_)
  {
    ROS_WARN("Cartesian fraction too low; falling back to pose planning");
    return moveToPose(target_pose, description);
  }

  robot_trajectory::RobotTrajectory robot_trajectory(arm_->getRobotModel(), group_name_);
  robot_trajectory.setRobotTrajectoryMsg(*arm_->getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization parameterization;
  if (!parameterization.computeTimeStamps(robot_trajectory, velocity_scaling_, acceleration_scaling_))
  {
    setFailure("PLANNING_FAILED", "cannot retime Cartesian path to " + description);
    return false;
  }
  robot_trajectory.getRobotTrajectoryMsg(trajectory);
  if (trajectory.joint_trajectory.points.empty())
  {
    setFailure("PLANNING_FAILED", "empty Cartesian path to " + description);
    return false;
  }
  ROS_INFO("Retimed Cartesian path to %s: %.2f s at velocity scale %.2f",
           description.c_str(), trajectory.joint_trajectory.points.back().time_from_start.toSec(),
           velocity_scaling_);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  const bool success = arm_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  arm_->stop();
  if (!success)
    setFailure("EXECUTION_FAILED", "Cartesian path to " + description);
  return success && !stop_requested_.load();
}

bool ColorSortingTask::commandGripper(double position)
{
  if (stop_requested_.load())
    return false;
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {"joint1", "joint2"};
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = {position, position};
  point.time_from_start = ros::Duration(gripper_motion_time_);
  goal.trajectory.points.push_back(point);
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
  for (const std::string& joint_name : goal.trajectory.joint_names)
  {
    control_msgs::JointTolerance path;
    path.name = joint_name;
    path.position = 0.10;
    goal.path_tolerance.push_back(path);
    control_msgs::JointTolerance target;
    target.name = joint_name;
    target.position = 0.03;
    goal.goal_tolerance.push_back(target);
  }
  goal.goal_time_tolerance = ros::Duration(3.0);
  gripper_client_->sendGoal(goal);
  if (!gripper_client_->waitForResult(ros::Duration(gripper_motion_time_ + 3.0)))
  {
    gripper_client_->cancelGoal();
    ROS_ERROR("Gripper command timed out");
    return false;
  }
  const actionlib::SimpleClientGoalState state = gripper_client_->getState();
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    const auto result = gripper_client_->getResult();
    ROS_ERROR("Gripper action failed: state=%s, error_code=%d, error_string='%s'",
              state.toString().c_str(), result ? result->error_code : std::numeric_limits<int>::min(),
              result ? result->error_string.c_str() : "");
    return false;
  }
  return !stop_requested_.load();
}

bool ColorSortingTask::addTableCollision()
{
  const std::string object_name = "sorting_table";
  geometry_msgs::PoseStamped table_pose;
  table_pose.header.frame_id = table_frame_;
  table_pose.header.stamp = ros::Time(0);
  table_pose.pose.orientation.w = 1.0;
  table_pose.pose.position.x = table_center_[0];
  table_pose.pose.position.y = table_center_[1];
  table_pose.pose.position.z = table_center_[2];
  try
  {
    if (table_frame_ != planning_frame_)
    {
      tf_listener_.waitForTransform(planning_frame_, table_frame_, ros::Time(0),
                                    ros::Duration(scene_update_timeout_));// 等待从table_frame_ map 到planning_frame_ base_link的变换
      geometry_msgs::PoseStamped transformed;
      tf_listener_.transformPose(planning_frame_, table_pose, transformed);
      table_pose = transformed;
    }
  }
  catch (const tf::TransformException& error)
  {
    ROS_ERROR("Cannot transform sorting table from %s to %s: %s", table_frame_.c_str(),
              planning_frame_.c_str(), error.what());
    return false;
  }

  auto known = [this, &object_name]() {// 检测当前场景中是否存在指定对象
    const auto names = scene_.getKnownObjectNames();
    return std::find(names.begin(), names.end(), object_name) != names.end();
  };
  if (known())
  {
    scene_.removeCollisionObjects({object_name});
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(scene_update_timeout_);
    ros::WallRate rate(20.0);
    while (ros::ok() && known() && ros::WallTime::now() < deadline)
      rate.sleep();
    if (known())
    {
      ROS_ERROR("MoveIt did not remove stale sorting table");
      return false;
    }
  }

  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = table_pose.header.frame_id;
  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[shape_msgs::SolidPrimitive::BOX_X] = table_size_[0] + 2.0 * table_collision_margin_;
  box.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = table_size_[1] + 2.0 * table_collision_margin_;
  box.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = table_size_[2];
  object.primitives.push_back(box);
  object.primitive_poses.push_back(table_pose.pose);
  object.operation = moveit_msgs::CollisionObject::ADD;
  scene_.applyCollisionObject(object);// 应用碰撞对象到场景中

  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(scene_update_timeout_);
  ros::WallRate rate(10.0);
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    if (known())
    {
      arm_->setSupportSurfaceName(object_name);  // 告诉MoveIt规划器，这个障碍物（桌子）是支撑面
      ROS_INFO("Sorting table confirmed in MoveIt planning scene: frame=%s, position=[%.3f, %.3f, %.3f], size=[%.3f, %.3f, %.3f]",
               table_pose.header.frame_id.c_str(), table_pose.pose.position.x,
               table_pose.pose.position.y, table_pose.pose.position.z, box.dimensions[0],
               box.dimensions[1], box.dimensions[2]);
      return true;
    }
    rate.sleep();
  }
  ROS_ERROR("MoveIt planning scene did not acknowledge '%s' within %.1f seconds",
            object_name.c_str(), scene_update_timeout_);
  return false;
}

bool ColorSortingTask::refreshOctomap()
{
  if (!require_octomap_)// 是否要求八叉树地图
    return true;
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(octomap_wait_timeout_);
  const ros::WallTime cloud_not_before = ros::WallTime::now();
  std::size_t cloud_points = 0;
  ros::WallRate rate(20.0);
  bool cloud_ready = false;
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      cloud_ready = last_cloud_wall_time_ > cloud_not_before && cloud_points_ > 0;// 确保是新鲜的数据 且点云数据非空 
      cloud_points = cloud_points_;
    }
    if (cloud_ready)
      break;
    if (stop_requested_.load())
      return false;
    rate.sleep();
  }
  if (!cloud_ready)
  {
    ROS_ERROR_STREAM("No fresh RGB-D cloud received on " << point_cloud_topic_);
    return false;
  }

  const double remaining = std::max(0.1, (deadline - ros::WallTime::now()).toSec());
  if (!clear_octomap_client_.waitForExistence(ros::Duration(remaining)))
  {
    ROS_ERROR_STREAM("Cannot clear MoveIt OctoMap: service unavailable " << clear_octomap_service_);
    return false;
  }
  std::uint64_t previous_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    previous_sequence = octomap_sequence_;
  }
  std_srvs::Empty service;
  if (!clear_octomap_client_.call(service))// 渰除八叉树地图
  {
    ROS_ERROR("Cannot clear MoveIt OctoMap");
    return false;
  }
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (octomap_sequence_ > previous_sequence)
      {
        ROS_INFO("Fresh MoveIt OctoMap confirmed from %s (%zu points/cloud)",
                 point_cloud_topic_.c_str(), cloud_points);
        return true;
      }
    }
    if (stop_requested_.load())
      return false;
    rate.sleep();
  }
  ROS_ERROR("MoveIt did not publish a fresh non-empty OctoMap");
  return false;
}

bool ColorSortingTask::waitForGraspPlugin()
{
  if (!use_grasp_attachment_) // 是否使用抓取吸附插件
    return true;
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(grasp_attachment_timeout_);
  ros::WallRate rate(20.0);
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    std::string status;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      status = grasp_status_;
    }
    if (status == "ready" || status.compare(0, 9, "detached:") == 0)
    {
      ROS_INFO("Gazebo grasp attachment plugin is ready");
      return true;
    }
    if (status.compare(0, 6, "error:") == 0)
    {
      ROS_ERROR_STREAM("Gazebo grasp plugin reported: " << status);
      return false;
    }
    rate.sleep();
  }
  ROS_ERROR_STREAM("No status received from Gazebo grasp plugin on " << grasp_status_topic_);
  return false;
}

bool ColorSortingTask::setGraspAttachment(const std::string& model_name, bool attach)
{
  if (!use_grasp_attachment_)
    return true;
  const std::string expected = (attach ? "attached:" : "detached:") + model_name;
  std::uint64_t initial_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    initial_sequence = grasp_status_sequence_;
  }
  std_msgs::String command;
  command.data = model_name;
  (attach ? grasp_attach_publisher_ : grasp_detach_publisher_).publish(command);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(grasp_attachment_timeout_);
  ros::WallRate rate(50.0);
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    std::string status;
    std::uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      status = grasp_status_;
      sequence = grasp_status_sequence_;
    }
    if (sequence > initial_sequence && status == expected)
    {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        attached_model_ = attach ? model_name : std::string();
      }
      ROS_INFO_STREAM("Gazebo grasp status: " << status);
      return true;
    }
    if (sequence > initial_sequence && status.compare(0, 6, "error:") == 0)
    {
      ROS_ERROR_STREAM("Gazebo grasp plugin reported: " << status);
      return false;
    }
    rate.sleep();
  }
  ROS_ERROR_STREAM("Timed out waiting for Gazebo grasp status '" << expected << "'");
  return false;
}

void ColorSortingTask::releaseAttachedObjectNoWait()
{
  std::string model;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    model.swap(attached_model_);
  }
  if (use_grasp_attachment_ && !model.empty())
  {
    std_msgs::String command;
    command.data = model;
    grasp_detach_publisher_.publish(command);
  }
}

bool ColorSortingTask::waitForObject(const std::string& color, const ros::WallTime& not_before,
                                     aubo_perception::DetectedObject& detected)
{
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(detection_timeout_);
  const ros::WallTime cache_deadline =
      ros::WallTime::now() + ros::WallDuration(target_cache_fallback_delay_);
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
    if (detections && receipt > last_receipt)
    {
      last_receipt = receipt;
      const aubo_perception::DetectedObject* largest = nullptr;
      for (const auto& candidate : detections->objects)
        if (candidate.color == color && (!largest || candidate.contour_area > largest->contour_area))
          largest = &candidate;
      if (largest)
      {
        samples.push_back(*largest);
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
          detected.pose.position.x /= samples.size();
          detected.pose.position.y /= samples.size();
          detected.pose.position.z /= samples.size();
          ROS_INFO("Averaged %zu '%s' detections at [%.3f, %.3f]", samples.size(),
                   color.c_str(), detected.pose.position.x, detected.pose.position.y);
          return true;
        }
      }
    }
    if (ros::WallTime::now() >= cache_deadline && cachedObject(color, detected))
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
  std::set<std::string> required;
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
        visible.insert(item.color);
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

bool ColorSortingTask::pickAndPlace(const aubo_perception::DetectedObject& detected)
{
  const std::string color = detected.color;
  const double object_x = detected.pose.position.x + grasp_offset_x_;
  const double object_y = detected.pose.position.y + grasp_offset_y_;
  const double grasp_z = table_z_ + 0.5 * object_height_ + grasp_height_offset_;
  const double travel_z = table_z_ + lift_height_;
  ROS_INFO("Picking %s at [%.3f, %.3f, %.3f]", color.c_str(), object_x, object_y, grasp_z);

  if (!commandGripper(gripper_open_) ||
      !moveToPose(makePose(object_x, object_y, table_z_ + pregrasp_height_), color + " pre-grasp") ||
      !cartesianTo(makePose(object_x, object_y, grasp_z), color + " grasp"))
    return false;
  const auto model = grasp_model_names_.find(color);
  const std::string object_model_name =
      model == grasp_model_names_.end() ? color + "_block" : model->second;
  if (!setGraspAttachment(object_model_name, true))
    return false;
  if (!commandGripper(gripper_closed_))
  {
    setGraspAttachment(object_model_name, false);
    return false;
  }
  if (!wallSleep(0.5, stop_requested_) ||
      !cartesianTo(makePose(object_x, object_y, travel_z), color + " lift"))
    return false;

  const auto place = place_targets_.find(color);
  if (place == place_targets_.end())
  {
    ROS_ERROR_STREAM("No place target configured for color '" << color << "'");
    return false;
  }
  double place_x = 0.0;
  double place_y = 0.0;
  if (!xyInTargetFrame(place_frame_, place->second, place_x, place_y) ||
      !moveToPose(makePose(place_x, place_y, travel_z), color + " pre-place") ||
      !cartesianTo(makePose(place_x, place_y, grasp_z + place_clearance_), color + " place") ||
      !commandGripper(gripper_open_) || !setGraspAttachment(object_model_name, false) ||
      !wallSleep(0.5, stop_requested_))
    return false;
  return cartesianTo(makePose(place_x, place_y, travel_z), color + " retreat");
}

bool ColorSortingTask::sortingOperation()
{
  bool all_complete = !sort_colors_.empty();// 分拣的颜色列表不为空
  for (const auto& color : sort_colors_)
    all_complete = all_complete && completed_colors_.count(color) != 0;
  if (all_complete)
  {
    completed_colors_.clear();
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
    publishState("DETECTING", color);
    aubo_perception::DetectedObject detected;
    if (!waitForObject(color, ros::WallTime::now(), detected))
      return false;
    observation_ready_.store(false);
    publishState("PICKING", color);
    if (!pickAndPlace(detected))
      return false;
    completed_colors_.insert(color);
    markTargetPicked(color);
    if (index + 1 < sort_colors_.size())
    {
      publishState("OBSERVING", "next object");
      if (!observation())
        return false;
    }
  }
  observation_ready_.store(false);
  if (!finish_named_target_.empty())
  {
    publishState("HOMING", finish_named_target_);
    return moveNamed(finish_named_target_);
  }
  return true;
}

}  // namespace aubo_sorting_core
