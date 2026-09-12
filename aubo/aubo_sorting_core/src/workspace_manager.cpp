#include <aubo_sorting_core/color_sorting_task.hpp>
#include "task_utils.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace aubo_sorting_core
{
using detail::xmlNumber;
using detail::xmlVector;

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
  pending_workspace_ = workspace;// 将接收到的workspace配置存储到pending_workspace_中
  has_pending_workspace_ = true;
}

bool ColorSortingTask::applyWorkspace(const WorkspaceConfig& workspace, std::string& error)
{
  if (workspace.table_center.size() != 3 || workspace.table_size.size() != 3 ||
      !std::isfinite(workspace.table_z) ||
      !std::all_of(workspace.table_center.begin(), workspace.table_center.end(),
                   [](double v) { return std::isfinite(v); }) ||
      !std::all_of(workspace.table_size.begin(), workspace.table_size.end(),
                   [](double v) { return std::isfinite(v) && v > 0.0; }))
  {
    error = "workspace table must have finite coordinates and positive dimensions";
    return false;
  }
  const auto old_center = table_center_;
  const auto old_size = table_size_;
  const auto old_frame = table_frame_;
  table_center_ = workspace.table_center;
  table_size_ = workspace.table_size;
  table_frame_ = workspace.table_frame;
  if (!addTableCollision())
  {
    table_center_ = old_center;
    table_size_ = old_size;
    table_frame_ = old_frame;
    error = "MoveIt did not accept the workspace table";
    return false;
  }
  table_z_ = workspace.table_z;
  place_frame_ = workspace.place_frame;
  place_targets_ = workspace.place_targets;
  grasp_model_names_ = workspace.grasp_model_names;
  workspace_id_ = workspace.id;
  completed_colors_.clear();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_tracks_.clear();
    detections_.reset();
  }
  publishTargetCache();
  observation_ready_.store(false);
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
  std::string error;
  XmlRpc::XmlRpcValue value;
  if (nh_.getParam(workspace_config_param_, value))
    has_workspace = workspaceFromParam(value, workspace, error);// 尝试从参数服务器加载workspace配置
  else
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    if (has_pending_workspace_)
    {
      workspace = pending_workspace_;
      has_workspace = true;
    }
  }
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
  if (!scene_.applyCollisionObject(object))
  {
    ROS_ERROR("MoveIt rejected workspace table replacement");
    return false;
  }
  arm_->setSupportSurfaceName(object_name);
  return true;
}

bool ColorSortingTask::refreshOctomap()
{
  // 更新八叉树地图
  // 首先等待点云数据 等待点云回调数据中的时间戳大于当前时间戳 且点云数据非空 说明有新的点云支持我们重新构建八叉树
  // 然后调用清除八叉树地图的服务 清除八叉树地图
  // 等待planning_sence的回调 看看八叉树的序列更新了吗
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
  if (!cloud_ready)// 没有收到新鲜的点云数据
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
    previous_sequence = octomap_sequence_;// 记录当前八叉树序列号
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
      if (octomap_sequence_ > previous_sequence)  // 八叉树序列序列号大于之前的序列号 说明有新的八叉树地图
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

}  // namespace aubo_sorting_core
