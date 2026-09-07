#include <aubo_mobile_nav_sorting/mission_context.h>
#include <cmath>
#include <set>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace aubo_mobile_nav_sorting
{
namespace
{
std::vector<double> vectorParam(const ros::NodeHandle &node, const std::string &name,
                                const std::vector<double> &fallback)
{
  std::vector<double> result;
  if (!node.getParam(name, result))
    result = fallback;
  if (result.empty())
    throw std::runtime_error(name + " must be a non-empty list");
  return result;
}

std::vector<std::vector<double>> matrixParam(const ros::NodeHandle &node, const std::string &name,
                                             const std::vector<std::vector<double>> &fallback)
{
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value))
    return fallback;
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::runtime_error(name + " must be a list");
  std::vector<std::vector<double>> result;
  for (int row = 0; row < value.size(); ++row)
  {
    if (value[row].getType() != XmlRpc::XmlRpcValue::TypeArray)
      throw std::runtime_error(name + " rows must be lists");
    std::vector<double> output_row;
    for (int column = 0; column < value[row].size(); ++column)
    {
      const auto type = value[row][column].getType();
      if (type == XmlRpc::XmlRpcValue::TypeInt)
        output_row.push_back(static_cast<int>(value[row][column]));
      else if (type == XmlRpc::XmlRpcValue::TypeDouble)
        output_row.push_back(static_cast<double>(value[row][column]));
      else
        throw std::runtime_error(name + " values must be numeric");
    }
    result.push_back(output_row);
  }
  return result;
}

std::string jsonEscape(const std::string &input)
{
  std::ostringstream output;
  for (const char character : input)
  {
    switch (character)
    {
    case '\\':
      output << "\\\\";
      break;
    case '"':
      output << "\\\"";
      break;
    case '\n':
      output << "\\n";
      break;
    case '\r':
      output << "\\r";
      break;
    case '\t':
      output << "\\t";
      break;
    default:
      output << character;
      break;
    }
  }
  return output.str();
}
} // namespace
MissionContext::MissionContext(const ros::NodeHandle &node_handle,
                               const ros::NodeHandle &private_node_handle)
    : node_handle_(node_handle), private_node_handle_(private_node_handle)
{
  loadParameters();
  validateWorkstations();
}

void MissionContext::loadParameters()
{
  private_node_handle_.param("navigation_action", navigation_action_, std::string("/move_base"));
  private_node_handle_.param("navigation_frame", navigation_frame_, std::string("map"));
  private_node_handle_.param("base_frame", base_frame_, std::string("base_footprint"));

  const auto goal = vectorParam(private_node_handle_, "sorting_goal", {2.15, 0.0, 0.0});
  if (goal.size() != 3)
    throw std::runtime_error("sorting_goal must be [x, y, yaw]");
  sorting_goal_ = {goal[0], goal[1], goal[2]};
  const auto pre_dock = vectorParam(private_node_handle_, "pre_dock_goal", {1.85, 0.0, 0.0});
  if (pre_dock.size() != 3)
    throw std::runtime_error("pre_dock_goal must be [x, y, yaw]");
  pre_dock_goal_ = {pre_dock[0], pre_dock[1], pre_dock[2]};

  private_node_handle_.param("near_field_enabled", near_field_enabled_, false);
  candidate_x_ = vectorParam(private_node_handle_, "near_field_candidate_x", {2.16, 2.10, 2.06});
  candidate_y_ = vectorParam(private_node_handle_, "near_field_candidate_y", {0.0, -0.08, 0.08});
  candidate_yaw_ =
      vectorParam(private_node_handle_, "near_field_candidate_yaw", {0.0, -0.08, 0.08});
  private_node_handle_.param("near_field_max_candidates", near_field_max_candidates_, 6);
  table_geometry_ = vectorParam(private_node_handle_, "near_field_table", {3.0, 0.0, 0.80, 1.20});
  detector_workspace_ =
      vectorParam(private_node_handle_, "near_field_detector_workspace", {0.40, 0.82, -0.22, 0.22});
  camera_target_ = vectorParam(private_node_handle_, "near_field_camera_target", {0.62, 0.0});
  workpiece_points_ = matrixParam(private_node_handle_, "near_field_workpieces",
                                  {{2.78, -0.12}, {2.86, 0.0}, {2.78, 0.12}});
  if (table_geometry_.size() != 4 || detector_workspace_.size() != 4 || camera_target_.size() != 2)
    throw std::runtime_error("invalid near-field geometry dimensions");
  for (const auto &point : workpiece_points_)
    if (point.size() != 2)
      throw std::runtime_error("near_field_workpieces entries must be [x, y]");

  private_node_handle_.param("near_field_base_clearance", base_clearance_, 0.40);
  private_node_handle_.param("near_field_direct_dock_enabled", direct_dock_enabled_, true);
  private_node_handle_.param("near_field_direct_dock_max_distance", direct_dock_max_distance_,
                             0.50);
  private_node_handle_.param("near_field_direct_dock_lateral_tolerance",
                             direct_dock_lateral_tolerance_, 0.04);
  private_node_handle_.param("near_field_direct_dock_yaw_tolerance", direct_dock_yaw_tolerance_,
                             0.04);
  private_node_handle_.param("near_field_direct_dock_goal_tolerance", direct_dock_goal_tolerance_,
                             0.06);
  private_node_handle_.param("near_field_direct_dock_timeout", direct_dock_timeout_, 15.0);
  private_node_handle_.param("near_field_direct_dock_stall_timeout", direct_dock_stall_timeout_,
                             2.5);
  private_node_handle_.param("near_field_direct_dock_progress_epsilon",
                             direct_dock_progress_epsilon_, 0.005);
  private_node_handle_.param("near_field_heading_alignment_enabled", heading_alignment_enabled_,
                             true);
  private_node_handle_.param("near_field_heading_max_correction", heading_max_correction_, 0.12);
  private_node_handle_.param("near_field_heading_speed", heading_speed_, 0.12);
  private_node_handle_.param("near_field_heading_goal_tolerance", heading_goal_tolerance_, 0.015);
  private_node_handle_.param("near_field_heading_final_tolerance", heading_final_tolerance_, 0.025);
  private_node_handle_.param("near_field_heading_timeout", heading_timeout_, 4.0);
  private_node_handle_.param("near_field_heading_stall_timeout", heading_stall_timeout_, 1.5);

  private_node_handle_.param("server_timeout", server_timeout_, 45.0);
  private_node_handle_.param("navigation_timeout", navigation_timeout_, 180.0);
  private_node_handle_.param("navigation_retries", navigation_retries_, 1);
  private_node_handle_.param("sorting_initialization_timeout", initialization_timeout_, 60.0);
  private_node_handle_.param("sorting_operation_timeout", operation_timeout_, 300.0);
  private_node_handle_.param("sorting_stop_timeout", stop_timeout_, 5.0);
  private_node_handle_.param("base_pose_max_age", tf_max_age_, 0.5);
  if (!std::isfinite(stop_timeout_) || stop_timeout_ <= 0.0 || !std::isfinite(tf_max_age_) ||
      tf_max_age_ <= 0.0)
    throw std::runtime_error(
        "sorting_stop_timeout and base_pose_max_age must be finite and positive");
  private_node_handle_.param("startup_delay", startup_delay_, 3.0);
  private_node_handle_.param("home_before_navigation", home_before_navigation_, true);
  private_node_handle_.param("auto_start", auto_start_, false);
  private_node_handle_.param("return_to_start", return_to_start_, true);
  private_node_handle_.param("return_frame", return_frame_, navigation_frame_);
  if (return_frame_.empty())
    throw std::runtime_error("return_frame must not be empty");
  if (!private_node_handle_.getParam("workstations", workstations_))
    workstations_.setSize(0);

  private_node_handle_.param("base_recovery_enabled", base_recovery_enabled_, true);
  private_node_handle_.param("base_recovery_cmd_vel_topic", velocity_topic_,
                             std::string("/cmd_vel_raw"));
  private_node_handle_.param("base_recovery_speed", base_recovery_speed_, 0.04);
  private_node_handle_.param("base_recovery_rate", base_recovery_rate_, 20.0);
  private_node_handle_.param("base_recovery_settle_time", base_recovery_settle_time_, 0.8);
  recovery_steps_ =
      matrixParam(private_node_handle_, "base_recovery_steps",
                  {{0.0, 0.06}, {0.0, -0.12}, {0.0, 0.06}, {0.05, 0.0}, {-0.10, 0.0}});
  for (const auto &step : recovery_steps_)
    if (step.size() != 2 || (std::abs(step[0]) > 1e-6 && std::abs(step[1]) > 1e-6))
      throw std::runtime_error("base_recovery_steps must contain single-axis [dx, dy]");
  private_node_handle_.param("post_sort_retreat_enabled", post_sort_retreat_enabled_, false);
  private_node_handle_.param("post_sort_retreat_distance", post_sort_retreat_distance_, 0.30);

  private_node_handle_.param("sorting_state_topic", sorting_state_topic_,
                             std::string("/sorting/state"));
  private_node_handle_.param("sorting_failure_topic", sorting_failure_topic_,
                             std::string("/sorting/failure"));
  private_node_handle_.param("sorting_home_service", home_service_name_,
                             std::string("/sorting/home"));
  private_node_handle_.param("sorting_prepare_service", prepare_service_name_,
                             std::string("/sorting/prepare_work"));
  private_node_handle_.param("sorting_observe_service", observe_service_name_,
                             std::string("/sorting/move_to_observation"));
  private_node_handle_.param("sorting_start_service", sort_service_name_,
                             std::string("/sorting/start"));
  private_node_handle_.param("sorting_stop_service", sorting_stop_service_name_,
                             std::string("/sorting/stop"));
  private_node_handle_.param("sorting_configure_service", configure_workspace_service_name_,
                             std::string("/sorting/configure_workspace"));
  private_node_handle_.param("sorting_workspace_param", workspace_parameter_,
                             std::string("/sorting/workspace_config"));
}

void MissionContext::validateWorkstations() const
{
  if (workstations_.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::runtime_error("workstations must be a list");
  std::set<std::string> identifiers; // 工作台 id 集合
  for (int index = 0; index < workstations_.size(); ++index)
  {
    const auto &workspace = workstations_[index];
    if (workspace.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      throw std::runtime_error("each workstation must be a mapping");
    const std::string identifier = memberString(workspace, "id"); // 工作台 id
    if (identifier.empty() || !identifiers.insert(identifier).second)
      throw std::runtime_error("each workstation needs a unique non-empty id");
    if (workspace.hasMember("navigation_goal_frame") &&
        memberString(workspace, "navigation_goal_frame").empty()) // 导航目标参考坐标系判断
      throw std::runtime_error(identifier + " has invalid navigation_goal_frame");
    memberPose(workspace, "navigation_goal"); // 导航目标的位置判断 类型+数目
    if (workspace.hasMember("pre_dock_goal")) // 直行停靠目标判断
      memberPose(workspace, "pre_dock_goal");
    for (const std::string key : {"table_center", "table_size"})
    {
      if (!workspace.hasMember(key) || workspace[key].getType() != XmlRpc::XmlRpcValue::TypeArray ||
          workspace[key].size() != 3)
        throw std::runtime_error(identifier + " has invalid " + key);
    }
  }
}

void MissionContext::publishState(MissionState state, const std::string &detail)
{
  std_msgs::String message;
  message.data = detail.empty() ? toString(state) : std::string(toString(state)) + " | " + detail;
  if (state_publisher_)
    state_publisher_.publish(message);
  ROS_INFO_STREAM("Navigation-sorting mission: " << message.data);
}

double MissionContext::angleError(double target, double actual)
{
  return std::atan2(std::sin(target - actual), std::cos(target - actual));
}

double MissionContext::number(const XmlRpc::XmlRpcValue &value)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return static_cast<int>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    return static_cast<double>(value);
  throw std::runtime_error("expected a numeric XML-RPC value");
}

bool MissionContext::memberBool(const XmlRpc::XmlRpcValue &value, const std::string &key,
                                bool fallback)
{
  if (!value.hasMember(key))
    return fallback;
  if (value[key].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    throw std::runtime_error(key + " must be boolean");
  return static_cast<bool>(value[key]);
}

double MissionContext::memberDouble(const XmlRpc::XmlRpcValue &value, const std::string &key,
                                    double fallback)
{
  return value.hasMember(key) ? number(value[key]) : fallback;
}

std::string MissionContext::memberString(const XmlRpc::XmlRpcValue &value, const std::string &key)
{
  if (!value.hasMember(key) || value[key].getType() != XmlRpc::XmlRpcValue::TypeString)
    return std::string();
  return static_cast<std::string>(value[key]);
}

MissionContext::Pose2D MissionContext::memberPose(const XmlRpc::XmlRpcValue &value,
                                                  const std::string &key)
{
  if (!value.hasMember(key) || value[key].getType() != XmlRpc::XmlRpcValue::TypeArray ||
      value[key].size() != 3)
    throw std::runtime_error(key + " must be [x, y, yaw]");
  return {number(value[key][0]), number(value[key][1]), number(value[key][2])};
}

std::string MissionContext::toJson(const XmlRpc::XmlRpcValue &value)
{
  std::ostringstream output;
  switch (value.getType())
  {
  case XmlRpc::XmlRpcValue::TypeBoolean:
    output << (static_cast<bool>(value) ? "true" : "false");
    break;
  case XmlRpc::XmlRpcValue::TypeInt:
    output << static_cast<int>(value);
    break;
  case XmlRpc::XmlRpcValue::TypeDouble:
    output << std::setprecision(16) << static_cast<double>(value);
    break;
  case XmlRpc::XmlRpcValue::TypeString:
    output << '"' << jsonEscape(static_cast<std::string>(value)) << '"';
    break;
  case XmlRpc::XmlRpcValue::TypeArray:
    output << '[';
    for (int index = 0; index < value.size(); ++index)
    {
      if (index)
        output << ',';
      output << toJson(value[index]);
    }
    output << ']';
    break;
  case XmlRpc::XmlRpcValue::TypeStruct:
  {
    output << '{';
    bool first = true;
    for (auto iterator = value.begin(); iterator != value.end(); ++iterator)
    {
      if (!first)
        output << ',';
      first = false;
      output << '"' << jsonEscape(iterator->first) << "\":" << toJson(iterator->second);
    }
    output << '}';
    break;
  }
  default:
    output << "null";
    break;
  }
  return output.str();
}
} // namespace aubo_mobile_nav_sorting
