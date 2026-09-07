#pragma once
#include <atomic>
#include <functional>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <XmlRpcValue.h>

#include <aubo_mobile_nav_sorting/NavSortingConfig.h>

#include <aubo_mobile_nav_sorting/mission_state.h>
namespace aubo_mobile_nav_sorting
{
// Settings are frozen while busy_ is true. Callback data and lifecycle admission
// use mutex_; cancellation flags are atomic. Execution scratch is worker-only.
struct MissionContext
{
  MissionContext(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);
  // Only reload while no executor is running; the mission freezes these settings.
  void loadParameters();
  void validateWorkstations() const;
  struct Pose2D
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  struct Candidate
  {
    Pose2D pose;
    double score = 0.0;
    double clearance = 0.0;
  };

  void publishState(MissionState state, const std::string &detail = std::string());// 发布 任务状态+detail(任务状态详情)
  static double angleError(double target, double actual);
  static double number(const XmlRpc::XmlRpcValue &value);// 将XmlRpcValue类型的数值转换为double类型
  static bool memberBool(const XmlRpc::XmlRpcValue &value, const std::string &key, bool fallback);// 从XmlRpcValue类型的参数中提取指定键的bool类型数值
  static double memberDouble(const XmlRpc::XmlRpcValue &value, const std::string &key,
                             double fallback);// 从XmlRpcValue类型的参数中提取指定键的double类型数值
  static std::string memberString(const XmlRpc::XmlRpcValue &value, const std::string &key);// 从XmlRpcValue类型的参数中提取指定键的string类型数值
  static Pose2D memberPose(const XmlRpc::XmlRpcValue &value, const std::string &key);// 从XmlRpcValue类型的参数中提取指定键的Pose2D类型数值
  static std::string toJson(const XmlRpc::XmlRpcValue &value);// 将XmlRpcValue类型的数值转换为JSON字符串
  std::function<void()> stop_base = [] {};
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  ros::Publisher state_publisher_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::shared_ptr<std::atomic<unsigned>> pending_rpcs_{new std::atomic<unsigned>(0)};
  std::atomic<bool> busy_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> stop_unconfirmed_{false};
  bool operation_active_ = false; // Accessed only by the mission worker.
  bool base_pose_failed_ = false; // Execution worker only.
  unsigned long operation_start_sequence_ = 0;
  // The following callback values are read/written under mutex_.
  bool base_locked_ = true;
  unsigned long base_lock_sequence_ = 0;
  unsigned long operation_lock_sequence_ = 0;
  SortingState sorting_state_ = SortingState::UNKNOWN;
  std::string sorting_state_detail_;
  std::string sorting_failure_;
  unsigned long sorting_sequence_ = 0;

  std::string navigation_action_;
  std::string navigation_frame_;
  std::string base_frame_;
  std::string velocity_topic_;
  std::string sorting_state_topic_;
  std::string sorting_failure_topic_;
  std::string home_service_name_;
  std::string prepare_service_name_;
  std::string observe_service_name_;
  std::string sort_service_name_;
  std::string sorting_stop_service_name_;
  std::string configure_workspace_service_name_;
  std::string workspace_parameter_;

  Pose2D sorting_goal_;// 导航的目标点位置和角度
  Pose2D pre_dock_goal_;// 预停点位置和角度
  std::vector<double> candidate_x_;
  std::vector<double> candidate_y_;
  std::vector<double> candidate_yaw_;
  std::vector<double> table_geometry_;
  std::vector<std::vector<double>> workpiece_points_;
  std::vector<double> detector_workspace_;
  std::vector<double> camera_target_;
  XmlRpc::XmlRpcValue workstations_;// 工位配置
  std::vector<std::vector<double>> recovery_steps_;

  bool near_field_enabled_ = false;// 是否启用近场直行
  bool direct_dock_enabled_ = true;// 是否启用直接停靠
  bool heading_alignment_enabled_ = true;
  bool home_before_navigation_ = true;
  bool base_recovery_enabled_ = true;
  bool post_sort_retreat_enabled_ = false;
  bool auto_start_ = false;
  bool return_to_start_ = true;
  std::string return_frame_;
  int near_field_max_candidates_ = 6;
  int navigation_retries_ = 1;
  double base_clearance_ = 0.40;// 近场直行的最小基线距离
  double direct_dock_max_distance_ = 0.50;
  double direct_dock_lateral_tolerance_ = 0.04;
  double direct_dock_yaw_tolerance_ = 0.04;
  double direct_dock_goal_tolerance_ = 0.06;
  double direct_dock_timeout_ = 15.0;
  double direct_dock_stall_timeout_ = 2.5;
  double direct_dock_progress_epsilon_ = 0.005;
  double heading_max_correction_ = 0.12;
  double heading_speed_ = 0.12;
  double heading_goal_tolerance_ = 0.015;
  double heading_final_tolerance_ = 0.025;
  double heading_timeout_ = 4.0;
  double heading_stall_timeout_ = 1.5;
  double navigation_timeout_ = 180.0;
  double server_timeout_ = 45.0;
  double initialization_timeout_ = 60.0;
  double operation_timeout_ = 300.0;
  double stop_timeout_ = 5.0;
  double tf_max_age_ = 0.5;
  double startup_delay_ = 3.0;
  double base_recovery_speed_ = 0.04;
  double base_recovery_rate_ = 20.0;
  double base_recovery_settle_time_ = 0.8;
  double post_sort_retreat_distance_ = 0.30;
};
} // namespace aubo_mobile_nav_sorting
