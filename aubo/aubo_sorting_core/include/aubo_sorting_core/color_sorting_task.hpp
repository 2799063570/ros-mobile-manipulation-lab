#ifndef AUBO_SORTING_CORE_COLOR_SORTING_TASK_HPP
#define AUBO_SORTING_CORE_COLOR_SORTING_TASK_HPP

#include <actionlib/client/simple_action_client.h>
#include <aubo_perception/DetectedObject.h>
#include <aubo_perception/DetectedObjectArray.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace aubo_sorting_core
{

class ColorSortingTask
{
public:
  ColorSortingTask(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~ColorSortingTask();

  ColorSortingTask(const ColorSortingTask&) = delete;
  ColorSortingTask& operator=(const ColorSortingTask&) = delete;

  void start();

private:
  using GripperClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

  struct TargetTrack     // 目标跟踪结构体 
  {
    double x{0.0};
    double y{0.0};
    int count{0};
    double m2{0.0};
    ros::WallTime last_seen;
    bool picked{false};
  };

  struct WorkspaceConfig
  {
    std::string id;
    std::vector<double> table_center;
    std::vector<double> table_size;
    std::string table_frame;
    double table_z{0.0};
    std::string place_frame;
    std::map<std::string, std::vector<double>> place_targets;
    std::map<std::string, std::string> grasp_model_names;
  };

  void loadParameters();
  bool verifyLoadedUpperArmLimit() const;
  void initialize();
  void publishState(const std::string& state, const std::string& detail = std::string());
  void setFailure(const std::string& category, const std::string& detail);

  void detectionCallback(const aubo_perception::DetectedObjectArrayConstPtr& message);
  void graspStatusCallback(const std_msgs::StringConstPtr& message);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  void planningSceneCallback(const moveit_msgs::PlanningSceneConstPtr& message);
  void workspaceUpdateCallback(const std_msgs::StringConstPtr& message);

  bool observeService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response);
  bool startService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response);
  bool stopService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response);
  bool openService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response);
  bool prepareWorkService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response);
  bool homeService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response);
  bool configureWorkspaceService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response);

  std::pair<bool, std::string> startOperation(const std::string& state,
                                              const std::function<bool()>& operation);
  bool observationOperation();
  bool initialObservationOperation();
  bool openOperation();
  bool homeOperation();
  bool prepareWorkOperation();
  bool sortingOperation();
  bool observation();
  bool verifyVisibleColors();
  bool pickAndPlace(const aubo_perception::DetectedObject& detected);

  geometry_msgs::PoseStamped makePose(double x, double y, double z) const;
  bool xyInTargetFrame(const std::string& source_frame, const std::vector<double>& xy,
                       double& x, double& y);
  bool moveToPose(const geometry_msgs::PoseStamped& pose, const std::string& description);
  bool moveNamed(const std::string& target);
  bool planAndExecute(const std::string& description);
  bool cartesianTo(const geometry_msgs::PoseStamped& target_pose,
                   const std::string& description);
  bool commandGripper(double position);
  bool addTableCollision();
  bool refreshOctomap();

  bool waitForGraspPlugin();
  bool setGraspAttachment(const std::string& model_name, bool attach);
  void releaseAttachedObjectNoWait();

  void updateTargetCache(const aubo_perception::DetectedObjectArray& message);
  bool detectionInCacheFrame(const aubo_perception::DetectedObjectArray& message,
                             const aubo_perception::DetectedObject& detected,
                             double& x, double& y);
  void publishTargetCache();
  void markTargetPicked(const std::string& color);
  bool cachedObject(const std::string& color, aubo_perception::DetectedObject& detected);
  bool waitForObject(const std::string& color, const ros::WallTime& not_before,
                     aubo_perception::DetectedObject& detected);

  bool workspaceFromParam(const XmlRpc::XmlRpcValue& value, WorkspaceConfig& workspace,
                          std::string& error) const;
  bool workspaceFromJson(const std::string& json, WorkspaceConfig& workspace,
                         std::string& error) const;
  bool applyWorkspace(const WorkspaceConfig& workspace, std::string& error);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener tf_listener_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  std::unique_ptr<GripperClient> gripper_client_;

  ros::Publisher state_publisher_;
  ros::Publisher detection_summary_publisher_;
  ros::Publisher base_lock_publisher_;
  ros::Publisher failure_publisher_;
  ros::Publisher target_cache_publisher_;
  ros::Publisher grasp_attach_publisher_;
  ros::Publisher grasp_detach_publisher_;
  ros::Subscriber detection_subscriber_;
  ros::Subscriber grasp_status_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber planning_scene_subscriber_;
  ros::Subscriber workspace_subscriber_;
  ros::ServiceClient clear_octomap_client_;
  std::vector<ros::ServiceServer> services_;

  std::string group_name_;
  std::string end_effector_link_;
  std::string target_frame_;
  std::string detections_topic_;
  std::string gripper_action_name_;
  std::string table_frame_;
  std::string observation_named_target_;
  std::string work_ready_named_target_;
  std::string grasp_attach_topic_;
  std::string grasp_detach_topic_;
  std::string grasp_status_topic_;
  std::string place_frame_;
  std::string finish_named_target_;
  std::string point_cloud_topic_;
  std::string planning_scene_topic_;
  std::string clear_octomap_service_;
  std::string base_lock_topic_;
  std::string target_cache_frame_;
  std::string failure_topic_;
  std::string workspace_config_param_;
  std::string workspace_update_topic_;
  std::string planning_frame_;

  std::vector<double> table_center_;
  std::vector<double> table_size_;
  std::vector<double> grasp_rpy_;
  std::vector<double> observation_pose_;
  std::vector<std::string> sort_colors_;
  std::map<std::string, std::string> grasp_model_names_;
  std::map<std::string, std::vector<double>> place_targets_;

  double table_z_{0.14};
  double table_collision_margin_{0.0};
  double object_height_{0.04};
  double grasp_height_offset_{0.01};
  double pregrasp_height_{0.25};
  double lift_height_{0.30};
  double place_clearance_{0.02};
  double cartesian_step_{0.01};
  double minimum_cartesian_fraction_{0.90};
  double gripper_open_{0.0};
  double gripper_closed_{0.28};
  double gripper_motion_time_{2.5};
  double gripper_contact_tolerance_{0.30};
  double grasp_attachment_timeout_{3.0};
  double detection_timeout_{15.0};
  double detection_settle_time_{1.0};
  double observation_verification_timeout_{4.0};
  double grasp_offset_x_{0.0};
  double grasp_offset_y_{0.0};
  double velocity_scaling_{0.15};
  double acceleration_scaling_{0.15};
  double gripper_server_timeout_{30.0};
  double scene_update_timeout_{10.0};
  double octomap_wait_timeout_{30.0};
  double target_cache_max_age_{30.0};
  double target_cache_outlier_distance_{0.12};
  double target_cache_fallback_delay_{2.0};
  double planning_time_{12.0};
  int detection_samples_{8};
  int observation_verification_min_frames_{1};
  int target_cache_min_observations_{5};
  bool use_grasp_attachment_{true};
  bool verify_observation_detections_{false};
  bool auto_move_to_observation_{true};
  bool auto_start_{false};
  bool require_octomap_{false};
  bool target_cache_fallback_enabled_{true};

  mutable std::mutex data_mutex_;
  mutable std::mutex operation_mutex_;
  aubo_perception::DetectedObjectArrayConstPtr detections_;
  ros::WallTime detections_wall_time_;
  std::string grasp_status_;
  std::uint64_t grasp_status_sequence_{0};
  std::string attached_model_;
  ros::WallTime last_cloud_wall_time_;
  std::size_t cloud_points_{0};
  std::uint64_t octomap_sequence_{0};
  std::string last_failure_;
  std::string workspace_id_{"default"};
  WorkspaceConfig pending_workspace_;
  bool has_pending_workspace_{false};
  std::set<std::string> completed_colors_;
  std::map<std::string, TargetTrack> target_tracks_;
  std::string state_{"INITIALIZING"};
  std::atomic<bool> busy_{true};
  std::atomic<bool> initialized_{false};
  std::atomic<bool> observation_ready_{false};
  std::atomic<bool> stop_requested_{false};
  bool robot_limits_valid_{false};
  std::thread initialization_thread_;
  std::thread operation_thread_;
};

}  // namespace aubo_sorting_core

#endif  // AUBO_SORTING_CORE_COLOR_SORTING_TASK_HPP
