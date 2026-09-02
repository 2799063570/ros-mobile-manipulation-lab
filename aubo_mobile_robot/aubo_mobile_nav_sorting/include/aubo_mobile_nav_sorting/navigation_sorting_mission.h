#ifndef AUBO_MOBILE_NAV_SORTING_NAVIGATION_SORTING_MISSION_H
#define AUBO_MOBILE_NAV_SORTING_NAVIGATION_SORTING_MISSION_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <XmlRpcValue.h>

#include <aubo_mobile_nav_sorting/NavSortingConfig.h>

namespace aubo_mobile_nav_sorting
{

class NavigationSortingMission
{
public:
  NavigationSortingMission(const ros::NodeHandle& node_handle,
                           const ros::NodeHandle& private_node_handle);
  ~NavigationSortingMission();

  NavigationSortingMission(const NavigationSortingMission&) = delete;
  NavigationSortingMission& operator=(const NavigationSortingMission&) = delete;

private:
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

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

  void loadParameters();
  void validateWorkstations() const;
  void seedDynamicParameters();
  void reconfigureCallback(NavSortingConfig& config, uint32_t level);

  bool startCallback(std_srvs::Trigger::Request& request,
                     std_srvs::Trigger::Response& response);
  bool stopCallback(std_srvs::Trigger::Request& request,
                    std_srvs::Trigger::Response& response);
  void sortingStateCallback(const std_msgs::String::ConstPtr& message);
  void sortingFailureCallback(const std_msgs::String::ConstPtr& message);
  void autoStartCallback(const ros::TimerEvent& event);
  bool submitMission(std::string& message);
  void runMission();
  bool runWorkstationSequence();

  bool waitForSortingReady();
  bool callSortingOperation(ros::ServiceClient& client,
                            const std::vector<std::string>& running_states,
                            const std::string& label);
  bool configureWorkspace(const XmlRpc::XmlRpcValue& workspace);
  bool planningFailed() const;

  bool navigate(const Pose2D& target, const std::string& stage,
                const std::string& goal_frame = std::string());
  bool navigateOnce(const Pose2D& target, const std::string& goal_frame);
  bool currentBasePose(Pose2D& pose,
                       const std::string& pose_frame = std::string()) const;
  bool alignHeading(double target_yaw, const std::string& label,
                    const std::string& pose_frame = std::string());
  bool driveStraightTo(const Pose2D& target, const std::string& label,
                       const std::string& state = "DIRECT_DOCKING",
                       const std::string& pose_frame = std::string());
  bool moveBaseDirect(double dx, double dy, int attempt, int total);
  void stopBase();

  bool prepareAndObserveOnce();
  bool prepareAndObserveWithRecovery();
  bool sortWithRecovery();
  bool stowForBaseRecovery();
  bool retreatAfterSorting(const XmlRpc::XmlRpcValue& workspace);
  bool coordinateNearField();
  std::vector<Candidate> nearFieldCandidates() const;
  double scoreCandidate(const Pose2D& pose, bool& valid,
                        double& clearance) const;
  bool canDirectDock(const Pose2D& start, const Pose2D& target) const;

  void publishState(const std::string& state,
                    const std::string& detail = std::string());
  static std::string stateName(const std::string& state);
  static double angleError(double target, double actual);
  static double number(const XmlRpc::XmlRpcValue& value);
  static bool memberBool(const XmlRpc::XmlRpcValue& value,
                         const std::string& key, bool fallback);
  static double memberDouble(const XmlRpc::XmlRpcValue& value,
                             const std::string& key, double fallback);
  static std::string memberString(const XmlRpc::XmlRpcValue& value,
                                  const std::string& key);
  static Pose2D memberPose(const XmlRpc::XmlRpcValue& value,
                           const std::string& key);
  static std::string toJson(const XmlRpc::XmlRpcValue& value);

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  std::unique_ptr<MoveBaseClient> navigation_client_;
  mutable tf::TransformListener tf_listener_;
  std::unique_ptr<dynamic_reconfigure::Server<NavSortingConfig>> dynamic_server_;

  ros::Publisher state_publisher_;
  ros::Publisher workspace_publisher_;
  ros::Publisher velocity_publisher_;
  ros::Subscriber sorting_state_subscriber_;
  ros::Subscriber sorting_failure_subscriber_;
  ros::ServiceServer start_service_;
  ros::ServiceServer stop_service_;
  ros::ServiceClient clear_costmaps_client_;
  ros::ServiceClient home_client_;
  ros::ServiceClient prepare_client_;
  ros::ServiceClient observe_client_;
  ros::ServiceClient sort_client_;
  ros::ServiceClient sorting_stop_client_;
  ros::ServiceClient configure_workspace_client_;
  ros::Timer auto_start_timer_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread mission_thread_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> stop_requested_{false};
  std::string sorting_state_;
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

  Pose2D sorting_goal_;
  Pose2D pre_dock_goal_;
  std::vector<double> candidate_x_;
  std::vector<double> candidate_y_;
  std::vector<double> candidate_yaw_;
  std::vector<double> table_geometry_;
  std::vector<std::vector<double>> workpiece_points_;
  std::vector<double> detector_workspace_;
  std::vector<double> camera_target_;
  XmlRpc::XmlRpcValue workstations_;
  std::vector<std::vector<double>> recovery_steps_;

  bool near_field_enabled_ = false;
  bool direct_dock_enabled_ = true;
  bool heading_alignment_enabled_ = true;
  bool home_before_navigation_ = true;
  bool base_recovery_enabled_ = true;
  bool post_sort_retreat_enabled_ = false;
  bool auto_start_ = false;
  int near_field_max_candidates_ = 6;
  int navigation_retries_ = 1;
  double base_clearance_ = 0.40;
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
  double startup_delay_ = 3.0;
  double base_recovery_speed_ = 0.04;
  double base_recovery_rate_ = 20.0;
  double base_recovery_settle_time_ = 0.8;
  double post_sort_retreat_distance_ = 0.30;
};

}  // namespace aubo_mobile_nav_sorting

#endif  // AUBO_MOBILE_NAV_SORTING_NAVIGATION_SORTING_MISSION_H
