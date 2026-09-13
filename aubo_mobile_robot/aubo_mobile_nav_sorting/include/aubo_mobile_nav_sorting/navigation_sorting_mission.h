#pragma once
#include <dynamic_reconfigure/server.h>
#include <aubo_mobile_nav_sorting/arm_executor.h>
#include <aubo_mobile_nav_sorting/base_executor.h>
namespace aubo_mobile_nav_sorting
{
class NavigationSortingMission : private MissionContext
{
public:
  NavigationSortingMission(const ros::NodeHandle &node_handle,
                           const ros::NodeHandle &private_node_handle);
  ~NavigationSortingMission();
  NavigationSortingMission(const NavigationSortingMission &) = delete;
  NavigationSortingMission &operator=(const NavigationSortingMission &) = delete;

private:
  void seedDynamicParameters();
  void reconfigureCallback(NavSortingConfig &config, uint32_t level);
  bool startCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  bool stopCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  void autoStartCallback(const ros::TimerEvent &event);
  bool submitMission(std::string &message);
  bool prepareAndObserveOnce();
  bool stowForBaseRecovery();
  bool prepareAndObserveWithRecovery();
  bool sortWithRecovery();
  bool sortAtWorkspaceWithRecovery(const XmlRpc::XmlRpcValue &workspace);
  bool retreatAfterSorting(const XmlRpc::XmlRpcValue &workspace);
  double scoreCandidate(const Pose2D &pose, bool &valid, double &clearance) const;
  std::vector<Candidate> nearFieldCandidates() const;
  bool coordinateNearField();
  bool runWorkstationSequence();
  void runMission();
  bool recoverStopCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &);
  void recoverStop();
  std::mutex lifecycle_mutex_; // Serializes admission and thread replacement.
  ros::ServiceServer recover_service_;
  std::unique_ptr<BaseExecutor> base_executor_;
  std::unique_ptr<ArmExecutor> arm_executor_;
  std::unique_ptr<dynamic_reconfigure::Server<NavSortingConfig>> dynamic_server_;
  ros::ServiceServer start_service_;
  ros::ServiceServer stop_service_;
  ros::Timer auto_start_timer_;
  std::thread mission_thread_;
};
} // namespace aubo_mobile_nav_sorting
