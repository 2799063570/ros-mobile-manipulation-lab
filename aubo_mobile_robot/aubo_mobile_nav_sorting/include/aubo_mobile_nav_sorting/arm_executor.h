#pragma once
#include <aubo_mobile_nav_sorting/mission_context.h>
namespace aubo_mobile_nav_sorting
{
class ArmExecutor
{
public:
  explicit ArmExecutor(MissionContext &context);
  ArmExecutor(const ArmExecutor &) = delete;
  ArmExecutor &operator=(const ArmExecutor &) = delete;
  bool waitForSortingReady();
  bool cancelSortingAndWait();
  bool recoverStop();
  bool planningFailed() const;
  bool configureWorkspace(const XmlRpc::XmlRpcValue &workspace);
  bool home(const std::string &label)
  {
    return callSortingOperation(home_client_, {SortingState::HOMING}, label);
  }
  bool prepare(const std::string &label)
  {
    return callSortingOperation(prepare_client_, {SortingState::PREPARING}, label);
  }
  bool observe(const std::string &label)
  {
    return callSortingOperation(observe_client_, {SortingState::OBSERVING}, label);
  }
  bool sort(const std::string &label)
  {
    return callSortingOperation(sort_client_,
                                {SortingState::SORTING, SortingState::DETECTING,
                                 SortingState::PICKING, SortingState::OBSERVING,
                                 SortingState::HOMING},
                                label);
  }

private:
  bool callTriggerBounded(ros::ServiceClient client, std_srvs::Trigger &service, double timeout,
                          bool cancelable = true, bool late_stop = true);
  void baseLockCallback(const std_msgs::Bool::ConstPtr &message);
  bool operationUnlocked() const;
  bool callSortingOperation(ros::ServiceClient &client,
                            const std::vector<SortingState> &running_states,
                            const std::string &label);
  void sortingStateCallback(const std_msgs::String::ConstPtr &message);
  void sortingFailureCallback(const std_msgs::String::ConstPtr &message);
  void stopBase()
  {
    context_.stop_base();
  }
  MissionContext &context_;
  ros::Publisher workspace_publisher_;
  ros::Subscriber sorting_state_subscriber_;
  ros::Subscriber sorting_failure_subscriber_;
  ros::Subscriber base_lock_subscriber_;
  ros::ServiceClient home_client_;
  ros::ServiceClient prepare_client_;
  ros::ServiceClient observe_client_;
  ros::ServiceClient sort_client_;
  ros::ServiceClient sorting_stop_client_;
  ros::ServiceClient configure_workspace_client_;
};
} // namespace aubo_mobile_nav_sorting
