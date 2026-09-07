#include <aubo_mobile_nav_sorting/arm_executor.h>
#include <aubo_mobile_nav_sorting/bounded_rpc.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
namespace aubo_mobile_nav_sorting
{
ArmExecutor::ArmExecutor(MissionContext &context) : context_(context)
{
  workspace_publisher_ = context_.node_handle_.advertise<std_msgs::String>(
      "/nav_sorting/current_workstation", 1, true); // 发布当前工作台
  sorting_state_subscriber_ = context_.node_handle_.subscribe(
      context_.sorting_state_topic_, 5, &ArmExecutor::sortingStateCallback, this); // 订阅分拣状态
  sorting_failure_subscriber_ = context_.node_handle_.subscribe(
      context_.sorting_failure_topic_, 5, &ArmExecutor::sortingFailureCallback,
      this); // 订阅分拣失败状态
  std::string base_lock_topic;
  context_.private_node_handle_.param("sorting_base_lock_topic", base_lock_topic,
                                      std::string("/sorting/base_locked"));
  base_lock_subscriber_ = context_.node_handle_.subscribe(
      base_lock_topic, 5, &ArmExecutor::baseLockCallback, this); // 订阅机械臂执行期间的底盘锁定状态

  home_client_ = context_.node_handle_.serviceClient<std_srvs::Trigger>(
      context_.home_service_name_); // 请求机械臂移动到初始位姿
  prepare_client_ = context_.node_handle_.serviceClient<std_srvs::Trigger>(
      context_.prepare_service_name_); // 请求机械臂移动到工位准备姿态
  observe_client_ = context_.node_handle_.serviceClient<std_srvs::Trigger>(
      context_.observe_service_name_); // 请求机械臂移动到观测位姿
  sort_client_ = context_.node_handle_.serviceClient<std_srvs::Trigger>(
      context_.sort_service_name_); // 请求分拣
  sorting_stop_client_ = context_.node_handle_.serviceClient<std_srvs::Trigger>(
      context_.sorting_stop_service_name_); // 请求停止拣选
  configure_workspace_client_ = context_.node_handle_.serviceClient<std_srvs::Trigger>(
      context_.configure_workspace_service_name_); // 请求配置工作台
}

bool ArmExecutor::waitForSortingReady()
{
  // 阻塞等待下游 ColorSortingTask 节点就绪
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(context_.initialization_timeout_);
  std::unique_lock<std::mutex> lock(context_.mutex_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (context_.stop_requested_)
      return false;
    const SortingState state = context_.sorting_state_;
    if (state == SortingState::IDLE || state == SortingState::READY ||
        state == SortingState::STOPPED)
      return true;
    if (state == SortingState::ERROR)
      return false;
    context_.condition_.wait_for(lock, std::chrono::milliseconds(200));
  }
  ROS_ERROR("Timed out waiting for sorting node readiness");
  return false;
}

bool ArmExecutor::callTriggerBounded(ros::ServiceClient client, std_srvs::Trigger &service,
                                     double timeout, bool cancelable, bool late_stop)
{
  auto stop_client = sorting_stop_client_;
  const auto status = boundedRpc(
      client, service, timeout,
      [this, cancelable]() { return cancelable && context_.stop_requested_; },
      late_stop ? std::function<void()>([stop_client]() mutable {
        std_srvs::Trigger stop;
        if (!stop_client.call(stop) || !stop.response.success)
          ROS_ERROR("Late sorting cancellation failed");
      })
                : std::function<void()>(),
      context_.pending_rpcs_);
  if (status == RpcStatus::Abandoned && late_stop)
    context_.stop_unconfirmed_ = true;
  if (status != RpcStatus::Completed)
    ROS_ERROR_STREAM("Service failed or interrupted: " << client.getService());
  return status == RpcStatus::Completed;
}

void ArmExecutor::baseLockCallback(const std_msgs::Bool::ConstPtr &message)
{
  std::lock_guard<std::mutex> lock(context_.mutex_);
  context_.base_locked_ = message->data;
  ++context_.base_lock_sequence_;
  context_.condition_.notify_all();
}

bool ArmExecutor::operationUnlocked() const
{
  // Caller holds context_.mutex_. ERROR may precede the sorting worker's cleanup.
  return !context_.base_locked_ && context_.base_lock_sequence_ > context_.operation_lock_sequence_;
}

bool ArmExecutor::recoverStop()
{
  if (context_.pending_rpcs_->load() != 0)
    return false;
  {
    std::lock_guard<std::mutex> lock(context_.mutex_);
    context_.operation_start_sequence_ = context_.sorting_sequence_;
    context_.operation_lock_sequence_ = context_.base_lock_sequence_;
  }
  if (!cancelSortingAndWait() || context_.pending_rpcs_->load() != 0)
    return false;
  context_.stop_unconfirmed_ = false;
  return true;
}

bool ArmExecutor::cancelSortingAndWait()
{
  context_.publishState(MissionState::STOPPING, "waiting for sorting cancellation");
  stopBase();
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(context_.stop_timeout_);
  std_srvs::Trigger stop;
  if (!callTriggerBounded(sorting_stop_client_, stop, context_.stop_timeout_, false, false) ||
      !stop.response.success)
  {
    context_.stop_unconfirmed_ = true;
    return false;
  }
  std::unique_lock<std::mutex> lock(context_.mutex_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    const SortingState state = context_.sorting_state_;
    if (context_.sorting_sequence_ > context_.operation_start_sequence_ &&
        (state == SortingState::READY || state == SortingState::ERROR ||
         state == SortingState::STOPPED) &&
        operationUnlocked())
    {
      context_.operation_active_ = false;
      return true;
    }
    context_.condition_.wait_for(lock, std::chrono::milliseconds(50));
  }
  context_.stop_unconfirmed_ = true;
  return false;
}

bool ArmExecutor::callSortingOperation(ros::ServiceClient &client,
                                       const std::vector<SortingState> &running_states,
                                       const std::string &label)
{
  if (context_.stop_requested_ || context_.operation_active_ || context_.stop_unconfirmed_)
    return false;
  unsigned long start_sequence;
  {
    std::lock_guard<std::mutex> lock(context_.mutex_);
    start_sequence = context_.sorting_sequence_;
    context_.operation_start_sequence_ = start_sequence;
    context_.operation_lock_sequence_ = context_.base_lock_sequence_;
    context_.sorting_failure_.clear();
  }
  context_.operation_active_ = true;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(context_.operation_timeout_);
  std_srvs::Trigger service;
  if (!callTriggerBounded(client, service, context_.operation_timeout_))
    return false;
  if (!service.response.success)
  {
    context_.operation_active_ = false;
    ROS_ERROR_STREAM(label << " command failed: " << service.response.message);
    return false;
  }
  bool saw_operation = false;
  std::unique_lock<std::mutex> lock(context_.mutex_);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (context_.stop_requested_)
      return false;
    const SortingState state = context_.sorting_state_;
    if (context_.sorting_sequence_ > start_sequence)
    {
      if (std::find(running_states.begin(), running_states.end(), state) != running_states.end())
        saw_operation = true;
      else if (state == SortingState::READY &&
               (saw_operation || context_.sorting_sequence_ > start_sequence + 1) &&
               operationUnlocked())
      {
        context_.operation_active_ = false;
        return true;
      }
      else if ((state == SortingState::ERROR || state == SortingState::STOPPED) &&
               operationUnlocked())
      {
        context_.operation_active_ = false;
        ROS_ERROR_STREAM(label << " failed: " << context_.sorting_state_detail_);
        return false;
      }
    }
    if (context_.stop_requested_)
      return false;
    context_.condition_.wait_for(lock, std::chrono::milliseconds(200));
  }
  ROS_ERROR_STREAM(label << " timed out after " << context_.operation_timeout_ << " s");
  return false;
}

bool ArmExecutor::planningFailed() const
{
  std::lock_guard<std::mutex> lock(context_.mutex_);
  return !context_.operation_active_ && !context_.stop_unconfirmed_ && !context_.stop_requested_ &&
         context_.sorting_failure_.find("PLANNING_FAILED") == 0;
}

bool ArmExecutor::configureWorkspace(const XmlRpc::XmlRpcValue &workspace)
{
  XmlRpc::XmlRpcValue payload;
  for (auto iterator = workspace.begin(); iterator != workspace.end(); ++iterator)
  {
    if (iterator->first != "navigation_goal" && iterator->first != "pre_dock_goal" &&
        iterator->first != "navigation_goal_frame" && iterator->first != "enabled")
      payload[iterator->first] = iterator->second;
  }
  ros::param::set(context_.workspace_parameter_, payload);
  std_msgs::String message;
  message.data = context_.toJson(workspace);
  workspace_publisher_.publish(message);
  if (context_.stop_requested_)
    return false;
  std_srvs::Trigger service;
  if (!callTriggerBounded(configure_workspace_client_, service, context_.server_timeout_) ||
      !service.response.success)
  {
    ROS_ERROR_STREAM("Workstation configuration rejected: " << service.response.message);
    return false;
  }
  return true;
}

void ArmExecutor::sortingStateCallback(const std_msgs::String::ConstPtr &message)
{
  std::lock_guard<std::mutex> lock(context_.mutex_);
  context_.sorting_state_ = parseSortingState(message->data);
  context_.sorting_state_detail_ = message->data;
  ++context_.sorting_sequence_;
  context_.condition_.notify_all();
}

void ArmExecutor::sortingFailureCallback(const std_msgs::String::ConstPtr &message)
{
  std::lock_guard<std::mutex> lock(context_.mutex_);
  context_.sorting_failure_ = message->data;
  context_.condition_.notify_all();
}

} // namespace aubo_mobile_nav_sorting
