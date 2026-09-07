#pragma once
#include <aubo_mobile_nav_sorting/mission_context.h>
#include <functional>
#include <chrono>
namespace aubo_mobile_nav_sorting
{
enum class RpcStatus
{
  Completed,
  Failed,
  Abandoned
};

// ROS 1 service calls have no per-call cancellation. Detached workers own all
// their data (never the mission object); an abandoned mutation locks out restart.
template <typename Service>
RpcStatus boundedRpc(ros::ServiceClient client, Service &service, double timeout,
                     const std::function<bool()> &cancelled,
                     const std::function<void()> &late_stop = {},
                     std::shared_ptr<std::atomic<unsigned>> pending = {})
{
  struct Result
  {
    std::mutex mutex;
    std::condition_variable condition;
    bool done = false;
    bool abandoned = false;
    bool success = false;
    Service service;
  };
  auto result = std::make_shared<Result>();
  result->service = service;
  if (pending)
    ++*pending;
  try
  {
    std::thread([client, result, late_stop, pending]() mutable {
      bool success = false;
      try
      {
        success = client.call(result->service);
      }
      catch (const std::exception &error)
      {
        ROS_ERROR_STREAM(error.what());
      }
      bool abandoned;
      {
        std::lock_guard<std::mutex> lock(result->mutex);
        result->success = success;
        result->done = true;
        abandoned = result->abandoned;
        result->condition.notify_all();
      }
      // A delayed start can arrive after the initial stop. Cancel once more on
      // its return, without clearing the restart interlock.
      if (abandoned && late_stop)
      {
        try
        {
          late_stop();
        }
        catch (const std::exception &error)
        {
          ROS_ERROR_STREAM(error.what());
        }
      }
      if (pending)
        --*pending;
    }).detach();
  }
  catch (...)
  {
    if (pending)
      --*pending;
    throw;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  std::unique_lock<std::mutex> lock(result->mutex);
  while (!result->done)
  {
    if (!ros::ok() || cancelled() || std::chrono::steady_clock::now() >= deadline)
    {
      result->abandoned = true;
      return RpcStatus::Abandoned;
    }
    result->condition.wait_for(lock, std::chrono::milliseconds(50));
  }
  service = result->service;
  return result->success ? RpcStatus::Completed : RpcStatus::Failed;
}

} // namespace aubo_mobile_nav_sorting
