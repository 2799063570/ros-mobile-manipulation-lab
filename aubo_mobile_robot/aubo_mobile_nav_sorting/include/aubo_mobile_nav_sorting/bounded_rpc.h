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

// ROS 1 服务调用无法逐次取消；超时只结束本地等待，不代表远端动作已经停止。
// 分离线程必须持有自己的数据，不能引用任务对象；放弃有副作用的调用后，
// 调用方须锁住任务重启入口，直到停止状态得到确认。
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
      // 延迟的启动请求可能晚于第一次停止才生效，返回后再次停止。
      // 此处不解除重启互锁；pending 也要等补偿停止结束后才递减。
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
  // 单调时钟保证服务超时不受仿真暂停或系统时间校准影响。
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
