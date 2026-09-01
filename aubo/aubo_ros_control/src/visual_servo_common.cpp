#include <aubo_ros_control/visual_servo_common.h>

#include <algorithm>
#include <cmath>

namespace aubo_ros_control
{
namespace visual_servo_internal
{

double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

bool finitePoint(const JointPoint& point)
{
  return std::all_of(point.begin(), point.end(),
                     [](double value) { return std::isfinite(value); });
}

CommandQueue::CommandQueue(std::size_t capacity)
  : capacity_(std::max<std::size_t>(2, capacity))
{
}

void CommandQueue::push(const JointPoint& point)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // 控制队列保持有界；生产端短时过快时，新视觉修正比旧的未执行点更有价值。
  if (queue_.size() >= capacity_)
    queue_.pop_front();
  queue_.push_back(point);
  condition_.notify_one();
}

bool CommandQueue::pop(JointPoint& point)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty())
    return false;
  point = queue_.front();
  queue_.pop_front();
  return true;
}

void CommandQueue::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
}

std::size_t CommandQueue::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control
