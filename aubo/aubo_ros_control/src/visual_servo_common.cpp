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

double applyDeadband(double value, double deadband)
{
  const double width = std::max(0.0, deadband);
  if (value > width)
    return value - width;
  if (value < -width)
    return value + width;
  return 0.0;
}

bool finitePoint(const JointPoint& point)
{
  return std::all_of(point.begin(), point.end(),
                     [](double value) { return std::isfinite(value); });
}

ServoState selectServoState(bool enabled, bool fault, bool fresh_target)
{
  if (fault) return ServoState::FAULT;
  if (!enabled) return ServoState::DISABLED;
  return fresh_target ? ServoState::TRACKING : ServoState::HOLD;
}

bool AlignmentTracker::update(bool inside_window, double now, double hold_time)
{
  if (!inside_window || !std::isfinite(now)) {
    reset();
    return false;
  }
  if (!candidate_ || now < since_) {
    candidate_ = true;
    aligned_ = false;
    since_ = now;
  }
  if (now - since_ >= hold_time)
    aligned_ = true;
  return aligned_;
}

void AlignmentTracker::reset()
{
  candidate_ = false;
  aligned_ = false;
  since_ = 0.0;
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
