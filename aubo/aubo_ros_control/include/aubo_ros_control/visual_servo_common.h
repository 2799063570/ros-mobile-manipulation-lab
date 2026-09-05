#ifndef AUBO_ROS_CONTROL_VISUAL_SERVO_COMMON_H
#define AUBO_ROS_CONTROL_VISUAL_SERVO_COMMON_H

#include <array>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>

namespace aubo_ros_control
{
namespace visual_servo_internal
{

// AUBO i5 的关节自由度以及控制链中统一使用的关节点类型。
constexpr std::size_t kDof = 6;
using JointPoint = std::array<double, kDof>;

// 数值和关节点的基础校验工具，由控制器与 SDK 后端共同使用。
double clampValue(double value, double low, double high);
// 连续软死区：区间内为零，区间外扣除死区宽度，避免边界处指令突跳。
double applyDeadband(double value, double deadband);
bool finitePoint(const JointPoint& point);

// 有界命令队列：生产速度过快时丢弃最旧指令，避免视觉伺服延迟累积。
class CommandQueue
{
public:
  explicit CommandQueue(std::size_t capacity);

  void push(const JointPoint& point);
  bool pop(JointPoint& point);
  void clear();
  std::size_t size() const;

private:
  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<JointPoint> queue_;
};

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control

#endif  // AUBO_ROS_CONTROL_VISUAL_SERVO_COMMON_H
