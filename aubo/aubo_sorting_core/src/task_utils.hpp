#ifndef AUBO_SORTING_CORE_TASK_UTILS_HPP
#define AUBO_SORTING_CORE_TASK_UTILS_HPP

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <atomic>
#include <string>
#include <vector>

// Internal helpers shared by the task implementation; not part of the exported API.
namespace aubo_sorting_core
{
namespace detail
{
bool wallSleep(double seconds, const std::atomic<bool>& stop_requested);
std::string join(const std::vector<std::string>& values, const std::string& separator);
std::string jsonEscape(const std::string& value);
double xmlNumber(const XmlRpc::XmlRpcValue& value);
std::vector<double> xmlVector(const XmlRpc::XmlRpcValue& value, int expected_size,
                              const std::string& name);
}  // namespace detail
}  // namespace aubo_sorting_core

#endif
