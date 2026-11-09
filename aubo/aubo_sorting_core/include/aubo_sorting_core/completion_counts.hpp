#ifndef AUBO_SORTING_CORE_COMPLETION_COUNTS_HPP
#define AUBO_SORTING_CORE_COMPLETION_COUNTS_HPP

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace aubo_sorting_core
{
// Controller-confirmed completed pick/place operations, independent of queue retention.
class CompletionCounts
{
public:
  using Counts = std::vector<std::pair<std::string, std::uint64_t>>;

  void configure(const std::vector<std::string>& categories)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& category : categories) addCategory(category);
  }

  void completed(const std::string& category)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    addCategory(category);
    ++counts_[category];
  }

  Counts snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Counts result;
    for (const auto& category : order_) result.emplace_back(category, counts_.at(category));
    return result;
  }

private:
  void addCategory(const std::string& category)
  {
    if (counts_.emplace(category, 0).second) order_.push_back(category);
  }
  mutable std::mutex mutex_;
  std::vector<std::string> order_;
  std::map<std::string, std::uint64_t> counts_;
};
}  // namespace aubo_sorting_core
#endif
