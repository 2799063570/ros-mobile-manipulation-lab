#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace aubo_sorting_core
{
// Heights are offsets above table_z. The attempt budget includes the first try.
inline std::vector<double> liftHeightCandidates(double initial, double minimum,
                                                double step, int max_attempts)
{
  if (!std::isfinite(initial) || !std::isfinite(minimum) || !std::isfinite(step) ||
      minimum <= 0.0 || initial < minimum || step <= 0.0 ||
      max_attempts < 1 || max_attempts > 100)
    throw std::invalid_argument("invalid lift height bounds, step or attempt budget");
  std::vector<double> heights;
  for (int i = 0; i < max_attempts; ++i)
  {
    double height = std::max(minimum, initial - i * step);
    if (height - minimum < 1e-9)
      height = minimum;
    heights.push_back(height);
    if (height <= minimum)
      break;
  }
  return heights;
}

enum class HeightAttemptResult { Succeeded, PlanningFailed, ExecutionFailed, Cancelled };

template <typename Attempt, typename Cancelled>
HeightAttemptResult runHeightRecovery(const std::vector<double>& heights,
                                      Attempt attempt, Cancelled cancelled)
{
  for (const double height : heights)
  {
    if (cancelled())
      return HeightAttemptResult::Cancelled;
    const auto result = attempt(height);
    if (cancelled())
      return HeightAttemptResult::Cancelled;
    // Only a planning failure, before execution, permits changing the goal.
    if (result != HeightAttemptResult::PlanningFailed)
      return result;
  }
  return HeightAttemptResult::PlanningFailed;
}
}  // namespace aubo_sorting_core
