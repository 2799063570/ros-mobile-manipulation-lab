#ifndef AUBO_MOBILE_NAV_SORTING_WORKSTATION_PROGRESS_H
#define AUBO_MOBILE_NAV_SORTING_WORKSTATION_PROGRESS_H

#include <string>
#include <vector>

namespace aubo_mobile_nav_sorting
{

enum class WorkstationProgress { PENDING, ACTIVE, COMPLETE, FAILED, STOPPED,
                                 STOP_UNCONFIRMED, DISABLED };

inline void advanceWorkstationProgress(const std::vector<std::string>& ids,
                                       std::vector<WorkstationProgress>& progress,
                                       const std::string& code,
                                       const std::string& detail)
{
  if (ids.size() != progress.size())
    return;

  if (code == "IDLE")
  {
    for (auto& status : progress)
      if (status != WorkstationProgress::DISABLED)
        status = WorkstationProgress::PENDING;
    return;
  }
  if (code == "SUCCEEDED")
  {
    for (auto& status : progress)
      if (status != WorkstationProgress::DISABLED)
        status = WorkstationProgress::COMPLETE;
    return;
  }
  if (code == "FAILED" || code == "STOPPED" || code == "STOP_UNCONFIRMED")
  {
    for (auto& status : progress)
      if (status == WorkstationProgress::ACTIVE ||
          status == WorkstationProgress::STOP_UNCONFIRMED)
        status = code == "FAILED" ? WorkstationProgress::FAILED :
                 code == "STOP_UNCONFIRMED" ? WorkstationProgress::STOP_UNCONFIRMED :
                 WorkstationProgress::STOPPED;
    return;
  }

  const bool complete = code == "WORKSTATION_COMPLETE";
  const bool active = code == "CONFIGURING_WORKSTATION" || code == "NAVIGATING" ||
                      code == "DIRECT_DOCKING" || code == "AT_WORKSTATION" ||
                      code == "SORTING";
  if (!complete && !active)
    return;

  for (std::size_t current = 0; current < ids.size(); ++current)
  {
    const bool bare_id = detail == ids[current] ||
                         detail.compare(0, ids[current].size() + 1, ids[current] + ";") == 0;
    const bool quoted_id = detail.find("workstation '" + ids[current] + "'") != std::string::npos;
    if (!bare_id && !quoted_id)
      continue;
    // The mission visits enabled workstations in order. Reconstruct earlier
    // completions from the current state even if brief transition messages were missed.
    for (std::size_t row = 0; row < progress.size(); ++row)
    {
      if (progress[row] == WorkstationProgress::DISABLED)
        continue;
      progress[row] = row < current || (complete && row == current)
                          ? WorkstationProgress::COMPLETE
                          : row == current ? WorkstationProgress::ACTIVE
                                           : WorkstationProgress::PENDING;
    }
    return;
  }
}

}  // namespace aubo_mobile_nav_sorting

#endif  // AUBO_MOBILE_NAV_SORTING_WORKSTATION_PROGRESS_H
