#pragma once
#include <string>

namespace aubo_mobile_nav_sorting
{
enum class MissionState
{
  ADJUSTING_BASE,
  ALIGNING_BASE,
  AT_WORKSTATION,
  CONFIGURING_WORKSTATION,
  DIRECT_DOCKING,
  FAILED,
  IDLE,
  INITIALIZING,
  NAVIGATING,
  PREPARING_ARM,
  RETREATING_BASE,
  SORTING,
  STOPPED,
  STOPPING,
  STOP_UNCONFIRMED,
  STOWING_ARM,
  SUCCEEDED,
  VALIDATING_DOCK,
  WORKSTATION_COMPLETE
};
enum class SortingState
{
  UNKNOWN,
  INITIALIZING,
  IDLE,
  READY,
  STOPPED,
  ERROR,
  HOMING,
  PREPARING,
  OBSERVING,
  SORTING,
  DETECTING,
  PICKING
};
inline const char *toString(MissionState state)
{
  switch (state)
  {
  case MissionState::ADJUSTING_BASE:
    return "ADJUSTING_BASE";
  case MissionState::ALIGNING_BASE:
    return "ALIGNING_BASE";
  case MissionState::AT_WORKSTATION:
    return "AT_WORKSTATION";
  case MissionState::CONFIGURING_WORKSTATION:
    return "CONFIGURING_WORKSTATION";
  case MissionState::DIRECT_DOCKING:
    return "DIRECT_DOCKING";
  case MissionState::FAILED:
    return "FAILED";
  case MissionState::IDLE:
    return "IDLE";
  case MissionState::INITIALIZING:
    return "INITIALIZING";
  case MissionState::NAVIGATING:
    return "NAVIGATING";
  case MissionState::PREPARING_ARM:
    return "PREPARING_ARM";
  case MissionState::RETREATING_BASE:
    return "RETREATING_BASE";
  case MissionState::SORTING:
    return "SORTING";
  case MissionState::STOPPED:
    return "STOPPED";
  case MissionState::STOPPING:
    return "STOPPING";
  case MissionState::STOP_UNCONFIRMED:
    return "STOP_UNCONFIRMED";
  case MissionState::STOWING_ARM:
    return "STOWING_ARM";
  case MissionState::SUCCEEDED:
    return "SUCCEEDED";
  case MissionState::VALIDATING_DOCK:
    return "VALIDATING_DOCK";
  case MissionState::WORKSTATION_COMPLETE:
    return "WORKSTATION_COMPLETE";
  }
  return "UNKNOWN";
}
inline SortingState parseSortingState(const std::string &text)
{
  const auto end = text.find('|');
  const auto first = text.find_first_not_of(" \t");
  if (first == std::string::npos)
    return SortingState::UNKNOWN;
  const auto name = text.substr(first, end == std::string::npos ? end : end - first);
  const auto token = name.substr(0, name.find_last_not_of(" \t") + 1);
  if (token == "INITIALIZING")
    return SortingState::INITIALIZING;
  if (token == "IDLE")
    return SortingState::IDLE;
  if (token == "READY")
    return SortingState::READY;
  if (token == "STOPPED")
    return SortingState::STOPPED;
  if (token == "ERROR")
    return SortingState::ERROR;
  if (token == "HOMING")
    return SortingState::HOMING;
  if (token == "PREPARING")
    return SortingState::PREPARING;
  if (token == "OBSERVING")
    return SortingState::OBSERVING;
  if (token == "SORTING")
    return SortingState::SORTING;
  if (token == "DETECTING")
    return SortingState::DETECTING;
  if (token == "PICKING")
    return SortingState::PICKING;
  return SortingState::UNKNOWN;
}
} // namespace aubo_mobile_nav_sorting
