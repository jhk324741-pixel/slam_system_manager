#ifndef SLAM_SYSTEM_MANAGER__SYSTEM_STATE_HPP_
#define SLAM_SYSTEM_MANAGER__SYSTEM_STATE_HPP_

#include <array>
#include <string_view>
#include <utility>

namespace slam_system_manager
{

enum class SystemState
{
  BOOT,
  SYSTEM_CHECK,
  SENSOR_STARTING,
  SENSOR_READY,
  WAIT_MODE,
  MAPPING_STARTING,
  MAPPING,
  MAP_SAVING,
  LOCALIZATION_STARTING,
  RELOCALIZING,
  LOCALIZED,
  ERROR,
  RECOVERING,
  SHUTDOWN
};

inline constexpr std::string_view toString(const SystemState state) noexcept
{
  switch (state) {
    case SystemState::BOOT: return "BOOT";
    case SystemState::SYSTEM_CHECK: return "SYSTEM_CHECK";
    case SystemState::SENSOR_STARTING: return "SENSOR_STARTING";
    case SystemState::SENSOR_READY: return "SENSOR_READY";
    case SystemState::WAIT_MODE: return "WAIT_MODE";
    case SystemState::MAPPING_STARTING: return "MAPPING_STARTING";
    case SystemState::MAPPING: return "MAPPING";
    case SystemState::MAP_SAVING: return "MAP_SAVING";
    case SystemState::LOCALIZATION_STARTING: return "LOCALIZATION_STARTING";
    case SystemState::RELOCALIZING: return "RELOCALIZING";
    case SystemState::LOCALIZED: return "LOCALIZED";
    case SystemState::ERROR: return "ERROR";
    case SystemState::RECOVERING: return "RECOVERING";
    case SystemState::SHUTDOWN: return "SHUTDOWN";
  }
  return "UNKNOWN";
}

inline constexpr bool canTransition(
  const SystemState from, const SystemState to) noexcept
{
  using Transition = std::pair<SystemState, SystemState>;
  constexpr std::array<Transition, 41> allowed{{
    {SystemState::BOOT, SystemState::SYSTEM_CHECK},
    {SystemState::BOOT, SystemState::ERROR},
    {SystemState::BOOT, SystemState::SHUTDOWN},
    {SystemState::SYSTEM_CHECK, SystemState::SENSOR_STARTING},
    {SystemState::SYSTEM_CHECK, SystemState::ERROR},
    {SystemState::SYSTEM_CHECK, SystemState::SHUTDOWN},
    {SystemState::SENSOR_STARTING, SystemState::SENSOR_READY},
    {SystemState::SENSOR_STARTING, SystemState::ERROR},
    {SystemState::SENSOR_STARTING, SystemState::SHUTDOWN},
    {SystemState::SENSOR_READY, SystemState::WAIT_MODE},
    {SystemState::SENSOR_READY, SystemState::SENSOR_STARTING},
    {SystemState::SENSOR_READY, SystemState::MAPPING_STARTING},
    {SystemState::SENSOR_READY, SystemState::LOCALIZATION_STARTING},
    {SystemState::SENSOR_READY, SystemState::ERROR},
    {SystemState::SENSOR_READY, SystemState::SHUTDOWN},
    {SystemState::WAIT_MODE, SystemState::SENSOR_STARTING},
    {SystemState::WAIT_MODE, SystemState::MAPPING_STARTING},
    {SystemState::WAIT_MODE, SystemState::LOCALIZATION_STARTING},
    {SystemState::WAIT_MODE, SystemState::ERROR},
    {SystemState::WAIT_MODE, SystemState::SHUTDOWN},
    {SystemState::MAPPING_STARTING, SystemState::MAPPING},
    {SystemState::MAPPING_STARTING, SystemState::WAIT_MODE},
    {SystemState::MAPPING_STARTING, SystemState::ERROR},
    {SystemState::MAPPING_STARTING, SystemState::SHUTDOWN},
    {SystemState::MAPPING, SystemState::MAP_SAVING},
    {SystemState::MAPPING, SystemState::WAIT_MODE},
    {SystemState::MAPPING, SystemState::ERROR},
    {SystemState::MAPPING, SystemState::SHUTDOWN},
    {SystemState::MAP_SAVING, SystemState::WAIT_MODE},
    {SystemState::MAP_SAVING, SystemState::ERROR},
    {SystemState::MAP_SAVING, SystemState::SHUTDOWN},
    {SystemState::LOCALIZATION_STARTING, SystemState::RELOCALIZING},
    {SystemState::LOCALIZATION_STARTING, SystemState::WAIT_MODE},
    {SystemState::LOCALIZATION_STARTING, SystemState::ERROR},
    {SystemState::RELOCALIZING, SystemState::LOCALIZED},
    {SystemState::RELOCALIZING, SystemState::WAIT_MODE},
    {SystemState::RELOCALIZING, SystemState::ERROR},
    {SystemState::LOCALIZED, SystemState::RELOCALIZING},
    {SystemState::LOCALIZED, SystemState::WAIT_MODE},
    {SystemState::LOCALIZED, SystemState::ERROR},
    {SystemState::ERROR, SystemState::RECOVERING}
  }};

  for (const auto & transition : allowed) {
    if (transition.first == from && transition.second == to) {
      return true;
    }
  }

  if (from == SystemState::ERROR && to == SystemState::SHUTDOWN) {
    return true;
  }
  if (from == SystemState::RECOVERING &&
    (to == SystemState::SYSTEM_CHECK || to == SystemState::SENSOR_STARTING ||
    to == SystemState::WAIT_MODE || to == SystemState::ERROR ||
    to == SystemState::SHUTDOWN))
  {
    return true;
  }
  if ((from == SystemState::LOCALIZATION_STARTING ||
    from == SystemState::RELOCALIZING || from == SystemState::LOCALIZED) &&
    to == SystemState::SHUTDOWN)
  {
    return true;
  }
  return false;
}

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__SYSTEM_STATE_HPP_
