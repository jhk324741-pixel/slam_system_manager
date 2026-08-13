#ifndef SLAM_SYSTEM_MANAGER__CONFIG_MANAGER_HPP_
#define SLAM_SYSTEM_MANAGER__CONFIG_MANAGER_HPP_

#include <array>
#include <cstddef>
#include <string>

namespace slam_system_manager
{

struct FrameConfig
{
  std::string map;
  std::string odom;
  std::string base;
  std::string lidar;
};

struct TopicConfig
{
  std::string pointcloud;
  std::string imu;
  std::string initial_pose;
  std::string localization_raw_odometry;
  std::string localization_correction;
  std::string localization_odometry;
  std::string localization_pose;
  std::string system_status;
};

struct LastPoseConfig
{
  bool enable{true};
  double save_interval_sec{5.0};
};

struct RelocalizationConfig
{
  double initial_pose_wait_timeout_sec{5.0};
  double initial_pose_settle_sec{6.0};
  std::size_t last_pose_retry_count{3U};
  double last_pose_retry_interval_sec{2.0};
  std::size_t min_initial_pose_subscribers{2U};
};

struct ExtrinsicConfig
{
  std::array<double, 3> translation{};
  std::array<double, 3> rotation_rpy{};
};

struct SystemConfig
{
  std::string map_root;
  FrameConfig frames;
  TopicConfig topics;
  LastPoseConfig last_pose;
  RelocalizationConfig relocalization;
  ExtrinsicConfig lidar_to_base;
  double status_publish_hz{2.0};
};

class ConfigManager
{
public:
  void load(const std::string & path);
  const SystemConfig & get() const;
  bool isLoaded() const noexcept;

private:
  SystemConfig config_;
  bool loaded_{false};
};

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__CONFIG_MANAGER_HPP_
